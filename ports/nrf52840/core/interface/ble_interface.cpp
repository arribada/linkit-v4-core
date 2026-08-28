#include "ble_interface.hpp"

#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "ble_stm_ota.h"
#include "error.hpp"
#include "debug.hpp"
#include "timer.hpp"


#define APP_BLE_CONN_CFG_TAG 1 /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME          "<unknown>" /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE \
	BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the Nordic UART Service (vendor specific). */
#define STM_OTA_SERVICE_UUID_TYPE \
	BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the STM OTA Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL 64 /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED /**< The advertising duration */

#define MIN_CONN_INTERVAL \
	MSEC_TO_UNITS(        \
	    20,               \
	    UNIT_1_25_MS) /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL \
	MSEC_TO_UNITS(        \
	    400,              \
	    UNIT_1_25_MS)   /**< Maximum acceptable connection interval (400 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY 0 /**< Slave latency. */
#define CONN_SUP_TIMEOUT \
	MSEC_TO_UNITS(       \
	    4000, UNIT_10_MS) /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY \
	APP_TIMER_TICKS(                   \
	    5000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY \
	APP_TIMER_TICKS(                  \
	    30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3 /**< Number of attempts before giving up the connection parameter negotiation. */

#define TRANSFER_TIMEOUT             10000 /** 10 seconds transfer timeout */

/**< BLE NUS service instance. */
// These next few lines are equivalent to BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT) but is necessary for C++ compilation
BLE_LINK_CTX_MANAGER_DEF(m_nus_link_ctx_storage, (1), sizeof(ble_nus_client_context_t));
static ble_nus_t m_nus = {
	.uuid_type = 0,
	.service_handle = 0,
	.tx_handles = { 0, 0, 0, 0 },
	.rx_handles = { 0, 0, 0, 0 },
	.p_link_ctx_storage = &m_nus_link_ctx_storage,
	.data_handler = nullptr,
};
NRF_SDH_BLE_OBSERVER(m_nus_obs, BLE_NUS_BLE_OBSERVER_PRIO, ble_nus_on_ble_evt, &m_nus);

BLE_STM_OTA_DEF(m_stm_ota);

NRF_BLE_GATT_DEF(m_gatt);           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising); /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
static uint16_t m_ble_nus_max_data_len =
    BLE_GATT_ATT_MTU_DEFAULT
    - 3; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[] = /**< Universally unique service identifier. */
    {
	    { BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE },
    };

extern Timer *system_timer;


void BleInterface::init() {
	m_carriage_return_received = false;
	m_receive_buffer_len = 0;
	m_is_first_ota_packet = false;

	ble_stack_init();
	gap_params_init();
	gatt_init();
	services_init();
}

void BleInterface::start(std::function<int(BLEServiceEvent &event)> on_event) {
	m_on_event = on_event;
}

void BleInterface::stop() {
	advertising_stop();

	// Disconnect if we were connected
	if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
		NRF_LOG_DEBUG("Disconnecting...");

		ret_code_t err_code;
		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

		if (err_code != NRF_SUCCESS) NRF_LOG_ERROR("sd_ble_gap_disconnect() failed: 0x%08lX", err_code);
	}

	// Discard any received data
	m_carriage_return_received = false;
	m_receive_buffer_len = 0;
	m_is_first_ota_packet = false;

	// Deregister the owning state's callback. It was posted by start() and,
	// until now, NEVER cleared — so ConfigurationState's handler stayed live
	// for the rest of the device's life. Any later connection re-entered it
	// from OperationalState, where it re-armed the 20-minute BLE inactivity
	// timer (which then forced Operational -> PreOperational -> Operational,
	// seen in the field on 2026-08-24 08:35:07), hijacked the DTE async writer
	// onto BLE, and drove the configuration LED states. The interface's own
	// housekeeping (m_conn_handle, RX buffers) runs before this dispatch in
	// ble_evt_handler, so dropping the callback here loses nothing.
	m_on_event = nullptr;
}

std::string BleInterface::read_line() {
	if (m_carriage_return_received && m_receive_buffer_len) {
		std::string str((char *)m_receive_buffer, m_receive_buffer_len);
		m_receive_buffer_len = 0;
		m_carriage_return_received = false;
		return str;
	}

	return std::string();
}

void BleInterface::set_device_name(const std::string &name) {
	ret_code_t err_code;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
	err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)name.c_str(), name.size());
	APP_ERROR_CHECK(err_code);

	// Setup advertising parameters
	advertising_stop();
	advertising_init();
	advertising_start();
}

bool BleInterface::write_best_effort(std::string str) {
	uint8_t *buffer = reinterpret_cast<uint8_t *>(const_cast<char *>(str.c_str()));
	size_t length = str.length();
	while (length) {
		uint16_t transfer_length = std::min(length, static_cast<size_t>(BLE_NUS_MAX_DATA_LEN));
		uint32_t err = ble_nus_data_send(&m_nus, buffer, &transfer_length, m_conn_handle);
		if (err == NRF_SUCCESS) {
			length -= transfer_length;
			buffer += transfer_length;
		} else {
			return false;  // Drop remaining data — no retry, no blocking
		}
	}
	return true;
}

bool BleInterface::write(std::string str) {
	uint32_t err_code;

	size_t length = str.length();

	// WARN: Unfortunately the ble_nus_data_send() library incorrectly requires a non-const data pointer
	// By examining the code we can see this is incorrect and at least for SDK v17.0.2 it is safe to cast away the const
	uint8_t *buffer = reinterpret_cast<uint8_t *>(const_cast<char *>(str.c_str()));

	// Record start time of send
	uint64_t start_t = system_timer->get_counter();

	do {
		uint16_t transfer_length = std::min(length, static_cast<size_t>(BLE_NUS_MAX_DATA_LEN));

		// WARN: Unfortunately the ble_nus_data_send() library incorrectly requires a non-const data pointer
		// By examining the code we can see this is incorrect and at least for SDK v17.0.2 it is safe to cast away the const
		err_code = ble_nus_data_send(&m_nus, buffer, &transfer_length, m_conn_handle);
		if (err_code != NRF_ERROR_RESOURCES && err_code != NRF_SUCCESS) {
			//DEBUG_ERROR("BleInterface::write: error_code=%08x", err_code);
			return false;
		}

		if (err_code == NRF_SUCCESS) {
			length -= transfer_length;
			buffer += transfer_length;
		}

		//DEBUG_TRACE("BleInterface::write: error_code=%08x remaining=%u", err_code, (unsigned int)length);

		// Check for timeout
		if (length && (system_timer->get_counter() - start_t) >= TRANSFER_TIMEOUT) return false;

	} while (length);

	return true;
}

void BleInterface::ble_stack_init() {
	ret_code_t err_code;
	err_code = nrf_sdh_enable_request();
	APP_ERROR_CHECK(err_code);

	// Configure the BLE stack using the default settings.
	// Fetch the start address of the application RAM.
	uint32_t ram_start = 0;
	err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
	APP_ERROR_CHECK(err_code);

	// Enable BLE stack.
	err_code = nrf_sdh_ble_enable(&ram_start);
	APP_ERROR_CHECK(err_code);

	// Register a handler for BLE events.
	NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, static_ble_evt_handler, NULL);
}

void BleInterface::gap_params_init() {
	uint32_t err_code;
	ble_gap_conn_params_t gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
	err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);
}

void BleInterface::gatt_init() {
	ret_code_t err_code;

	err_code = nrf_ble_gatt_init(&m_gatt, static_gatt_evt_handler);
	APP_ERROR_CHECK(err_code);

	err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
	APP_ERROR_CHECK(err_code);
}

void BleInterface::services_init() {
	uint32_t err_code;
	ble_nus_init_t nus_init;
	ble_stm_ota_init_t stm_ota_init;
	nrf_ble_qwr_init_t qwr_init = { 0 };

	// Initialize Queued Write Module.
	qwr_init.error_handler = static_nrf_qwr_error_handler;

	err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
	APP_ERROR_CHECK(err_code);

	// Initialize NUS.
	memset(&nus_init, 0, sizeof(nus_init));

	nus_init.data_handler = static_nus_data_handler;

	err_code = ble_nus_init(&m_nus, &nus_init);
	APP_ERROR_CHECK(err_code);

	// Initialize STM OTA.
	memset(&m_stm_ota, 0, sizeof(m_stm_ota));

	stm_ota_init.event_handler = static_stm_ota_data_handler;

	err_code = ble_stm_ota_init(&m_stm_ota, &stm_ota_init);
	APP_ERROR_CHECK(err_code);
}

void BleInterface::advertising_init() {
	uint32_t err_code;
	ble_advertising_init_t init;

	memset(&init, 0, sizeof(init));

	init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
	init.advdata.include_appearance = false;
	init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

	init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	init.srdata.uuids_complete.p_uuids = m_adv_uuids;

	init.config.ble_adv_fast_enabled = true;
	init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
	init.config.ble_adv_fast_timeout = APP_ADV_DURATION;
	// The SDK module restarts advertising all by itself on
	// BLE_GAP_EVT_DISCONNECTED (on_disconnected -> ble_advertising_start).
	// Because the disconnection requested by stop() is ASYNCHRONOUS, that event
	// arrives AFTER we have left configuration mode: the beacon would start
	// transmitting again indefinitely (APP_ADV_DURATION = UNLIMITED). We cut
	// this implicit restart and redo it EXPLICITLY in our own handler, only if
	// advertising is still wanted.
	init.config.ble_adv_on_disconnect_disabled = true;
	init.evt_handler = static_on_adv_evt;

	DEBUG_TRACE("ble_advertising_init");
	err_code = ble_advertising_init(&m_advertising, &init);
	APP_ERROR_CHECK(err_code);

	ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

#ifdef BENCH_TEST
/// @brief Bench: is advertising wanted, and which mode does the SDK module believe is active?
bool BleInterface::bench_is_advertising() {
	// OUR intention only. Do not mix adv_mode_current into it: that field stays
	// stuck on FAST after sd_ble_gap_adv_stop(), so an AND/OR with it always
	// yields a positive verdict — a gauge unable to observe a stop, one that
	// declared the fix a failure while it was in fact working.
	return m_advertising_wanted;
}

int BleInterface::bench_adv_mode() {
	return (int)m_advertising.adv_mode_current;
}

unsigned int BleInterface::bench_probe_advertising() {
	// The only authoritative measurement: we ask the SoftDevice to stop.
	// NRF_SUCCESS -> it really was advertising; NRF_ERROR_INVALID_STATE -> it was not.
	return (unsigned int)sd_ble_gap_adv_stop(m_advertising.adv_handle);
}

/// @brief Bench: replays the BLE disconnection without a phone.
/// It is THIS sequence that left the beacon advertising permanently: stop()
/// requests the disconnection, the event arrives after we have left config mode,
/// and the SDK module restarted advertising. We reproduce it identically here by
/// passing the event to the SDK module and then to our own handler.
void BleInterface::bench_inject_disconnect() {
	ble_evt_t evt;
	memset(&evt, 0, sizeof(evt));
	evt.header.evt_id = BLE_GAP_EVT_DISCONNECTED;
	evt.evt.gap_evt.conn_handle = m_advertising.current_slave_link_conn_handle;
	evt.evt.gap_evt.params.disconnected.reason = BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION;
	ble_advertising_on_ble_evt(&evt, &m_advertising);
	ble_evt_handler(&evt, nullptr);
}
#endif

void BleInterface::advertising_start() {
	m_advertising_wanted = true;
	uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
	// No APP_ERROR_CHECK here: it causes a FULL RESET of the beacon. Yet the
	// most likely error is NRF_ERROR_INVALID_STATE (advertising already
	// active), which is perfectly benign. Rebooting a sealed device out on a
	// mission for that would be worse than the problem.
	if (err_code != NRF_SUCCESS)
		DEBUG_ERROR("BleInterface::advertising_start: ble_advertising_start=0x%08lX", (unsigned long)err_code);
}

void BleInterface::advertising_stop() {
	m_advertising_wanted = false;
	uint32_t err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
	// NRF_ERROR_INVALID_STATE = we were not advertising: normal, we say nothing.
	// The return code used to be completely ignored, so a real failure went
	// unnoticed even though it leaves the radio powered on.
	if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
		DEBUG_ERROR("BleInterface::advertising_stop: sd_ble_gap_adv_stop=0x%08lX", (unsigned long)err_code);
}

void BleInterface::ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
	uint32_t err_code;

	switch (p_ble_evt->header.evt_id) {
	case BLE_GAP_EVT_CONNECTED:
		NRF_LOG_INFO("Connected");
		//err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
		//APP_ERROR_CHECK(err_code);
		m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
		APP_ERROR_CHECK(err_code);
		if (m_on_event) {
			BLEServiceEvent e;
			e.event_type = BLEServiceEventType::CONNECTED;
			(void)m_on_event(e);
		}
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		NRF_LOG_INFO("Disconnected for reason 0x%02X", p_ble_evt->evt.gap_evt.params.disconnected.reason);
		// LED indication will be changed when advertising starts.
		m_conn_handle = BLE_CONN_HANDLE_INVALID;

		// Discard any received data
		m_carriage_return_received = false;
		m_receive_buffer_len = 0;
		m_is_first_ota_packet = false;

		if (m_on_event) {
			BLEServiceEvent e;
			e.event_type = BLEServiceEventType::DISCONNECTED;
			(void)m_on_event(e);
		}
		// Explicit restart (the SDK no longer does it, cf. advertising_init):
		// in configuration mode, a phone that disconnects must be able to
		// reconnect; after a stop(), the radio must stay off.
		if (m_advertising_wanted) advertising_start();
		break;

	case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
		NRF_LOG_DEBUG("PHY update request.");
		ble_gap_phys_t const phys = {
			.tx_phys = BLE_GAP_PHY_AUTO,
			.rx_phys = BLE_GAP_PHY_AUTO,
		};
		err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
		APP_ERROR_CHECK(err_code);
	} break;

	case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		// Pairing not supported
		err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GATTS_EVT_SYS_ATTR_MISSING:
		// No system attributes have been stored.
		err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GATTC_EVT_TIMEOUT:
		// Disconnect on GATT Client timeout event.
		err_code =
		    sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GATTS_EVT_TIMEOUT:
		// Disconnect on GATT Server timeout event.
		err_code =
		    sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		APP_ERROR_CHECK(err_code);
		break;

	default:
		// No implementation needed.
		break;
	}
}

/**@brief Function for handling events from the GATT library. */
void BleInterface::gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt) {
	if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) {
		m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
		NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len);
	}
	NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x", p_gatt->att_mtu_desired_central,
	              p_gatt->att_mtu_desired_periph);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
void BleInterface::nrf_qwr_error_handler(uint32_t nrf_error) {
	APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the events from the STM OTA BLE Service.
 *
 * @details This function will process the events received from the STM OTA BLE Service
 *
 * @param[in] p_evt       STM OTA Service event.
 */
/**@snippet [Handling the data received over BLE] */
void BleInterface::stm_ota_event_handler(uint16_t conn_handle, ble_stm_ota_t *p_stm_ota, ble_stm_ota_event_t *p_evt) {
	// No owner registered — bail out before touching m_on_event.
	//
	// This runs in SoftDevice interrupt context, reached through the plain-C
	// frames of ble_stm_ota.c. Those frames are EXIDX_CANTUNWIND, so an
	// exception thrown below cannot unwind through them and lands straight in
	// std::terminate. Every m_on_event() call further down is either bare or
	// wrapped in `catch (ErrorCode)`, which does NOT catch the
	// std::bad_function_call that an empty std::function throws.
	//
	// Before stop() started clearing m_on_event this could not happen: the
	// callback was set for the lifetime of the device. Now it can — a reed
	// swipe out of configuration while a phone still has OTA writes in flight
	// leaves a few hundred ms during which queued write-without-response
	// packets still arrive. Reply "interrupted" so the peer fails cleanly.
	if (!m_on_event) {
		uint8_t status[3] = { STM_OTA_STATUS_FILE_RECEPTION_INTERRUPTED, STM_OTA_STATUS_IGNORE, STM_OTA_STATUS_IGNORE };
		ble_stm_ota_on_file_upload_status(conn_handle, p_stm_ota, status);
		return;
	}

	BLEServiceEvent s_evt;
	switch (p_evt->event_type) {
	case STM_OTA_EVENT_ACTION: {
		if (p_evt->action == STM_OTA_ACTION_START_WIRELESS || p_evt->action == STM_OTA_ACTION_START_USER_APP) {
			// The next receive raw data packet will contain at least 8 bytes
			// which shall include the total file size and a CRC32 checksum
			m_is_first_ota_packet = true;
			m_ota_file_id = p_evt->address;  // Record file ID

			// Send handshake back to client to inform them that we receive their
			// start request
			uint8_t status[3] = { STM_OTA_STATUS_IGNORE, STM_OTA_STATUS_IGNORE, STM_OTA_STATUS_START_UPLOAD_OK };
			ble_stm_ota_on_file_upload_status(conn_handle, p_stm_ota, status);
		} else if (p_evt->action == STM_OTA_ACTION_STOP_ALL || p_evt->action == STM_OTA_ACTION_CANCEL) {
			s_evt.event_type = BLEServiceEventType::OTA_ABORT;
			m_on_event(s_evt);
			uint8_t status[3] = { STM_OTA_STATUS_FILE_RECEPTION_INTERRUPTED, STM_OTA_STATUS_IGNORE,
			                      STM_OTA_STATUS_IGNORE };
			ble_stm_ota_on_file_upload_status(conn_handle, p_stm_ota, status);
		} else if (p_evt->action == STM_OTA_ACTION_UPLOAD_FINISHED) {
			s_evt.event_type = BLEServiceEventType::OTA_END;
			try {
				m_on_event(s_evt);
				uint8_t status[3] = { STM_OTA_STATUS_FILE_RECEPTION_OK, STM_OTA_STATUS_INTEGRITY_OK,
				                      STM_OTA_STATUS_IGNORE };
				ble_stm_ota_on_file_upload_status(conn_handle, p_stm_ota, status);
			} catch (ErrorCode e) {
				DEBUG_TRACE("OTA_END: Got error: %u", e);
				if (OTA_TRANSFER_CRC_ERROR == e) {
					uint8_t status[3] = { STM_OTA_STATUS_FILE_RECEPTION_OK, STM_OTA_STATUS_INTEGRITY_NOT_OK,
					                      STM_OTA_STATUS_IGNORE };
					ble_stm_ota_on_file_upload_status(conn_handle, p_stm_ota, status);
					break;
				} else {
					uint8_t status[3] = { STM_OTA_STATUS_FILE_RECEPTION_INTERRUPTED, STM_OTA_STATUS_IGNORE,
					                      STM_OTA_STATUS_IGNORE };
					ble_stm_ota_on_file_upload_status(conn_handle, p_stm_ota, status);
				}
			}
		}
	} break;
	case STM_OTA_EVENT_RAW_DATA: {
		if (m_is_first_ota_packet) {
			// If the length is less than 8 bytes then flag an error
			if (p_evt->length < 8) {
				uint8_t status[3] = { STM_OTA_STATUS_FILE_RECEPTION_INTERRUPTED, STM_OTA_STATUS_IGNORE,
				                      STM_OTA_STATUS_IGNORE };
				ble_stm_ota_on_file_upload_status(conn_handle, p_stm_ota, status);
				return;
			}

			m_is_first_ota_packet = false;

			// Debug: print first 16 bytes of OTA packet
			DEBUG_TRACE("OTA first packet (%u bytes): %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X "
			            "%02X %02X %02X",
			            p_evt->length, p_evt->length > 0 ? ((uint8_t *)p_evt->p_data)[0] : 0,
			            p_evt->length > 1 ? ((uint8_t *)p_evt->p_data)[1] : 0,
			            p_evt->length > 2 ? ((uint8_t *)p_evt->p_data)[2] : 0,
			            p_evt->length > 3 ? ((uint8_t *)p_evt->p_data)[3] : 0,
			            p_evt->length > 4 ? ((uint8_t *)p_evt->p_data)[4] : 0,
			            p_evt->length > 5 ? ((uint8_t *)p_evt->p_data)[5] : 0,
			            p_evt->length > 6 ? ((uint8_t *)p_evt->p_data)[6] : 0,
			            p_evt->length > 7 ? ((uint8_t *)p_evt->p_data)[7] : 0,
			            p_evt->length > 8 ? ((uint8_t *)p_evt->p_data)[8] : 0,
			            p_evt->length > 9 ? ((uint8_t *)p_evt->p_data)[9] : 0,
			            p_evt->length > 10 ? ((uint8_t *)p_evt->p_data)[10] : 0,
			            p_evt->length > 11 ? ((uint8_t *)p_evt->p_data)[11] : 0,
			            p_evt->length > 12 ? ((uint8_t *)p_evt->p_data)[12] : 0,
			            p_evt->length > 13 ? ((uint8_t *)p_evt->p_data)[13] : 0,
			            p_evt->length > 14 ? ((uint8_t *)p_evt->p_data)[14] : 0,
			            p_evt->length > 15 ? ((uint8_t *)p_evt->p_data)[15] : 0);

			s_evt.event_type = BLEServiceEventType::OTA_START;
			s_evt.file_id = m_ota_file_id;
			s_evt.file_size = ((uint8_t *)p_evt->p_data)[3] | ((uint8_t *)p_evt->p_data)[2] << 8
			                  | ((uint8_t *)p_evt->p_data)[1] << 16 | ((uint8_t *)p_evt->p_data)[0] << 24;
			s_evt.crc32 = ((uint8_t *)p_evt->p_data)[7] | ((uint8_t *)p_evt->p_data)[6] << 8
			              | ((uint8_t *)p_evt->p_data)[5] << 16 | ((uint8_t *)p_evt->p_data)[4] << 24;

			// Notify start event
			try {
				m_on_event(s_evt);
			} catch (ErrorCode e) {
				// Exception raised, indicate the procedure failed
				DEBUG_TRACE("OTA_FILE_DATA: Got error: %u", e);
				uint8_t status[3] = { STM_OTA_STATUS_FILE_RECEPTION_INTERRUPTED, STM_OTA_STATUS_IGNORE,
				                      STM_OTA_STATUS_IGNORE };
				ble_stm_ota_on_file_upload_status(conn_handle, p_stm_ota, status);
			}

			// If any bytes remain then pass these on as file data
			if (p_evt->length > 8) {
				s_evt.event_type = BLEServiceEventType::OTA_FILE_DATA;
				s_evt.data = (uint8_t *)p_evt->p_data + 8;
				s_evt.length = p_evt->length - 8;

				// Notify file data
				try {
					m_on_event(s_evt);
				} catch (ErrorCode e) {
					// Exception raised, indicate the procedure failed
					DEBUG_TRACE("OTA_FILE_DATA: Got error: %u", e);
					uint8_t status[3] = { STM_OTA_STATUS_FILE_RECEPTION_INTERRUPTED, STM_OTA_STATUS_IGNORE,
					                      STM_OTA_STATUS_IGNORE };
					ble_stm_ota_on_file_upload_status(conn_handle, p_stm_ota, status);
				}
			}
		} else {
			s_evt.event_type = BLEServiceEventType::OTA_FILE_DATA;
			s_evt.data = p_evt->p_data;
			s_evt.length = p_evt->length;

			try {
				m_on_event(s_evt);
			} catch (ErrorCode e) {
				// Exception raised, indicate the procedure failed
				DEBUG_TRACE("OTA_FILE_DATA: Got error: %u", e);
				uint8_t status[3] = { STM_OTA_STATUS_FILE_RECEPTION_INTERRUPTED, STM_OTA_STATUS_IGNORE,
				                      STM_OTA_STATUS_IGNORE };
				ble_stm_ota_on_file_upload_status(conn_handle, p_stm_ota, status);
			}
		}
	} break;
	default: break;
	}
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
void BleInterface::nus_data_handler(ble_nus_evt_t *p_evt) {
	if (p_evt->type == BLE_NUS_EVT_RX_DATA) {
		if (!m_carriage_return_received) {
			// If receiving this data would overrun our buffer then we need to discard what we already have to prevent the buffer from filling and not being cleared
			if (m_receive_buffer_len + p_evt->params.rx_data.length > sizeof(m_receive_buffer))
				m_receive_buffer_len = 0;

			if (p_evt->params.rx_data.length > sizeof(m_receive_buffer)) {
				DEBUG_WARN("BleInterface::nus_data_handler: received data from BLE NUS larger than buffer.");
			} else {
				memcpy(&m_receive_buffer[m_receive_buffer_len], p_evt->params.rx_data.p_data,
				       p_evt->params.rx_data.length);
				m_receive_buffer_len = m_receive_buffer_len + p_evt->params.rx_data.length;
				DEBUG_TRACE("BleInterface::nus_data_handler: received %u bytes cumulative %u (%03x).",
				            p_evt->params.rx_data.length, m_receive_buffer_len, m_receive_buffer_len);
				if (m_receive_buffer_len > 0 && m_receive_buffer[m_receive_buffer_len - 1] == '\r')
					m_carriage_return_received = true;

				// Notify the user that we have a string waiting to be read with readline()
				// only if end line is received
				if (m_on_event && m_carriage_return_received) {
					BLEServiceEvent e;
					e.event_type = BLEServiceEventType::DTE_DATA_RECEIVED;
					(void)m_on_event(e);
				}
			}
		} else {
			DEBUG_WARN("BleInterface::nus_data_handler: Received data from BLE NUS. But discarded as buffer already "
			           "contains a string.");
		}

#if 0
        DEBUG_TRACE("BleInterface::nus_data_handler: BLE packet:");
        for (unsigned int i = 0; i < p_evt->params.rx_data.length; i++)
        	printf("%c", p_evt->params.rx_data.p_data[i]);
        printf("\n");
        for (unsigned int i = 0; i < p_evt->params.rx_data.length; i++)
        	printf("%02x ", p_evt->params.rx_data.p_data[i]);
        printf("\n");
        //NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
#endif
	}
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
void BleInterface::on_adv_evt(ble_adv_evt_t ble_adv_evt) {
	switch (ble_adv_evt) {
	case BLE_ADV_EVT_FAST:
		NRF_LOG_INFO("Start advertising");
		//err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
		//APP_ERROR_CHECK(err_code);
		break;
	case BLE_ADV_EVT_IDLE:
		// Second chemin de resurrection: ce rearmement etait inconditionnel.
		if (m_advertising_wanted) advertising_start();
		break;
	case BLE_ADV_EVT_DIRECTED_HIGH_DUTY:
	case BLE_ADV_EVT_DIRECTED:
	case BLE_ADV_EVT_SLOW:
	case BLE_ADV_EVT_FAST_WHITELIST:
	case BLE_ADV_EVT_SLOW_WHITELIST:
	case BLE_ADV_EVT_WHITELIST_REQUEST:
	case BLE_ADV_EVT_PEER_ADDR_REQUEST:
	default: break;
	}
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
void BleInterface::on_conn_params_evt(ble_conn_params_evt_t *p_evt) {
	uint32_t err_code;

	if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
		APP_ERROR_CHECK(err_code);
	}
}

/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
void BleInterface::conn_params_error_handler(uint32_t nrf_error) {
	APP_ERROR_HANDLER(nrf_error);
}
