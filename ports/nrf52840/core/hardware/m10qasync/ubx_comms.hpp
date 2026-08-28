/**
 * @file ubx_comms.hpp
 * @brief ubx comms driver.
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <functional>
#include "ubx.hpp"
#include "events.hpp"
#include "error.hpp"
#include "debug.hpp"
#include "filesystem.hpp"
#include "timeutils.hpp"

class AsyncLowLevel;

enum UBXCommsMgaDbdStatus { SUCCESS, INCORRECT_NUM_MESSAGES };
struct UBXCommsEventNavReport {
	UBX::NAV::PVT::MSG_PVT pvt;
	UBX::NAV::STATUS::MSG_STATUS status;
	UBX::NAV::DOP::MSG_DOP dop;
};
struct UBXCommsEventSatReport {
	UBX::NAV::SAT::MSG_SAT sat;
};
struct UBXCommsEventSent {};
struct UBXCommsEventError {
	unsigned int error_type;
	UBXCommsEventError(unsigned int a) : error_type(a) {}
};
struct UBXCommsEventDebug {
	uint8_t *buffer;
	unsigned int length;
	UBXCommsEventDebug(uint8_t *a, unsigned int b) : buffer(a), length(b) {}
};
struct UBXCommsEventRespTimeout {};
struct UBXCommsEventAckNack {
	bool ack;
	UBXCommsEventAckNack(bool a) : ack(a) {}
};
struct UBXCommsEventCfgValget {
	uint8_t *msg;
	unsigned int length;
	UBXCommsEventCfgValget(uint8_t *a, unsigned int b) : msg(a), length(b) {}
};
struct UBXCommsEventMgaAck {
	bool ack;
	unsigned int num_dbd_messages;
	UBXCommsEventMgaAck(bool a, unsigned int b) : ack(a), num_dbd_messages(b) {}
};
struct UBXCommsEventMgaDBD {
	uint8_t *const database;
	unsigned int length;
	UBXCommsEventMgaDBD(uint8_t *const a, unsigned int b) : database(a), length(b) {}
};

struct UBXCommsEventRawMeasurement {
	uint8_t measc12[12];
	uint8_t meas20[20];
	uint8_t meas50[50];
	bool has_measc12;
	bool has_meas20;
	bool has_meas50;
	UBXCommsEventRawMeasurement() : has_measc12(false), has_meas20(false), has_meas50(false) {
		std::memset(measc12, 0, sizeof(measc12));
		std::memset(meas20, 0, sizeof(meas20));
		std::memset(meas50, 0, sizeof(meas50));
	}
};

struct UBXCommsEventSendComplete {};

struct UBXCommsEventMonVer {
	char swVersion[30];
	char hwVersion[10];
	UBXCommsEventMonVer(const uint8_t *payload, unsigned int len) {
		std::memset(swVersion, 0, sizeof(swVersion));
		std::memset(hwVersion, 0, sizeof(hwVersion));
		if (len >= sizeof(UBX::MON::MSG_VER)) {
			const UBX::MON::MSG_VER *ver = (const UBX::MON::MSG_VER *)payload;
			std::memcpy(swVersion, ver->swVersion, sizeof(swVersion));
			std::memcpy(hwVersion, ver->hwVersion, sizeof(hwVersion));
		}
	}
};

struct UBXCommsEventSecUniqId {
	uint8_t uniqueId[5];
	UBXCommsEventSecUniqId(const uint8_t *payload, unsigned int len) {
		std::memset(uniqueId, 0, sizeof(uniqueId));
		if (len >= sizeof(UBX::SEC::MSG_UNIQID)) {
			const UBX::SEC::MSG_UNIQID *uid = (const UBX::SEC::MSG_UNIQID *)payload;
			std::memcpy(uniqueId, uid->uniqueId, sizeof(uniqueId));
		}
	}
};

class UBXCommsEventListener {
public:
	virtual ~UBXCommsEventListener() {}
	virtual void react(const UBXCommsEventSendComplete &) {}
	virtual void react(const UBXCommsEventAckNack &) {}
	virtual void react(const UBXCommsEventCfgValget &) {}
	virtual void react(const UBXCommsEventMgaAck &) {}
	virtual void react(const UBXCommsEventNavReport &) {}
	virtual void react(const UBXCommsEventSatReport &) {}
	virtual void react(const UBXCommsEventMgaDBD &) {}
	virtual void react(const UBXCommsEventMonVer &) {}
	virtual void react(const UBXCommsEventSecUniqId &) {}
	virtual void react(const UBXCommsEventRawMeasurement &) {}
	virtual void react(const UBXCommsEventDebug &) {}
	virtual void react(const UBXCommsEventError &) {}
};

class UBXComms : public EventEmitter<UBXCommsEventListener> {
public:
	UBXComms(unsigned int libuarte_async_instance = 0);
	template <typename T>
	void send_packet_with_expect(UBX::MessageClass cls, uint8_t id, const T &content,
	                             UBX::MessageClass resp_cls = UBX::MessageClass::MSG_CLASS_ACK,
	                             uint8_t resp_msg_id = UBX::ACK::ID_ACK, size_t dynamic_size = 0) {
		unsigned int content_size;
		if constexpr (std::is_same<T, UBX::Empty>::value) {
			content_size = 0;
		} else {
			content_size = sizeof(content) + dynamic_size;
		}

		wait_tx_idle();

		UBX::HeaderAndPayloadCRC *msg = (UBX::HeaderAndPayloadCRC *)m_tx_buffer;
		msg->syncChars[0] = UBX::SYNC_CHAR1;
		msg->syncChars[1] = UBX::SYNC_CHAR2;
		msg->msgClass = cls;
		msg->msgId = id;
		msg->msgLength = content_size;
		std::memcpy(msg->payload, &content, content_size);

		compute_crc((const uint8_t *const)&msg->msgClass, content_size + sizeof(UBX::Header) - 2,
		            msg->payload[msg->msgLength], msg->payload[msg->msgLength + 1]);
		send_with_expect(m_tx_buffer, content_size + sizeof(UBX::Header) + 2, resp_cls, resp_msg_id);
	}

	void init();
	void deinit();

	// Send raw bytes to UART (for bridge/passthrough mode)
	bool send_raw(const uint8_t *data, size_t len);

	// Bridge/passthrough mode: forward raw UART RX to callback instead of parsing UBX
	using PassthroughCallback = std::function<void(const uint8_t *, size_t)>;
	void set_passthrough(bool active, PassthroughCallback callback = nullptr);
	bool is_passthrough() const { return m_passthrough_active; }

	// Call from main context to forward ISR-buffered passthrough data
	void process_passthrough_rx();

	template <typename T> void send_packet(UBX::MessageClass cls, uint8_t id, T content) {
		wait_tx_idle();

		unsigned int content_size;
		if constexpr (std::is_same<T, UBX::Empty>::value) {
			content_size = 0;
		} else {
			content_size = sizeof(content);
		};
		UBX::HeaderAndPayloadCRC *msg = (UBX::HeaderAndPayloadCRC *)m_tx_buffer;
		msg->syncChars[0] = UBX::SYNC_CHAR1;
		msg->syncChars[1] = UBX::SYNC_CHAR2;
		msg->msgClass = cls;
		msg->msgId = id;
		msg->msgLength = content_size;
		std::memcpy(msg->payload, &content, content_size);
		compute_crc((const uint8_t *const)&msg->msgClass, content_size + sizeof(UBX::Header) - 2,
		            msg->payload[msg->msgLength], msg->payload[msg->msgLength + 1]);
		send(m_tx_buffer, content_size + sizeof(UBX::Header) + 2);
	}

	void filter_buffer(uint8_t *buffer, unsigned int length);
	void cancel_expect();
	void expect(UBX::MessageClass resp_cls, uint8_t resp_msg_id);
	void send_with_expect(uint8_t *buffer, unsigned int sz,
	                      UBX::MessageClass resp_cls = UBX::MessageClass::MSG_CLASS_ACK,
	                      uint8_t resp_msg_id = UBX::ACK::ID_ACK);
	void send(uint8_t *buffer, unsigned int sz, bool notify_sent = false, bool use_ext_buffer = false);
	void wait_send() { wait_tx_idle(); }
	void set_baudrate(unsigned int baudrate);
	/// @brief Restart reception after a UART error.
	///
	/// `handle_error()` STOPS the libuarte RX so as not to loop on the error, and
	/// the only place that restarted it was `set_baudrate()`. Any branch that
	/// tolerates the error without changing rate therefore left the UART deaf until
	/// the end of the state: not another byte came back, the MAX_FRAMING_ERRORS_BOOT
	/// threshold became unreachable, and the first framing error during `configure`
	/// killed the session. No-op if the RX is running.
	void restart_rx();
	void start_dbd_filter();
	void stop_dbd_filter();
	bool is_expected_msg_count(uint8_t *buffer, unsigned int length, unsigned int expected, unsigned int &actual_count,
	                           UBX::MessageClass msg_cls, uint8_t msg_id);
	void copy_mga_ano_to_buffer(File &file, uint8_t *dest_buffer, const unsigned int buffer_size, std::time_t now,
	                            unsigned int &num_bytes_copied, unsigned int &num_msg_copied,
	                            unsigned int &ano_start_pos, unsigned int ano_stale_threshold_s = 24 * 3600);
	void set_debug_enable(bool e) { m_debug_enable = e; }

private:
	// Passthrough/bridge state
	volatile bool m_passthrough_active = false;
	PassthroughCallback m_passthrough_callback;
	uint8_t m_pt_isr_buf[1024];  // ISR buffer for passthrough mode
	volatile uint16_t m_pt_isr_buf_len = 0;

	UBXCommsEventNavReport m_nav_report;
	UBXCommsEventSatReport m_sat_report;
	UBXCommsEventRawMeasurement m_raw_meas;
	unsigned int m_instance;
	volatile bool m_is_send_busy;
	bool m_notify_sent;
	unsigned int m_nav_report_iTOW;
	bool m_dbd_filter_en;
	bool m_debug_enable;
	struct Expect {
		bool enable;
		uint8_t req_cls;
		uint8_t req_msg_id;
		uint8_t resp_cls;
		uint8_t resp_msg_id;
	} m_expect;
	uint8_t m_tx_buffer[256];
	uint8_t m_rx_buffer[1024];
	unsigned int m_rx_buffer_offset;
	bool m_is_init;
	/// RX stopped by handle_error() and not restarted yet. Prevents a double
	/// start_rx (which would re-arm the libuarte DMA buffers).
	bool m_rx_stopped = false;

	// Bounded wait for previous TX to complete (max 100ms with 1ms polling)
	static constexpr unsigned int SEND_BUSY_TIMEOUT_MS = 100;
	void wait_tx_idle();

	void handle_error(unsigned int);
	void handle_tx_done();
	void handle_rx_buffer(uint8_t *buffer, unsigned int length);
	UBX::HeaderAndPayloadCRC *parse_message(uint8_t **buffer, unsigned int *length);
	void compute_crc(const uint8_t *const buffer, const unsigned int length, uint8_t &ck_a, uint8_t &ck_b);
	void run_expect_filter(const UBX::HeaderAndPayloadCRC *const msg);
	void run_nav_filter(const UBX::HeaderAndPayloadCRC *const msg);
	void run_rxm_filter(const UBX::HeaderAndPayloadCRC *const msg);
	void run_dbd_filter(uint8_t *buffer, unsigned int length);

	friend AsyncLowLevel;
};
