/**
 * @file m10qasync_config.cpp
 * @brief M10Q driver — UBX configuration helpers, assistance data, device info,
 *        and the u-center passthrough bridge.
 *
 * Split off the tail of m10qasync.cpp, which was 3 741 lines. Everything here is
 * called BY the state machine in that file but contains none of it: these are
 * the VALSET builders, the AssistNow / DBD plumbing, MON-VER and SEC-UNIQID
 * queries, and the raw bridge. The cut is a single contiguous slice, so the two
 * files still add up to the original line for line.
 *
 * The four things it still needs from the state machine -- DEFAULT_BAUDRATE,
 * MAX_BAUDRATE, VAL_GNSS and STATE_EQUAL -- live in m10qasync_internal.hpp.
 */

#include <cstdint>
#include <cstring>
#include <vector>

#include "m10qasync.hpp"
#include "m10qasync_internal.hpp"
#include "ubx.hpp"
#include "ubx_comms.hpp"
#include "bsp.hpp"
#include "gpio.hpp"
#include "debug.hpp"
#include "error.hpp"
#include "pmu.hpp"
#include "binascii.hpp"
#include "scheduler.hpp"
#include "rtc.hpp"
#include "timeutils.hpp"
#include "config_store.hpp"
#include "filesystem.hpp"

extern Scheduler *system_scheduler;
extern RTC *rtc;
extern ConfigurationStore *configuration_store;
extern FileSystem *main_filesystem;

using namespace UBX;

void M10QAsyncReceiver::sync_baud_rate(unsigned int baud) {
	DEBUG_TRACE("M10QAsyncReceiver::sync_baud_rate: Syncing baud rate to %u", baud);

	m_ubx_comms.set_baudrate(baud);

	// Test configuration by sending a known invalid message and expecting a NACK
	CFG::MSG::MSG_MSG_NORATE cfg_msg_invalid = {
		.msgClass = MessageClass::MSG_CLASS_BAD,
		.msgID = 0,
	};

	initiate_timeout(500);
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_MSG, cfg_msg_invalid,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_NACK);
}


void M10QAsyncReceiver::save_config() {
	DEBUG_TRACE("M10QAsyncReceiver::save_config: GPS CFG-CFG ->");
	CFG::CFG::MSG_CFG cfg_msg_cfg_cfg = {
		.clearMask = 0,
		.saveMask = 0xFFFFFFFF,
		// CFG::CFG::CLEARMASK_IOPORT   |
		// 			   CFG::CFG::CLEARMASK_MSGCONF  |
		// 			   CFG::CFG::CLEARMASK_INFMSG   |
		// 			   CFG::CFG::CLEARMASK_NAVCONF  |
		// 			   CFG::CFG::CLEARMASK_RXMCONF  |
		// 			   CFG::CFG::CLEARMASK_SENCONF  |
		// 			   CFG::CFG::CLEARMASK_RINVCONF |
		// 			   CFG::CFG::CLEARMASK_ANTCONF  |
		// 			   CFG::CFG::CLEARMASK_LOGCONF  |
		// 			   CFG::CFG::CLEARMASK_FTSCONF,
		.loadMask = 0,
		//.deviceMask  = CFG::CFG::DEVMASK_SPIFLASH, //BBR changed to SPIFLASH for not realy saving but go to next step.
		.deviceMask = CFG::CFG::DEVMASK_BBR,  //BBR changed to SPIFLASH for not realy saving but go to next step.
	};

	initiate_timeout();
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_CFG, cfg_msg_cfg_cfg);
}

void M10QAsyncReceiver::clear_config() {
	DEBUG_INFO("M10QAsyncReceiver::clear_config: erasing the receiver BBR config (CFG-CFG clear)");
	VAL_GNSS("clear_bbr_config");
	CFG::CFG::MSG_CFG cfg_msg_cfg_cfg = {
		.clearMask = 0xFFFFFFFF,
		.saveMask = 0,
		.loadMask = 0,  // pas de rechargement: la config RAM en cours reste active
		.deviceMask = CFG::CFG::DEVMASK_BBR,
	};
	// AN ACKNOWLEDGEMENT IS EXPECTED. The general rule for the UBX-CFG class (M10
	// SPG 5.10 interface description, §3.10) is "acknowledged by ACK-ACK if
	// processed, ACK-NAK otherwise", and CFG-CFG carries NO exemption clause --
	// unlike CFG-RST, whose spec explicitly says not to expect a reply. Sending
	// without waiting therefore threw away the only proof that the erase happened.
	//
	// Why it matters: CFG-CFG was REMOVED from protocol 34.20 (SPG 5.20), where it
	// is replaced by UBX-CFG-OTP. On a module provisioned with that firmware the
	// erase would fail silently and the BBR escape hatch would no longer exist. The
	// absence of an ACK is the thing to report, not a detail.
	initiate_timeout();
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_CFG, cfg_msg_cfg_cfg);
}

void M10QAsyncReceiver::soft_reset() {
	// navBbrMask selects what survives the GNSS-only software reset:
	//   0x0000 = hot start  (keep ephemeris+almanac+pos+time+clock+aiding)
	//   0xFFFF = cold start (wipe everything in BBR)
	// A cold start is requested via GPSNavSettings.cold_start. It is issued
	// here, BEFORE supply_time_assistance (step 13), supply_position_assistance
	// (step 14) and the offline-database (ANO) send in the same configure
	// sequence — so the receiver is wiped clean and immediately re-seeded with
	// fresh time + position + ANO. RESETMODE_SOFTWARE_RESET_GNSS_ONLY keeps the
	// UART/port config, so no baud re-sync is needed.
	const uint16_t bbr_mask = m_nav_settings.cold_start ? 0xFFFF : 0x0000;
	DEBUG_TRACE("M10QAsyncReceiver::soft_reset: GPS CFG-RST-> navBbrMask=0x%04X (%s)", bbr_mask,
	            m_nav_settings.cold_start ? "COLD" : "hot");
	VAL_GNSS("soft_reset navBbrMask=0x%04X cold=%u", bbr_mask, (unsigned)m_nav_settings.cold_start);
	CFG::RST::MSG_RST cfg_msg_cfg_rst = {
		.navBbrMask = bbr_mask,
		.resetMode = CFG::RST::RESETMODE_SOFTWARE_RESET_GNSS_ONLY,
		//.resetMode = CFG::RST::RESETMODE_HARDWARE_RESET_IMMEDIATE,
		.reserved1 = 0,
	};
	m_ubx_comms.send_packet(MessageClass::MSG_CLASS_CFG, CFG::ID_RST, cfg_msg_cfg_rst);
	m_ubx_comms.wait_send();
	// 2026-08: the cold-start request is consumed HERE, once the CFG-RST has
	// actually been sent. Without that the flag stayed stuck in m_nav_settings and,
	// on the warm-wake path (which did not refresh the settings), every following
	// session erased the BBR again -- the exact opposite of what deep-idle is for.
	// The log above is the field evidence that the wipe did happen.
	if (m_nav_settings.cold_start) {
		DEBUG_INFO("M10QAsyncReceiver: COLD START applique (BBR effacee) — demande consommee");
		m_nav_settings.cold_start = false;
	}
}

void M10QAsyncReceiver::setup_uart_port() {
	DEBUG_TRACE("M10QAsyncReceiver::setup_uart_port: Configuring UART1 with VALSET ->");

	// 8N1 at MAX_BAUDRATE (460800). The receiver keeps this in BBR, which is why
	// state_poweron has to probe every rate in BOOT_BAUD_TABLE rather than
	// assume the 9600 of a factory-fresh part.
	CFG::UART1::BAUDRATE.set_value(MAX_BAUDRATE);
	CFG::UART1::STOPBITS.set_value(CFG::UART1::StopBits::ONE);
	CFG::UART1::DATABITS.set_value(CFG::UART1::DataBits::EIGHT);
	CFG::UART1::PARITY.set_value(CFG::UART1::Parity::NONE);
	CFG::UART1::ENABLED.set_value(1);

	// UBX in and out, NMEA off in both directions: the parser only speaks UBX,
	// and NMEA sentences would just burn UART time and current.
	CFG::UART1::INPROT_UBX.set_value(1);
	CFG::UART1::OUTPROT_UBX.set_value(1);
	CFG::UART1::INPROT_NMEA.set_value(0);
	CFG::UART1::OUTPROT_NMEA.set_value(0);

	// TX-ready line unused: we read the UART continuously, so there is no pin to
	// assert and no threshold to reach. All five values are therefore 0.
	CFG::UART1::TXREADY_ENABLED.set_value(0);
	CFG::UART1::TXREADY_POLARITY.set_value(0);
	CFG::UART1::TXREADY_PIN.set_value(0);
	CFG::UART1::TXREADY_THRESHOLD.set_value(0);
	CFG::UART1::TXREADY_INTERFACE.set_value(0);

	// Collect all parameters in a vector
	std::vector<UBX::CFG::UBXParameter> uart1_config = {
		CFG::UART1::ENABLED,           CFG::UART1::BAUDRATE,         CFG::UART1::STOPBITS,
		CFG::UART1::DATABITS,          CFG::UART1::PARITY,           CFG::UART1::INPROT_UBX,
		CFG::UART1::OUTPROT_UBX,       CFG::UART1::INPROT_NMEA,      CFG::UART1::OUTPROT_NMEA,
		CFG::UART1::TXREADY_ENABLED,   CFG::UART1::TXREADY_POLARITY, CFG::UART1::TXREADY_PIN,
		CFG::UART1::TXREADY_THRESHOLD, CFG::UART1::TXREADY_INTERFACE
	};
	uint8_t layers = CFG::VALSET::LAYERS::BBR | CFG::VALSET::LAYERS::RAM;
	//DEBUG_TRACE("M10QAsyncReceiver::setup_uart_port: save %x", layers);
	//uint8_t layers = CFG::VALSET::LAYERS::RAM;
	// Create the VALSET message with dynamically sized parameters
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t uart1_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &uart1_valset_msg = *new (uart1_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, layers, uart1_config);
	size_t cfgDataSize = uart1_valset_msg.get_cfgData_size(uart1_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, uart1_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
	m_ubx_comms.wait_send();
}

// UBX configuration is written to a DELIBERATE storage layer per setting, and
// the choice is not cosmetic. As written today:
//   BBR|RAM  setup_power_management, setup_continuous_mode,
//            setup_simple_navigation_settings, setup_gnss_channel_sharing
//            -- survives a reset, and a rail cut when V_BCKP holds it.
//   RAM      everything else, including every NAV/RXM message rate and the
//            expert navigation settings -- re-applied on every session, so a
//            stale value can never outlive a firmware change.
// Persisting the wrong thing has already cost us a receiver: a baud rate held
// in BBR that the boot probe did not cover left the M10Q permanently mute (see
// BOOT_BAUD_TABLE at the top of this file). Each call below used to carry a
// commented-out alternative layer beside it; those have been removed. Changing
// a layer is a reviewed decision, not a line to uncomment.
void M10QAsyncReceiver::setup_power_management() {
	DEBUG_TRACE("M10QAsyncReceiver::setup_power_management: Configuring power management with VALSET ->");

	// Set power management parameters
	CFG::PM::OPERATEMODE.set_value(CFG::PM::OPERATEMODE_VALUES::PSMCT);  // Set to cyclic tracking mode
	CFG::PM::POSUPDATEPERIOD.set_value(1000);                            // Set position update period to 1000 seconds
	CFG::PM::ACQPERIOD.set_value(10000);  // Set acquisition period to 10000 seconds if failed
	CFG::PM::GRIDOFFSET.set_value(0);     // No offset for GPS week alignment
	CFG::PM::ONTIME.set_value(1);         // Time in Tracking state (1 second)
	CFG::PM::MINACQTIME.set_value(300);   // Minimum acquisition time (300 ms)
	CFG::PM::MAXACQTIME.set_value(0);     // Maximum acquisition time (no limit)
	CFG::PM::DONOTENTEROFF.set_value(1);  // Do not enter off state if fix fails
	CFG::PM::WAITTIMEFIX.set_value(0);    // No need to wait for time fix
	CFG::PM::UPDATEEPH.set_value(1);      // Regular ephemeris updates

	// Collect all parameters for VALSET
	std::vector<CFG::UBXParameter> pm_config = {
		CFG::PM::OPERATEMODE, CFG::PM::POSUPDATEPERIOD, CFG::PM::ACQPERIOD,  CFG::PM::GRIDOFFSET,
		CFG::PM::ONTIME,      CFG::PM::MINACQTIME,      CFG::PM::MAXACQTIME, CFG::PM::DONOTENTEROFF,
		CFG::PM::WAITTIMEFIX, CFG::PM::UPDATEEPH,
	};

	// Create the VALSET message with power management configuration
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t pm_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &pm_valset_msg = *new (pm_valset_msg_storage) CFG::VALSET::MSG_VALSET(
	    0x00, CFG::VALSET::LAYERS::BBR | CFG::VALSET::LAYERS::RAM, pm_config);
	size_t cfgDataSize = pm_valset_msg.get_cfgData_size(pm_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, pm_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::setup_continuous_mode() {
	DEBUG_TRACE("M10QAsyncReceiver::setup_continuous_mode: Configuring continuous mode with VALSET ->");

	// Set power management to FULL mode for continuous operation
	CFG::PM::OPERATEMODE.set_value(CFG::PM::FULL);  // Disable all power-saving modes

	// Collect parameters in a vector for the VALSET message
	std::vector<CFG::UBXParameter> pm_config = { CFG::PM::OPERATEMODE };

	// Create the VALSET message for power management configuration
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t pm_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &pm_valset_msg = *new (pm_valset_msg_storage) CFG::VALSET::MSG_VALSET(
	    0x00, CFG::VALSET::LAYERS::BBR | CFG::VALSET::LAYERS::RAM, pm_config);
	size_t cfgDataSize = pm_valset_msg.get_cfgData_size(pm_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, pm_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::setup_simple_navigation_settings() {
	DEBUG_TRACE("M10QAsyncReceiver::setup_simple_navigation_settings: Configuring NAVSPG with VALSET ->");

	// Set the navigation parameters for standard precision
	CFG::NAVSPG::FIXMODE.set_value(
	    static_cast<CFG::NAVSPG::FIXMODE_VALUES>(m_nav_settings.fix_mode));  // Set position fix mode dynamically
	CFG::NAVSPG::DYNMODEL.set_value(
	    static_cast<CFG::NAVSPG::DYNMODEL_VALUES>(m_nav_settings.dyn_model));  // Set dynamic platform model dynamically
	CFG::NAVSPG::UTCSTANDARD.set_value(CFG::NAVSPG::UTCSTANDARD_VALUES::UTC_AUTO);  // UTC standard auto
	CFG::NAVSPG::OUTFIL_PDOP.set_value(250);                                        // Position DOP threshold (25.0)
	CFG::NAVSPG::OUTFIL_TDOP.set_value(250);                                        // Time DOP threshold (25.0)
	CFG::NAVSPG::OUTFIL_PACC.set_value(100);                                        // Position accuracy mask
	CFG::NAVSPG::OUTFIL_TACC.set_value(350);                                        // Time accuracy mask
	CFG::NAVSPG::CONSTR_ALT.set_value(0);                                           // Fixed altitude for 2D fix mode
	CFG::NAVSPG::CONSTR_ALTVAR.set_value(10000);                                    // Fixed altitude variance
	CFG::NAVSPG::INFIL_MINELEV.set_value(m_nav_settings.min_elev);                  // Minimum elevation angle [deg]
	CFG::NAVSPG::CONSTR_DGNSSTO.set_value(60);                                      // DGNSS timeout

	// Collect all parameters in a vector for VALSET
	std::vector<CFG::UBXParameter> navspg_config = {
		CFG::NAVSPG::FIXMODE,       CFG::NAVSPG::DYNMODEL,      CFG::NAVSPG::UTCSTANDARD,    CFG::NAVSPG::OUTFIL_PDOP,
		CFG::NAVSPG::OUTFIL_TDOP,   CFG::NAVSPG::OUTFIL_PACC,   CFG::NAVSPG::OUTFIL_TACC,    CFG::NAVSPG::CONSTR_ALT,
		CFG::NAVSPG::CONSTR_ALTVAR, CFG::NAVSPG::INFIL_MINELEV, CFG::NAVSPG::CONSTR_DGNSSTO,
	};

	// Create the VALSET message with the navigation parameters
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_valset_msg = *new (nav_valset_msg_storage) CFG::VALSET::MSG_VALSET(
	    0x00, CFG::VALSET::LAYERS::BBR | CFG::VALSET::LAYERS::RAM, navspg_config);
	size_t cfgDataSize = nav_valset_msg.get_cfgData_size(navspg_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::setup_expert_navigation_settings() {
	DEBUG_TRACE("M10QAsyncReceiver::setup_expert_navigation_settings: Configuring NAVSPG and ANA with VALSET ->");

	// NAVSPG Configuration (Standard Precision Navigation)
	CFG::NAVSPG::INFIL_MINSVS.set_value(3);                       // Minimum satellites for navigation
	CFG::NAVSPG::INFIL_MAXSVS.set_value(32);                      // Maximum satellites for navigation
	CFG::NAVSPG::INFIL_MINCNO.set_value(m_nav_settings.min_cno);  // Minimum satellite signal level [dBHz]
	CFG::NAVSPG::INIFIX3D.set_value(0);                           // Do not require initial 3D fix
	//CFG::NAVSPG::WKNROLLOVER.set_value(0);                          // Default GPS week rollover min value allowed is 1 - default is 2148, do not update
	CFG::NAVSPG::ACKAIDING.set_value(1);  // Acknowledge assistance messages
	CFG::NAVSPG::SIGATTCOMP.set_value(static_cast<uint8_t>(
	    CFG::NAVSPG::SIGATTCOMP_VALUES::SIGCOMP_AUTO));  // Auto signal attenuation compensation (patch antenna)

	// ANA (AssistNow Autonomous) Configuration
	CFG::ANA::USE_ANA.set_value(m_nav_settings.assistnow_autonomous_enable
	                                ? static_cast<uint8_t>(CFG::ANA::USE_ANA_VALUES::ANA_ENABLED)
									: static_cast<uint8_t>(CFG::ANA::USE_ANA_VALUES::ANA_DISABLED));
	CFG::ANA::ORBMAXERR.set_value(m_nav_settings.orbmaxerr);  // Maximum orbit error for AssistNow Autonomous [m]

	// Collect parameters into a vector
	std::vector<CFG::UBXParameter> navspg_ana_config = { CFG::NAVSPG::INFIL_MINSVS, CFG::NAVSPG::INFIL_MAXSVS,
	                                                     CFG::NAVSPG::INFIL_MINCNO, CFG::NAVSPG::INIFIX3D,
	                                                     //CFG::NAVSPG::WKNROLLOVER,
	                                                     CFG::NAVSPG::ACKAIDING, CFG::NAVSPG::SIGATTCOMP,
	                                                     CFG::ANA::USE_ANA, CFG::ANA::ORBMAXERR };

	// Create and send VALSET message
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_ana_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_ana_valset_msg =
	    *new (nav_ana_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, navspg_ana_config);
	size_t cfgDataSize = nav_ana_valset_msg.get_cfgData_size(navspg_ana_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_ana_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::supply_time_assistance() {
	// A receiver that kept its BBR has its own GNSS time base, necessarily better
	// than ours. We teach it nothing, and risk only constraining it wrongly.
	if (m_bbr_retained) {
		DEBUG_INFO("M10QAsyncReceiver::supply_time_assistance: BBR retained — the receiver has its own time, injection "
		           "unnecessary");
		m_step++;
		m_op_state = OpState::IDLE;
		run_state_machine();
		return;
	}

	// The REAL uncertainty on our time. Zero means the source cannot be bounded (a
	// time restored from flash, or a virtual clock): the tAccS field then cannot be
	// filled in honestly, and a wrong time announced as certain is worse than no
	// time at all -- it narrows the receiver's search window around a wrong value.
	const unsigned int tacc = rtc->time_accuracy_s();
	if (tacc == 0) {
		DEBUG_INFO("M10QAsyncReceiver::supply_time_assistance: time source accuracy not boundable (%s) — no injection",
		           rtc->source() == RtcSource::RESTORED ? "restored from flash" : "never synchronised");
		m_step++;
		m_op_state = OpState::IDLE;
		run_state_machine();
		return;
	}

	DEBUG_INFO("M10QAsyncReceiver::supply_time_assistance: MGA-INI-TIME tAccS=%u s (age %u s, drift %d ppm)", tacc,
	           rtc->age_s(), (int)rtc->drift_ppm());
	uint16_t year;
	uint8_t month, day, hour, min, sec;

	convert_datetime_to_epoch(rtc->gettime(), year, month, day, hour, min, sec);

	MGA::MSG_INI_TIME_UTC cfg_msg_ini_time_utc = {
		.type = 0x10,
		.version = 0x00,
		.ref = 0x00,
		.leapSecs = -128,  // Number of leap seconds unknown
		.year = year,
		.month = month,
		.day = day,
		.hour = hour,
		.minute = min,
		.second = sec,
		.reserved1 = 0,
		.ns = 0,
		.tAccS = (uint16_t)tacc,  // incertitude mesuree, plus une constante optimiste
		.reserved2 = { 0 },
		.tAccNs = 0
	};

	initiate_timeout();
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_MGA, MGA::ID_INI_TIME_UTC, cfg_msg_ini_time_utc,
	                                    MessageClass::MSG_CLASS_MGA, MGA::ID_ACK);
}

void M10QAsyncReceiver::supply_position_assistance() {
	const auto &last_gps = configuration_store->get_last_gps_entry();
	if (!last_gps.info.valid) {
		DEBUG_TRACE("M10QAsyncReceiver::supply_position_assistance: no valid last position");
		return;
	}

	// The age of the position can only be computed if the current time is reliable.
	// With a time restored from flash the gap can be weeks, and the age would be a
	// fiction -- better to inject nothing.
	if (rtc->time_accuracy_s() == 0) {
		DEBUG_INFO("M10QAsyncReceiver::supply_position_assistance: time unreliable — position age not computable, "
		           "no injection");
		return;
	}

	// Age de la derniere position connue.
	const std::time_t fix_t = convert_epochtime(last_gps.info.year, last_gps.info.month, last_gps.info.day,
	                                            last_gps.info.hour, last_gps.info.min, last_gps.info.sec);
	const std::time_t now_t = rtc->gettime();
	const unsigned long age_s = (now_t > fix_t) ? (unsigned long)(now_t - fix_t) : 0UL;

	// Reference speed: the one MEASURED at the last fix (gSpeed, mm/s), bounded.
	// The floor covers an animal that was still at the moment of the fix but has
	// moved since; the ceiling stops an outlier from blowing up the radius.
	static constexpr unsigned long POS_SPEED_FLOOR_CM_S = 50;    // 0,5 m/s
	static constexpr unsigned long POS_SPEED_CEIL_CM_S = 500;    // 5 m/s
	static constexpr unsigned long POS_ACC_MAX_CM = 30000000UL;  // 300 km
	unsigned long speed_cm_s = (last_gps.info.gSpeed > 0) ? (unsigned long)last_gps.info.gSpeed / 10UL : 0UL;
	if (speed_cm_s < POS_SPEED_FLOOR_CM_S) speed_cm_s = POS_SPEED_FLOOR_CM_S;
	if (speed_cm_s > POS_SPEED_CEIL_CM_S) speed_cm_s = POS_SPEED_CEIL_CM_S;

	// An honest uncertainty radius: the accuracy of the fix, plus a margin, plus
	// how far the animal could have travelled since. Without that last term we were
	// handing the receiver a position days old with the accuracy it had at the
	// instant of the fix -- a false constraint it uses to narrow its search.
	const unsigned long acc_cm = (unsigned long)last_gps.info.hAcc / 10UL + 100UL + age_s * speed_cm_s;
	if (acc_cm > POS_ACC_MAX_CM) {
		DEBUG_INFO("M10QAsyncReceiver::supply_position_assistance: position too old (age %lu s, radius %lu km) — "
		           "no injection",
		           age_s, acc_cm / 100000UL);
		return;
	}

	DEBUG_INFO(
	    "M10QAsyncReceiver::supply_position_assistance: MGA-INI-POS lat=%f lon=%f radius=%lu m (age %lu s, %lu cm/s)",
	    last_gps.info.lat, last_gps.info.lon, acc_cm / 100UL, age_s, speed_cm_s);

	MGA::MSG_INI_POS_LLH msg = {
		.type = 0x01,
		.version = 0x00,
		.reserved1 = { 0 },
		.lat = static_cast<int32_t>(last_gps.info.lat * 1e7),
		.lon = static_cast<int32_t>(last_gps.info.lon * 1e7),
		.alt = last_gps.info.height / 10,  // mm -> cm
		.posAcc = (uint32_t)acc_cm,
	};

	initiate_timeout();
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_MGA, MGA::ID_INI_TIME_UTC, msg,
	                                    MessageClass::MSG_CLASS_MGA, MGA::ID_ACK);
}

void M10QAsyncReceiver::disable_odometer() {
	DEBUG_TRACE("M10QAsyncReceiver::disable_odometer: Disabling odometer with VALSET ->");


	// Set the odometer parameters to the desired values for disabling
	CFG::ODO::USE_ODO.set_value(0x00);                    // Disable odometer usage
	CFG::ODO::USE_COG.set_value(0x00);                    // Disable low-speed course over ground filter
	CFG::ODO::OUTLPVEL.set_value(0x00);                   // Disable low-pass filtered velocity output
	CFG::ODO::OUTLPCOG.set_value(0x00);                   // Disable low-pass filtered course over ground output
	CFG::ODO::PROFILE.set_value(CFG::ODO::Profile::RUN);  // Set profile to CUSTOM (disabled state)
	CFG::ODO::COG_MAXSPEED.set_value(1);    // Set minimal max speed for course over ground filter (example value)
	CFG::ODO::COG_MAXPOSACC.set_value(50);  // Set example value for max position accuracy
	CFG::ODO::VEL_LPGAIN.set_value(153);    // Set velocity low-pass filter gain
	CFG::ODO::COG_LPGAIN.set_value(76);     // Set course over ground low-pass filter gain

	// Collect all parameters in a vector for MSG_VALSET
	std::vector<CFG::UBXParameter> odo_config = { CFG::ODO::USE_ODO,       CFG::ODO::USE_COG,    CFG::ODO::OUTLPVEL,
	                                              CFG::ODO::OUTLPCOG,      CFG::ODO::PROFILE,    CFG::ODO::COG_MAXSPEED,
	                                              CFG::ODO::COG_MAXPOSACC, CFG::ODO::VEL_LPGAIN, CFG::ODO::COG_LPGAIN };

	// Create the VALSET message with the odometer parameters
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t odo_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &odo_valset_msg =
	    *new (odo_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, odo_config);
	size_t cfgDataSize = odo_valset_msg.get_cfgData_size(odo_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, odo_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::disable_timepulse_output() {
	DEBUG_TRACE("M10QAsyncReceiver::disable_timepulse_output: Configuring timepulse output with VALSET ->");
	// Minimal configuration for disabling time pulse with 1 Hz setting
	CFG::TP::PULSE_DEF.set_value(CFG::TP::PULSE_DEF::PERIOD);  // Set pulse mode to period
	CFG::TP::PERIOD_TP1.set_value(1000000);                    // Set period to 1 second (1 Hz) in microseconds
	CFG::TP::PERIOD_LOCK_TP1.set_value(1000000);               // Set locked period to 1 second in microseconds
	CFG::TP::LEN_TP1.set_value(0);                             // Set pulse length to 0 (no pulse)
	CFG::TP::TP1_ENA.set_value(0);                             // Disable the time pulse
	CFG::TP::POL_TP1.set_value(0);                             // Set time pulse polarity to default (falling edge)

	// Collect only necessary parameters in the configuration vector
	std::vector<CFG::UBXParameter> tp_config = { CFG::TP::PULSE_DEF, CFG::TP::PERIOD_TP1, CFG::TP::PERIOD_LOCK_TP1,
	                                             CFG::TP::LEN_TP1,   CFG::TP::POL_TP1,    CFG::TP::TP1_ENA };

	// Create the VALSET message with the timepulse parameters
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t tp_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &tp_valset_msg =
	    *new (tp_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, tp_config);
	size_t cfgDataSize = tp_valset_msg.get_cfgData_size(tp_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, tp_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::enable_nav_pvt_message() {
	DEBUG_TRACE("M10QAsyncReceiver::enable_nav_pvt_message: Configuring NAV PVT output on UART1 with VALSET ->");

	// Set output rate for UBX-NAV-PVT message on UART1 port
	CFG::MSGOUT::NAV_PVT_UART1.set_value(1);  // Set rate to 1 (enables message output on UART1)

	// Collect the NAV PVT UART1 configuration parameter in a vector
	std::vector<CFG::UBXParameter> nav_pvt_config = { CFG::MSGOUT::NAV_PVT_UART1 };

	// Create and send the VALSET message with the NAV PVT configuration
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_pvt_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_pvt_valset_msg =
	    *new (nav_pvt_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, nav_pvt_config);
	size_t cfgDataSize = nav_pvt_valset_msg.get_cfgData_size(nav_pvt_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_pvt_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::disable_nav_pvt_message() {
	DEBUG_TRACE("M10QAsyncReceiver::disable_nav_pvt_message: Disabling NAV-PVT message with VALSET ->");

	CFG::MSGOUT::NAV_PVT_UART1.set_value(0);  // Disable NAV-PVT message on UART1

	std::vector<CFG::UBXParameter> nav_pvt_config = { CFG::MSGOUT::NAV_PVT_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_pvt_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_pvt_valset_msg =
	    *new (nav_pvt_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, nav_pvt_config);

	size_t cfgDataSize = nav_pvt_valset_msg.get_cfgData_size(nav_pvt_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_pvt_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::enable_nav_dop_message() {
	DEBUG_TRACE("M10QAsyncReceiver::enable_nav_dop_message: Enabling NAV-DOP message with VALSET ->");

	CFG::MSGOUT::NAV_DOP_UART1.set_value(1);  // Enable NAV-DOP message on UART1

	std::vector<CFG::UBXParameter> nav_dop_config = { CFG::MSGOUT::NAV_DOP_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_dop_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_dop_valset_msg =
	    *new (nav_dop_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, nav_dop_config);

	size_t cfgDataSize = nav_dop_valset_msg.get_cfgData_size(nav_dop_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_dop_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::disable_nav_dop_message() {
	DEBUG_TRACE("M10QAsyncReceiver::disable_nav_dop_message: Disabling NAV-DOP message with VALSET ->");

	CFG::MSGOUT::NAV_DOP_UART1.set_value(0);  // Disable NAV-DOP message on UART1

	std::vector<CFG::UBXParameter> nav_dop_config = { CFG::MSGOUT::NAV_DOP_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_dop_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_dop_valset_msg =
	    *new (nav_dop_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, nav_dop_config);

	size_t cfgDataSize = nav_dop_valset_msg.get_cfgData_size(nav_dop_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_dop_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::enable_nav_status_message() {
	DEBUG_TRACE("M10QAsyncReceiver::enable_nav_status_message: Enabling NAV-STATUS message with VALSET ->");

	CFG::MSGOUT::NAV_STATUS_UART1.set_value(1);  // Enable NAV-STATUS message on UART1

	std::vector<CFG::UBXParameter> nav_status_config = { CFG::MSGOUT::NAV_STATUS_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_status_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_status_valset_msg =
	    *new (nav_status_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, nav_status_config);

	size_t cfgDataSize = nav_status_valset_msg.get_cfgData_size(nav_status_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_status_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::disable_nav_status_message() {
	DEBUG_TRACE("M10QAsyncReceiver::disable_nav_status_message: Disabling NAV-STATUS message with VALSET ->");

	CFG::MSGOUT::NAV_STATUS_UART1.set_value(0);  // Disable NAV-STATUS message on UART1

	std::vector<CFG::UBXParameter> nav_status_config = { CFG::MSGOUT::NAV_STATUS_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_status_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_status_valset_msg =
	    *new (nav_status_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, nav_status_config);

	size_t cfgDataSize = nav_status_valset_msg.get_cfgData_size(nav_status_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_status_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::enable_nav_sat_message() {
	DEBUG_TRACE("M10QAsyncReceiver::enable_nav_sat_message: Enabling NAV-SAT message with VALSET ->");

	CFG::MSGOUT::NAV_SAT_UART1.set_value(1);  // Enable NAV-SAT message on UART1

	std::vector<CFG::UBXParameter> nav_sat_config = { CFG::MSGOUT::NAV_SAT_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_sat_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_sat_valset_msg =
	    *new (nav_sat_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, nav_sat_config);

	size_t cfgDataSize = nav_sat_valset_msg.get_cfgData_size(nav_sat_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_sat_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::disable_nav_sat_message() {
	DEBUG_TRACE("M10QAsyncReceiver::disable_nav_sat_message: Disabling NAV-SAT message with VALSET ->");

	CFG::MSGOUT::NAV_SAT_UART1.set_value(0);  // Disable NAV-SAT message on UART1

	std::vector<CFG::UBXParameter> nav_sat_config = { CFG::MSGOUT::NAV_SAT_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t nav_sat_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &nav_sat_valset_msg =
	    *new (nav_sat_valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, nav_sat_config);

	size_t cfgDataSize = nav_sat_valset_msg.get_cfgData_size(nav_sat_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, nav_sat_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::enable_rxm_measc12_message() {
	DEBUG_TRACE("M10QAsyncReceiver::enable_rxm_measc12_message");
	CFG::MSGOUT::RXM_MEASC12_UART1.set_value(1);
	std::vector<CFG::UBXParameter> config = { CFG::MSGOUT::RXM_MEASC12_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &valset_msg = *new (valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, config);
	size_t cfgDataSize = valset_msg.get_cfgData_size(config);
	initiate_timeout();
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::enable_rxm_meas20_message() {
	DEBUG_TRACE("M10QAsyncReceiver::enable_rxm_meas20_message");
	CFG::MSGOUT::RXM_MEAS20_UART1.set_value(1);
	std::vector<CFG::UBXParameter> config = { CFG::MSGOUT::RXM_MEAS20_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &valset_msg = *new (valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, config);
	size_t cfgDataSize = valset_msg.get_cfgData_size(config);
	initiate_timeout();
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::enable_rxm_meas50_message() {
	DEBUG_TRACE("M10QAsyncReceiver::enable_rxm_meas50_message");
	CFG::MSGOUT::RXM_MEAS50_UART1.set_value(1);
	std::vector<CFG::UBXParameter> config = { CFG::MSGOUT::RXM_MEAS50_UART1 };
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &valset_msg = *new (valset_msg_storage) CFG::VALSET::MSG_VALSET(0x00, CFG::VALSET::LAYERS::RAM, config);
	size_t cfgDataSize = valset_msg.get_cfgData_size(config);
	initiate_timeout();
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);
}

void M10QAsyncReceiver::setup_gnss_channel_sharing() {
	DEBUG_TRACE("M10QAsyncReceiver::setup_gnss_channel_sharing: Configuring GNSS signals with VALSET (mask=0x%02X) ->",
	            m_nav_settings.constellation_mask);

	// Decode constellation bitmask: bit0=GPS, bit1=GAL, bit2=GLO, bit3=BDS, bit4=QZSS, bit5=SBAS
	uint8_t gps_en = (m_nav_settings.constellation_mask & 0x01) ? 1 : 0;
	uint8_t gal_en = (m_nav_settings.constellation_mask & 0x02) ? 1 : 0;
	uint8_t glo_en = (m_nav_settings.constellation_mask & 0x04) ? 1 : 0;
	uint8_t bds_en = (m_nav_settings.constellation_mask & 0x08) ? 1 : 0;
	uint8_t qzss_en = (m_nav_settings.constellation_mask & 0x10) ? 1 : 0;
	uint8_t sbas_en = (m_nav_settings.constellation_mask & 0x20) ? 1 : 0;

	CFG::SIGNAL::GPS_ENA.set_value(gps_en);
	CFG::SIGNAL::GPS_L1CA_ENA.set_value(gps_en);
	CFG::SIGNAL::GAL_ENA.set_value(gal_en);
	CFG::SIGNAL::GAL_E1_ENA.set_value(gal_en);
	CFG::SIGNAL::GLO_ENA.set_value(glo_en);
	CFG::SIGNAL::GLO_L1_ENA.set_value(glo_en);
	CFG::SIGNAL::BDS_ENA.set_value(bds_en);
	CFG::SIGNAL::BDS_B1C_ENA.set_value(bds_en);  // M10Q uses B1C (not B1I)
	CFG::SIGNAL::BDS_B1_ENA.set_value(0);        // B1I disabled on M10Q
	CFG::SIGNAL::QZSS_ENA.set_value(qzss_en);
	CFG::SIGNAL::QZSS_L1CA_ENA.set_value(qzss_en);
	CFG::SIGNAL::QZSS_L1S_ENA.set_value(0);
	CFG::SIGNAL::SBAS_ENA.set_value(sbas_en);
	CFG::SIGNAL::SBAS_L1CA_ENA.set_value(sbas_en);

	// Collect all parameters in a vector for MSG_VALSET
	std::vector<UBX::CFG::UBXParameter> gnss_signal_config = {
		CFG::SIGNAL::GPS_ENA,     CFG::SIGNAL::GPS_L1CA_ENA, CFG::SIGNAL::SBAS_ENA,      CFG::SIGNAL::SBAS_L1CA_ENA,
		CFG::SIGNAL::GAL_ENA,     CFG::SIGNAL::GAL_E1_ENA,   CFG::SIGNAL::BDS_ENA,       CFG::SIGNAL::BDS_B1_ENA,
		CFG::SIGNAL::BDS_B1C_ENA, CFG::SIGNAL::QZSS_ENA,     CFG::SIGNAL::QZSS_L1CA_ENA, CFG::SIGNAL::QZSS_L1S_ENA,
		CFG::SIGNAL::GLO_ENA,     CFG::SIGNAL::GLO_L1_ENA
	};

	// Create the VALSET message for GNSS signal configuration
	alignas(CFG::VALSET::MSG_VALSET)
	    uint8_t gnss_valset_msg_storage[sizeof(CFG::VALSET::MSG_VALSET) + CFG::VALSET::MSG_VALSET_MAX_CFG];
	auto &gnss_valset_msg = *new (gnss_valset_msg_storage) CFG::VALSET::MSG_VALSET(
	    0x00, CFG::VALSET::LAYERS::BBR | CFG::VALSET::LAYERS::RAM, gnss_signal_config);
	size_t cfgDataSize = gnss_valset_msg.get_cfgData_size(gnss_signal_config);
	initiate_timeout();

	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_CFG, CFG::ID_VALSET, gnss_valset_msg,
	                                    MessageClass::MSG_CLASS_ACK, ACK::ID_ACK, cfgDataSize);

	m_ubx_comms.wait_send();
}

void M10QAsyncReceiver::fetch_navigation_database() {
	UBX::Empty msg_dbd = {};
	initiate_timeout();
	m_ubx_comms.send_packet(MessageClass::MSG_CLASS_MGA, MGA::ID_DBD, msg_dbd);
}

void M10QAsyncReceiver::dump_navigation_database(unsigned int len) {
	for (unsigned int i = 0; i < len; i++)
		printf("%02X", m_navigation_database[i]);
	printf("\n");
}

void M10QAsyncReceiver::query_mon_ver() {
	DEBUG_TRACE("M10QAsyncReceiver::query_mon_ver");
	UBX::Empty msg = {};
	initiate_timeout();
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_MON, MON::ID_VER, msg, MessageClass::MSG_CLASS_MON,
	                                    MON::ID_VER);
}

void M10QAsyncReceiver::query_sec_uniqid() {
	DEBUG_TRACE("M10QAsyncReceiver::query_sec_uniqid");
	UBX::Empty msg = {};
	initiate_timeout();
	m_ubx_comms.send_packet_with_expect(MessageClass::MSG_CLASS_SEC, SEC::ID_UNIQID, msg, MessageClass::MSG_CLASS_SEC,
	                                    SEC::ID_UNIQID);
}

void M10QAsyncReceiver::react(const UBXCommsEventMonVer &ver) {
	if (m_op_state == OpState::PENDING) {
		cancel_timeout();
		std::memcpy(m_gnss_sw_version, ver.swVersion, sizeof(m_gnss_sw_version));
		std::memcpy(m_gnss_hw_version, ver.hwVersion, sizeof(m_gnss_hw_version));
		m_gnss_sw_version[sizeof(m_gnss_sw_version) - 1] = '\0';
		m_gnss_hw_version[sizeof(m_gnss_hw_version) - 1] = '\0';
		DEBUG_INFO("GNSS SW: %s HW: %s", m_gnss_sw_version, m_gnss_hw_version);
		m_op_state = OpState::SUCCESS;
		run_state_machine();
	}
}

void M10QAsyncReceiver::react(const UBXCommsEventSecUniqId &uid) {
	if (m_op_state == OpState::PENDING) {
		cancel_timeout();
		std::memcpy(m_gnss_unique_id, uid.uniqueId, sizeof(m_gnss_unique_id));
		m_gnss_info_valid = true;
		DEBUG_INFO("GNSS UID: %02X%02X%02X%02X%02X", m_gnss_unique_id[0], m_gnss_unique_id[1], m_gnss_unique_id[2],
		           m_gnss_unique_id[3], m_gnss_unique_id[4]);
		notify(GPSEventDeviceInfoReady{});
		m_op_state = OpState::SUCCESS;
		run_state_machine();
	}
}

GNSSDeviceInfo M10QAsyncReceiver::get_device_info() const {
	GNSSDeviceInfo info = {};
	if (m_gnss_info_valid) {
		std::memcpy(info.swVersion, m_gnss_sw_version, sizeof(info.swVersion));
		std::memcpy(info.hwVersion, m_gnss_hw_version, sizeof(info.hwVersion));
		std::memcpy(info.uniqueId, m_gnss_unique_id, sizeof(info.uniqueId));
		info.valid = true;
	}
	return info;
}

GNSSAlmanacStatus M10QAsyncReceiver::get_almanac_status(unsigned int ano_stale_threshold_s) const {
	GNSSAlmanacStatus status = {};

	if (!main_filesystem) {
		status.stale = true;
		return status;
	}

	try {
		LFSFile file(main_filesystem, "gps_config.dat", LFS_O_RDONLY);
		status.file_present = true;
		status.file_size = (unsigned int)file.size();

		// Parse UBX messages to count ANO records and check dates.
		// Only the 6-byte header and the first 8 bytes of an ANO payload
		// (up through the date fields) are inspected, so we keep two tiny
		// stack buffers and seek past the rest. The previous 256-byte stack
		// buffer (MAX_PACKET_LEN) was a stack-pressure risk on large almanac
		// files parsed synchronously from the DTE handler context.
		uint8_t header_buf[sizeof(Header)];
		uint8_t ano_head_buf[8];  // type, version, svId, gnssId, year, month, day, reserved1
		unsigned int total_records = 0;
		unsigned int valid_records = 0;
		std::time_t deltatime = (std::time_t)0xFFFFFFFF;
		// Mitigation M2 (2026-05): treat a virtual RTC (cold-first-boot
		// initialised to 1 by main.cpp's fallback) as "not available" for ANO
		// staleness. Before this guard the previous abs() math happened to be
		// safe (deltatime ~50 years → always > threshold → marked stale), but
		// that was accidental. Anything below year 2000 is virtual or unset.
		constexpr std::time_t RTC_MIN_REAL_FOR_ANO = 946684800;  // 2000-01-01
		bool rtc_available = rtc && rtc->is_set() && rtc->gettime() >= RTC_MIN_REAL_FOR_ANO;
		std::time_t now = rtc_available ? rtc->gettime() : 0;

		// Hard cap on iterations: defense in depth against a corrupted LFS file
		// that could otherwise loop forever if every read returns sizeof(Header)
		// without advancing. Real-world ANO files have a few hundred records;
		// 20000 is well above any plausible legitimate count.
		constexpr unsigned int MAX_ITERATIONS = 20000;
		unsigned int iter = 0;

		while (iter++ < MAX_ITERATIONS) {
			// Periodic watchdog kick — parsing a multi-KB almanac file does
			// hundreds of synchronous LFS reads from the DTE handler context.
			if ((iter & 0x3F) == 0)  // every 64 iterations
				PMU::kick_watchdog();

			lfs_ssize_t sz = file.read(header_buf, sizeof(Header));
			if (sz != (lfs_ssize_t)(sizeof(Header))) break;

			Header *hdr = (Header *)header_buf;
			unsigned int payload_len = hdr->msgLength;
			// Payload + 2-byte CRC must fit a reasonable UBX packet size;
			// reject garbage that would seek us way past EOF or wrap LFS.
			if (payload_len + 2 > MAX_PACKET_LEN) break;

			bool is_ano = (hdr->msgClass == MessageClass::MSG_CLASS_MGA && hdr->msgId == MGA::ID_ANO);

			if (is_ano && rtc_available && payload_len >= sizeof(ano_head_buf)) {
				sz = file.read(ano_head_buf, sizeof(ano_head_buf));
				if (sz != (lfs_ssize_t)sizeof(ano_head_buf)) break;
				// Skip the rest of the payload (data[64] + reserved2[4]) plus the 2-byte CRC.
				lfs_soff_t off = file.seek((lfs_soff_t)(payload_len - sizeof(ano_head_buf) + 2), LFS_SEEK_CUR);
				if (off < 0) break;

				total_records++;

				// ano_head_buf layout: [0]=type [1]=version [2]=svId [3]=gnssId
				//                      [4]=year [5]=month  [6]=day [7]=reserved1
				std::time_t ano_time =
				    convert_epochtime(2000 + ano_head_buf[4], ano_head_buf[5], ano_head_buf[6], 12, 0, 0);
				std::time_t timediff = std::abs(ano_time - now);

				if (timediff < deltatime) {
					deltatime = timediff;
					valid_records = 1;
				} else if (timediff == deltatime) {
					valid_records++;
				}
			} else {
				// Non-ANO message, or ANO without RTC: skip payload + CRC entirely.
				lfs_soff_t off = file.seek((lfs_soff_t)(payload_len + 2), LFS_SEEK_CUR);
				if (off < 0) break;
				if (is_ano) total_records++;
			}
		}

		status.total_records = total_records;
		status.valid_records = valid_records;

		if (!rtc_available || (ano_stale_threshold_s > 0 && deltatime >= ano_stale_threshold_s)) {
			status.stale = true;
			status.valid_records = 0;
		}

	} catch (...) {
		status.file_present = false;
		status.stale = true;
	}

	return status;
}

bool M10QAsyncReceiver::start_bridge(PassthroughCallback rx_callback) {
	if (m_bridge_active) return true;

	// In deep-idle the UART is deinitialised and its PPI channels are released: the
	// rest of this function only calls exit_shutdown() from `idle`, then touches the
	// UART -> released PPI channel -> APP_ERROR_CHECK_BOOL(false) -> SoC reset.
	if (STATE_EQUAL(backupidle) || STATE_EQUAL(enterbackup)) {
		DEBUG_WARN("M10QAsyncReceiver::start_bridge: refuse — GNSS en deep-idle, "
		           "sortir d'abord avec $GNSSBCKP#001;0");
		return false;
	}

	// Cancel any pending state machine task
	cancel_timeout();
	system_scheduler->cancel_task(m_state_machine_handle);

	// Power on GNSS module and init UART (if not already on)
	const bool was_idle = (m_state == State::idle);
	if (was_idle) {
		exit_shutdown();
	}

	// Move to the rate the M10Q ACTUALLY answers on -- but which one that is depends
	// on what we have just done to the receiver:
	//   - coming from `idle`: exit_shutdown() above cut and re-applied the rail, so
	//     the M10Q has just done a POR and comes back at its BOOT rate (9600 if the
	//     BBR is lost, MAX if it was retained) -- it is the probe index that knows
	//     this, not m_synced_baud;
	//   - otherwise the receiver is already powered and configured: it talks at
	//     m_synced_baud.
	// Observed on the bench: forcing m_synced_baud (460800, inherited from the last
	// session) onto a freshly restarted receiver at 9600 left the bridge mute --
	// the very fault this fix was meant to remove, but in the other direction.
	const unsigned int bridge_baud = was_idle ? boot_baud_for_step(0) : m_synced_baud;
	m_ubx_comms.set_baudrate(bridge_baud);

	// Enable passthrough: raw UART RX goes to callback
	m_ubx_comms.set_passthrough(true, rx_callback);
	m_bridge_active = true;

	DEBUG_INFO("M10QAsyncReceiver: bridge mode ACTIVE (%u baud, %s)", bridge_baud,
	           was_idle ? "receiver restarted" : "receiver already configured");
	return true;
}

void M10QAsyncReceiver::stop_bridge() {
	if (!m_bridge_active) return;

	m_ubx_comms.set_passthrough(false);
	m_bridge_active = false;

	DEBUG_INFO("M10QAsyncReceiver: bridge mode STOPPED");

	// Power off the GNSS and return to idle
	enter_shutdown();
	// Restore the SAME baseline as power_off_immediate(): the bridge has pre-empted
	// every client (it cancels the tasks and cuts the rail without going through the
	// FSM). Without this, m_num_power_on stayed at 1 after a PWRON GNSS followed by
	// a bridge, and NO subsequent session could cut the rail any more: ~25-30 mA
	// continuously until the next reset.
	m_state = State::idle;
	m_num_power_on = 0;
	m_powering_off = false;
	m_op_state = OpState::IDLE;
	m_step = 0;
	m_unrecoverable_error = false;
	m_pmreq_baud = DEFAULT_BAUDRATE;  // le rail vient d'etre coupe
	m_bbr_retained = false;
	// This list must stay aligned with power_off_immediate(): these five were
	// missing, and a deep-idle flag or a wake counter inherited from the bridge
	// would have applied to the first session after it.
	m_deep_idle_pending = false;
	m_enterbackup_warm = false;
	m_consecutive_wake_failures = 0;
	m_pmreq_verify_retries = 0;
	m_backupidle_entered_ms = 0;
}

bool M10QAsyncReceiver::bridge_send(const uint8_t *data, size_t len) {
	if (!m_bridge_active) return false;
	return m_ubx_comms.send_raw(data, len);
}

void M10QAsyncReceiver::bridge_process_rx() {
	if (m_bridge_active) m_ubx_comms.process_passthrough_rx();
}
