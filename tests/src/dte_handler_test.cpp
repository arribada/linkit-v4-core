#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

#include "sys_log.hpp"
#include "dte_handler.hpp"

#include "gps_service.hpp"
#include "config_store_fs.hpp"
#include "fake_memory_access.hpp"
#include "fake_battery_mon.hpp"

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "mock_sensor.hpp"
#include "mock_logger.hpp"
#include "previpass.h"
#include "binascii.hpp"


#define BLOCK_COUNT   (256)
#define BLOCK_SIZE    (64 * 1024)
#define PAGE_SIZE     (256)
#define MAX_FILE_SIZE (4 * 1024 * 1024)


extern FileSystem *main_filesystem;
extern ConfigurationStore *configuration_store;
extern MemoryAccess *memory_access;
extern BatteryMonitor *battery_monitor;
extern GPSDevice *gps_device;


TEST_GROUP(DTEHandler) {
	RamFlash *ram_flash;
	LFSFileSystem *ram_filesystem;
	LFSConfigurationStore *store;
	FakeMemoryAccess *fake_memory_access;
	MockLog *mock_system_log;
	MockLog *mock_sensor_log;
	DTEHandler *dte_handler;
	FakeBatteryMonitor *fake_battery_monitor;
	GPSLogFormatter gps_log_formatter;
	SysLogFormatter sys_log_formatter;

	void setup() {
		ram_flash = new RamFlash(BLOCK_COUNT, BLOCK_SIZE, PAGE_SIZE);
		ram_filesystem = new LFSFileSystem(ram_flash);
		ram_filesystem->format();
		ram_filesystem->mount();
		main_filesystem = ram_filesystem;
		store = new LFSConfigurationStore(*ram_filesystem);
		store->init();
		configuration_store = store;
		fake_memory_access = new FakeMemoryAccess();
		memory_access = fake_memory_access;
		mock_system_log = new MockLog("system.log");
		mock_system_log->set_log_formatter(&sys_log_formatter);
		mock_sensor_log = new MockLog("sensor.log");
		mock_sensor_log->set_log_formatter(&gps_log_formatter);
		dte_handler = new DTEHandler();
		fake_battery_monitor = new FakeBatteryMonitor();
		battery_monitor = fake_battery_monitor;
		fake_battery_monitor->set_values(0, 0, false, false);
	}

	void teardown() {
		mock().checkExpectations();
		delete dte_handler;
		delete mock_sensor_log;
		delete mock_system_log;
		delete fake_memory_access;
		delete fake_battery_monitor;
		delete store;
		ram_filesystem->umount();
		delete ram_filesystem;
		main_filesystem = nullptr;
		SensorManager::clear();
		CalibratableManager::clear();
		mock().clear();
	}

	std::string read_file_into_string(std::string path) {
		std::ifstream input_file(path);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnull-dereference"
		return std::string((std::istreambuf_iterator<char>(input_file)), std::istreambuf_iterator<char>());
#pragma GCC diagnostic pop
	}

	std::string read_paspw_file(std::string paspw_file) {
		std::string paspw_data = read_file_into_string(paspw_file);
		std::string buffer;

		// Each packet is of the form "0000..........." so we iteratively
		// find each packet and concatenate it into a working buffer
		size_t pos = 0;
		while (1) {
			// Find start of packet
			pos = paspw_data.find("\"0000", pos);
			if (pos == std::string::npos) break;
			// Find end of packet
			pos++;
			size_t end = paspw_data.find("\"", pos);
			if (end == std::string::npos) break;
			std::string s = paspw_data.substr(pos, end - pos);
			if (s.size() > 30)  // Filter out any unsupported packet types
				buffer += s;
		}

		// Unhexlify and decode the packet
		return buffer;
	}
};


TEST(DTEHandler, PARML_REQ) {
	std::string resp;
	std::string req = DTEEncoder::encode(DTECommand::PARML_REQ);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));

	// Build expected PARML response dynamically from param_map
	std::string payload;
	for (unsigned int i = 0; i < param_map_size; i++) {
		if (param_map[i].is_implemented) {
			if (!payload.empty()) payload += ",";
			payload += param_map[i].key;
		}
	}
	char header[16];
	snprintf(header, sizeof(header), "%03X", (unsigned int)payload.size());
	std::string expected = std::string("$O;PARML#") + header + ";" + payload + "\r";
	STRCMP_EQUAL(expected.c_str(), resp.c_str());
}

TEST(DTEHandler, PARMW_REQ) {
	std::string resp;
	std::string req = "$PARMW#008;ARP05=90\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMW#000;\r", resp.c_str());
	CHECK_EQUAL(90U, configuration_store->read_param<unsigned int>(ParamID::TR_NOM));
}

TEST(DTEHandler, PARMR_REQ) {
	std::string resp;
	std::string req =
	    "$PARMR#0BF;IDT06,IDP12,IDT02,IDT03,ART01,ART02,POT03,IDP11,ART03,ARP05,ARP01,ARP19,ARP18,GNP01,ARP11,ARP16,"
	    "GNP02,GNP03,GNP05,UNP01,UNP02,UNP03,LBP01,LBP02,ARP06,LBP04,LBP05,LBP06,ARP12,LBP07,LBP08,LBP09\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL(
	    "$O;PARMR#13D;IDT06=0,IDP12=0,IDT02=LinkIt V4,IDT03=V0.1,ART01=01/01/1970 "
	    "00:00:00,ART02=0,POT03=0,IDP11=FACTORY,ART03=07/10/2021 "
	    "22:41:14,ARP05=60,ARP01=2,ARP19=0,ARP18=0,GNP01=1,ARP11=1,ARP16=10,GNP02=1,GNP03=2,GNP05=120,UNP01=0,UNP02=0,"
	    "UNP03=1,LBP01=0,LBP02=10,ARP06=240,LBP04=2,LBP05=0,LBP06=1,ARP12=4,LBP07=2,LBP08=1,LBP09=120\r",
	    resp.c_str());
}

TEST(DTEHandler, STATR_REQ) {
	std::string resp;
	std::string req =
	    "$STATR#0BF;IDT06,IDP12,IDT02,IDT03,ART01,ART02,POT03,IDP11,ART03,ARP05,ARP01,ARP19,ARP18,GNP01,ARP11,ARP16,"
	    "GNP02,GNP03,GNP05,UNP01,UNP02,UNP03,LBP01,LBP02,ARP06,LBP04,LBP05,LBP06,ARP12,LBP07,LBP08,LBP09\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL(
	    "$O;STATR#13D;IDT06=0,IDP12=0,IDT02=LinkIt V4,IDT03=V0.1,ART01=01/01/1970 "
	    "00:00:00,ART02=0,POT03=0,IDP11=FACTORY,ART03=07/10/2021 "
	    "22:41:14,ARP05=60,ARP01=2,ARP19=0,ARP18=0,GNP01=1,ARP11=1,ARP16=10,GNP02=1,GNP03=2,GNP05=120,UNP01=0,UNP02=0,"
	    "UNP03=1,LBP01=0,LBP02=10,ARP06=240,LBP04=2,LBP05=0,LBP06=1,ARP12=4,LBP07=2,LBP08=1,LBP09=120\r",
	    resp.c_str());
}

TEST(DTEHandler, STATR_REQ_CheckEmptyRequest) {
	std::string resp;
	std::string req = "$STATR#000;\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL(
	    "$O;STATR#0CE;IDT06=0,IDT02=LinkIt V4,IDT03=V0.1,ART01=01/01/1970 00:00:00,ART02=0,POT03=0,ART03=07/10/2021 "
	    "22:41:14,ART10=0,ART11=0,IDT04=simulator,POT06=0,IDT10=305419896,SYT01=0,PPT01=0,PPT02=0,PPT03=0,PPT04=0,"
	    "MRT01=0\r",
	    resp.c_str());
}

TEST(DTEHandler, PARMR_REQ_CheckEmptyRequest) {
	std::string resp;
	std::string req = "$PARMR#000;\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	// Refreshed 2026-05 after addition of GNP47/GNP48/GNP49 (GNSS extras),
	// SMP00/SMP01 (SMD params), GNP50 (REUSE_LAST max fix age, Plan 1 step 1),
	// RLP01/02/03 (rolling-window rate limiter, Plan 1 step 2) and UNP08
	// default change 1→1000 (RC charge adaptive default).
	// Checksum updated from #6EF → #71C → #728 → #744 → #780 → #788 → #780 → #792 → #77E
	// (ARP36/ARP37 no-fix policy REMOVED 2026-07 — behavior hardwired; -20 bytes).
	// #797: +ARP44/ARP45/ARP46 (blind MAC profile) appended at the end.
	// Latest change: removed LDP02=3 entry (EXT_LED_MODE deprecated — slot 117
	// now reserved, hidden from DTE; external LED was an Icoteq Horizon /
	// Artic-R2 era feature not wired on LinkIt V4 / RSPB). Coincidence that
	// the size lands back on the same #780 as the pre-GNP51 baseline: the 8
	// bytes lost to ",LDP02=3" match the 8 bytes gained by ",GNP51=0".
	// #7B0 -> #7FF (2026-08): +MRP00..MRP08 appended (moored-vs-underway
	// mode). STATR_REQ_CheckEmptyRequest moves #0C6 -> #0CE for MRT01, the
	// read-only state mirror (T-keys are the STATR set).
	STRCMP_EQUAL(
	    "$O;PARMR#818;IDP12=0,IDP11=FACTORY,ARP05=60,ARP01=2,ARP19=0,ARP18=0,GNP01=1,ARP11=1,ARP16=10,GNP02=1,GNP03=2,"
	    "GNP05=120,GNP04=0,UNP01=0,UNP02=0,UNP03=1,LBP01=0,LBP02=10,ARP06=240,LBP04=2,LBP05=0,LBP06=1,ARP12=4,LBP07=2,"
	    "LBP08=1,LBP09=120,UNP04=10,PPP01=15,PPP02=90,PPP03=30,PPP04=1000,PPP05=300,PPP06=10,GNP09=530,GNP10=3,GNP11=0,"
	    "GNP20=1,GNP21=5,GNP22=1,GNP23=60,ARP30=1,LDP01=1,ARP31=1,ARP32=1,ARP33=900,ARP34=90,GNP24=1,LBP10=5,LBP11=4,"
	    "ZOP01=1,ZOP04=0,ZOP05=1,ZOP06=01/01/2020 "
	    "00:00:00,ZOP08=1,ZOP10=240,ZOP11=2,ZOP12=16777215,ZOP13=0,ZOP14=4,ZOP15=2,ZOP16=5,ZOP17=240,ZOP18=-123.392,"
	    "ZOP19=-48.8752,ZOP20=1000,CTP01=0,CTP02=FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF,CTP03=1,CTP04="
	    "60,ARP35=5,GNP25=1,GNP26=0,UNP11=1.1,PHP01=0,PHP02=0,PHP03=nan,STP01=0,STP02=0,STP03=nan,LTP01=0,LTP02=0,"
	    "LTP03=nan,CDP01=0,CDP02=0,CDP03=nan,CDP04=nan,CDP05=nan,THP01=0,THP02=0,THP03=nan,THP04=0,THP05=0,AXP01=0,"
	    "AXP02=0,AXP03=0,AXP04=5,AXP08=0,AXP09=0,PRP01=0,PRP02=0,DBP01=1,GNP27=0,UNP05=1,UNP06=1,UNP07=1000,UNP08=1000,"
	    "GNP40=15,GNP41=300,UNP22=4,UNP23=3600,UNP24=7200,UNP25=5,UNP12=0,UNP13=0,GNP42=10,GNP43=10,LBP12=5,PRP03=0,"
	    "GNP28=0,STP04=0,STP05=1,STP06=1000,PHP04=0,PHP05=1,PHP06=1000,LTP04=0,LTP05=1,LTP06=1000,PRP04=0,PRP05=1,"
	    "PRP06=1000,AXP05=0,AXP06=1,AXP07=1000,THP06=0,THP07=1,THP08=1000,IDP14=,PWP05=0,LBP14=0,GNP30=0,PRP07=0,GNP31="
	    ",PWP06=0,LRP01=,LRP02=,LRP03=,LRP04=,LRP05=,LRP06=,LRP07=1,LRP08=4,LRP09=0,LRP10=3,LRP11=0,LRP12=0,LRP13=0,"
	    "LRP14=2,LRP15=1,ARP40=5,ARP41=1,ARP42=30,MTP01=0,MTP02=10,MTP03=25,MTP04=50,MTP05=3,MTP06=0,MTP07=0,ARP51="
	    "03921fb104b92859209b18abd009de96,ARP52=2c93600d6be3bac0ccfe9047c02c058e,ARP53="
	    "550b4bec21009c7a7b5bebaa937cdb41,ARP54=0,UNP20=2700,ARP43=0,UNP30=3,UNP09=200,UNP10=10000,GNP44=5,GNP45=0,"
	    "GNP46=0,AXP10=0,AXP11=50,LDP03=01/01/1970 "
	    "00:00:00,SMP00=0,SMP01=0,GNP50=86400,RLP01=0,RLP02=3600,RLP03=10,HMP00=0,HMP01=24,HMP02=3,HMP10=2,HMP11=7200,"
	    "HMP12=0,HMP13=1,GNP51=0,GNP52=0,GNP53=0,GNP54=0,ARP44=0,ARP45=4,ARP46=60,PPP07=0,PPP08=14,PPP09=0,MRP00=0,"
	    "MRP01=150,MRP02=3,MRP03=2,MRP04=900,MRP05=4,MRP06=3600,MRP07=1,MRP08=1,PPP10=0,PPP11=20,PPP12=0\r",
	    resp.c_str());
}

TEST(DTEHandler, PROFW_PROFR_REQ) {
	std::string resp;
	std::string req = "$PROFW#018;Profile Name For Tracker\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PROFW#000;\r", resp.c_str());
	req = "$PROFR#000;\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PROFR#018;Profile Name For Tracker\r", resp.c_str());
}

TEST(DTEHandler, SECUR_REQ) {
	std::string resp;
	std::string req = "$SECUR#008;12345678\r";
	CHECK_TRUE(DTEAction::SECUR == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;SECUR#000;\r", resp.c_str());
}

TEST(DTEHandler, ERASE_REQ) {
	std::string resp;
	std::string req = "$ERASE#001;3\r";
	mock().expectOneCall("truncate").onObject(mock_system_log).andReturnValue(0);
	mock().expectOneCall("truncate").onObject(mock_sensor_log).andReturnValue(0);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;ERASE#000;\r", resp.c_str());

	req = "$ERASE#001;1\r";
	mock().expectOneCall("truncate").onObject(mock_sensor_log).andReturnValue(0);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;ERASE#000;\r", resp.c_str());

	req = "$ERASE#001;2\r";
	mock().expectOneCall("truncate").onObject(mock_system_log).andReturnValue(0);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;ERASE#000;\r", resp.c_str());

	req = "$ERASE#001;0\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$N;ERASE#001;7\r", resp.c_str());
}

TEST(DTEHandler, RSTVW_REQ) {
	// Increment TX_COUNTER to 1
	configuration_store->increment_tx_counter();
	unsigned int tx_counter = configuration_store->read_param<unsigned int>(ParamID::TX_COUNTER);
	CHECK_EQUAL(1U, tx_counter);

	// This should reset TX_COUNTER to zero
	std::string resp;
	std::string req = "$RSTVW#001;1\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));

	STRCMP_EQUAL("$O;RSTVW#000;\r", resp.c_str());

	tx_counter = configuration_store->read_param<unsigned int>(ParamID::TX_COUNTER);
	CHECK_EQUAL(0U, tx_counter);

	// Increment RX_COUNTER to 1
	configuration_store->increment_rx_counter();
	unsigned int rx_counter = configuration_store->read_param<unsigned int>(ParamID::ARGOS_RX_COUNTER);
	CHECK_EQUAL(1U, rx_counter);

	// This should reset RX_COUNTER to zero
	req = "$RSTVW#001;3\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;RSTVW#000;\r", resp.c_str());

	rx_counter = configuration_store->read_param<unsigned int>(ParamID::ARGOS_RX_COUNTER);
	CHECK_EQUAL(0U, rx_counter);

	// Increment RX_TIME to 1
	configuration_store->increment_rx_time(1);
	unsigned int rx_time = configuration_store->read_param<unsigned int>(ParamID::ARGOS_RX_TIME);
	CHECK_EQUAL(1U, rx_time);

	// This should reset RX_TIME to zero
	req = "$RSTVW#001;4\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;RSTVW#000;\r", resp.c_str());

	rx_time = configuration_store->read_param<unsigned int>(ParamID::ARGOS_RX_TIME);
	CHECK_EQUAL(0U, rx_time);
}

TEST(DTEHandler, RSTBW_REQ) {
	std::string resp;
	std::string req = "$RSTBW#000;\r";
	CHECK_TRUE(DTEAction::RESET == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;RSTBW#000;\r", resp.c_str());
}

TEST(DTEHandler, FACTW_REQ) {
	std::string resp;
	std::string req = "$FACTW#000;\r";
	CHECK_TRUE(DTEAction::FACTR == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;FACTW#000;\r", resp.c_str());
}

TEST(DTEHandler, DUMPM_REQ) {
	std::string resp;
	std::string req = "$DUMPM#007;100,200\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;DUMPM#2AC;AAECAwQFBgcICQoLDA0ODxAREhMUFRYXGBkaGxwdHh8gISIjJCUmJygpKissLS4vMDEyMzQ1Njc4OTo7PD0+"
	             "P0BBQkNERUZHSElKS0xNTk9QUVJTVFVWV1hZWltcXV5fYGFiY2RlZmdoaWprbG1ub3BxcnN0dXZ3eHl6e3x9fn+"
	             "AgYKDhIWGh4iJiouMjY6PkJGSk5SVlpeYmZqbnJ2en6ChoqOkpaanqKmqq6ytrq+wsbKztLW2t7i5uru8vb6/"
	             "wMHCw8TFxsfIycrLzM3Oz9DR0tPU1dbX2Nna29zd3t/g4eLj5OXm5+jp6uvs7e7v8PHy8/T19vf4+fr7/P3+/"
	             "wABAgMEBQYHCAkKCwwNDg8QERITFBUWFxgZGhscHR4fICEiIyQlJicoKSorLC0uLzAxMjM0NTY3ODk6Ozw9Pj9AQUJDREVGR0hJSk"
	             "tMTU5PUFFSU1RVVldYWVpbXF1eX2BhYmNkZWZnaGlqa2xtbm9wcXJzdHV2d3h5ent8fX5/"
	             "gIGCg4SFhoeIiYqLjI2Oj5CRkpOUlZaXmJmam5ydnp+goaKjpKWmp6ipqqusra6vsLGys7S1tre4ubq7vL2+"
	             "v8DBwsPExcbHyMnKy8zNzs/Q0dLT1NXW19jZ2tvc3d7f4OHi4+Tl5ufo6err7O3u7/Dx8vP09fb3+Pn6+/z9/v8=\r",
	             resp.c_str());
}

TEST(DTEHandler, PASPW_REQ_KineisAllcastAop) {
	/*
	 * Real capture, not a synthetic frame: CLS retrieve-kineis-aop
	 * (api.groupcls.com/telemetry/api/v1), taken 2026-08-27T18:31 UTC.
	 * 478 bytes, sha256
	 * fcf7b67e06a33e9c3836cac208ad4f74ea0754cdcff5a3db709a423e92c332b1.
	 *
	 * Sixteen packets, each prefixed by the 8-bit transmitting satellite
	 * address: ten AOP monosat (0x26C, 25 bytes), four AOP multisat (0x35A,
	 * 38 bytes) and two long constellation status (0x62F, 38 bytes). The
	 * whole 478 bytes are consumed with nothing left over, which is the
	 * first thing worth asserting -- a codec that stops early would still
	 * look like it worked.
	 */
	std::string allcast_binary = Binascii::unhexlify(read_file_into_string("data/kineis_aop_20260827.hex"));
	CHECK_EQUAL(478U, allcast_binary.length());

	BaseRawData paspw_raw = { 0, 0, allcast_binary };

	std::string resp;
	std::string req = DTEEncoder::encode(DTECommand::PASPW_REQ, paspw_raw);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PASPW#000;\r", resp.c_str());

	/*
	 * ART03 carries the timestamp of the NEWEST bulletin in the frame, here
	 * satellite 0x62 at 05:43:36 -- not a date at midnight as the old A-DCS
	 * codec stored. Worth knowing on the GUI side: the field now has a
	 * meaningful time of day, which is what makes the AOP age computable to
	 * better than a day.
	 */
	req = "$PARMR#005;ART03\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMR#019;ART03=27/08/2026 05:43:36\r", resp.c_str());

	BasePassPredict pp = configuration_store->read_pass_predict();
	CHECK_EQUAL(25U, pp.num_records);

	/*
	 * Spot checks on four satellites chosen for what each one proves, rather
	 * than the first four in the table:
	 *   0x0B  a monosat record, decoded end to end;
	 *   0x1D  downlink OFF -- the flag is per satellite, and a codec that
	 *         defaulted it to ON would still pass every other check here;
	 *   0xAD  reached through a MULTISAT record, i.e. an address plus a delta
	 *         date relative to a reference satellite, not a full bulletin;
	 *   0xF1  uplink Argos 4 while most of the constellation is Kineis V1,
	 *         so the payload type really is read and not assumed.
	 */
	CHECK_EQUAL(0x0BU, pp.records[0].satHexId);
	CHECK_EQUAL(1U, pp.records[0].downlinkStatus);
	CHECK_EQUAL(SAT_UPLK_ON_KINEIS_V1, pp.records[0].uplinkStatus);
	CHECK_EQUAL(2026U, pp.records[0].bulletin.year);
	CHECK_EQUAL(8U, pp.records[0].bulletin.month);
	CHECK_EQUAL(27U, pp.records[0].bulletin.day);
	DOUBLES_EQUAL(7006.593, pp.records[0].semiMajorAxisKm, 0.001);
	DOUBLES_EQUAL(97.8970, pp.records[0].inclinationDeg, 0.0001);
	DOUBLES_EQUAL(203.946, pp.records[0].ascNodeLongitudeDeg, 0.001);
	DOUBLES_EQUAL(-24.328, pp.records[0].ascNodeDriftDeg, 0.001);
	DOUBLES_EQUAL(97.3098, pp.records[0].orbitPeriodMin, 0.0001);

	CHECK_EQUAL(0x1DU, pp.records[2].satHexId);
	CHECK_EQUAL(SAT_DNLK_OFF, pp.records[2].downlinkStatus);

	CHECK_EQUAL(0xADU, pp.records[17].satHexId);
	CHECK_EQUAL(2026U, pp.records[17].bulletin.year);
	CHECK_EQUAL(4U, pp.records[17].bulletin.hour);
	CHECK_EQUAL(3U, pp.records[17].bulletin.minute);
	DOUBLES_EQUAL(7032.996, pp.records[17].semiMajorAxisKm, 0.001);
	DOUBLES_EQUAL(0.154, pp.records[17].ascNodeLongitudeDeg, 0.001);

	CHECK_EQUAL(0xF1U, pp.records[20].satHexId);
	CHECK_EQUAL(SAT_UPLK_ON_ARGOS_4, pp.records[20].uplinkStatus);
	DOUBLES_EQUAL(7129.166, pp.records[20].semiMajorAxisKm, 0.001);
	DOUBLES_EQUAL(-1.00, pp.records[20].semiMajorAxisDriftMeterPerDay, 0.01);

	/*
	 * Orbital sanity over the whole table. These bounds are physics, not
	 * fitted values: a sun-synchronous Argos/Kineis orbit sits near 7000 km
	 * and 98 deg. A decoder that shifted a field by one bit would still
	 * produce a plausible-looking record here and there, but not 25 of them.
	 */
	for (unsigned int i = 0; i < pp.num_records; i++) {
		CHECK_TRUE(pp.records[i].satHexId != 0x00);
		CHECK_EQUAL(2026U, pp.records[i].bulletin.year);
		CHECK_EQUAL(8U, pp.records[i].bulletin.month);
		CHECK_EQUAL(27U, pp.records[i].bulletin.day);
		CHECK_TRUE(pp.records[i].semiMajorAxisKm > 6900.0f);
		CHECK_TRUE(pp.records[i].semiMajorAxisKm < 7300.0f);
		CHECK_TRUE(pp.records[i].inclinationDeg > 97.0f);
		CHECK_TRUE(pp.records[i].inclinationDeg < 99.5f);
		CHECK_TRUE(pp.records[i].ascNodeLongitudeDeg >= 0.0f);
		CHECK_TRUE(pp.records[i].ascNodeLongitudeDeg < 360.0f);
		CHECK_TRUE(pp.records[i].orbitPeriodMin > 95.0f);
		CHECK_TRUE(pp.records[i].orbitPeriodMin < 105.0f);
		CHECK_TRUE(pp.records[i].downlinkStatus <= SAT_DNLK_ON);
		CHECK_TRUE(pp.records[i].uplinkStatus <= SAT_UPLK_ON_KINEIS_V1);
	}

	/* Satellite addresses are unique: a decode that restarted mid-table
	 * would duplicate one rather than lose it. */
	for (unsigned int i = 0; i < pp.num_records; i++)
		for (unsigned int j = i + 1; j < pp.num_records; j++)
			CHECK_TRUE(pp.records[i].satHexId != pp.records[j].satHexId);
}

TEST(DTEHandler, PASPW_REQ_LegacyAdcsFormatIsRefused_DayOfYear) {
	/*
	 * Legacy A-DCS allcast vector, supplied by CLS: the day-of-year wider than 8 bits case.
	 *
	 * Kept on purpose rather than deleted. The PREPASS v4.0 migration made the
	 * codec accept the KINEIS allcast set only -- addresses 0x136, 0x26C,
	 * 0x35A, 0x443, 0x575, 0x62F -- and this file carries the old A-DCS
	 * service addresses (0x0137, 0x0BE5, 0x0D45...). What matters is HOW it is
	 * refused: cleanly, with VALUE_OUT_OF_RANGE, and without touching the
	 * stored table. Silently accepting it would write garbage orbital elements
	 * and the beacon would aim at satellites that are not there.
	 */
	std::string allcast_ref = read_paspw_file("data/incorrect_aop.json");
	std::string allcast_binary = Binascii::unhexlify(allcast_ref);

	BasePassPredict avant = configuration_store->read_pass_predict();

	BaseRawData paspw_raw = { 0, 0, allcast_binary };
	std::string resp;
	std::string req = DTEEncoder::encode(DTECommand::PASPW_REQ, paspw_raw);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$N;PASPW#001;5\r", resp.c_str());

	/* The stored table is untouched by a refused upload. */
	BasePassPredict apres = configuration_store->read_pass_predict();
	CHECK_EQUAL(avant.num_records, apres.num_records);
	for (unsigned int i = 0; i < avant.num_records; i++)
		CHECK_EQUAL(avant.records[i].satHexId, apres.records[i].satHexId);
}

TEST(DTEHandler, PASPW_REQ_LegacyAdcsFormatIsRefused_Argos4) {
	/*
	 * Legacy A-DCS allcast vector, supplied by CLS: the Argos 4 satellites case.
	 *
	 * Kept on purpose rather than deleted. The PREPASS v4.0 migration made the
	 * codec accept the KINEIS allcast set only -- addresses 0x136, 0x26C,
	 * 0x35A, 0x443, 0x575, 0x62F -- and this file carries the old A-DCS
	 * service addresses (0x0137, 0x0BE5, 0x0D45...). What matters is HOW it is
	 * refused: cleanly, with VALUE_OUT_OF_RANGE, and without touching the
	 * stored table. Silently accepting it would write garbage orbital elements
	 * and the beacon would aim at satellites that are not there.
	 */
	std::string allcast_ref = read_paspw_file("data/20230308105834.json");
	std::string allcast_binary = Binascii::unhexlify(allcast_ref);

	BasePassPredict avant = configuration_store->read_pass_predict();

	BaseRawData paspw_raw = { 0, 0, allcast_binary };
	std::string resp;
	std::string req = DTEEncoder::encode(DTECommand::PASPW_REQ, paspw_raw);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$N;PASPW#001;5\r", resp.c_str());

	/* The stored table is untouched by a refused upload. */
	BasePassPredict apres = configuration_store->read_pass_predict();
	CHECK_EQUAL(avant.num_records, apres.num_records);
	for (unsigned int i = 0; i < avant.num_records; i++)
		CHECK_EQUAL(avant.records[i].satHexId, apres.records[i].satHexId);
}

TEST(DTEHandler, PASPW_REQ_LegacyAdcsFormatIsRefused_TypeC) {
	/*
	 * Legacy A-DCS allcast vector, supplied by CLS: the type C satellite AOP case.
	 *
	 * Kept on purpose rather than deleted. The PREPASS v4.0 migration made the
	 * codec accept the KINEIS allcast set only -- addresses 0x136, 0x26C,
	 * 0x35A, 0x443, 0x575, 0x62F -- and this file carries the old A-DCS
	 * service addresses (0x0137, 0x0BE5, 0x0D45...). What matters is HOW it is
	 * refused: cleanly, with VALUE_OUT_OF_RANGE, and without touching the
	 * stored table. Silently accepting it would write garbage orbital elements
	 * and the beacon would aim at satellites that are not there.
	 */
	std::string allcast_ref = read_paspw_file("data/allcast.2.117.06.json");
	std::string allcast_binary = Binascii::unhexlify(allcast_ref);

	BasePassPredict avant = configuration_store->read_pass_predict();

	BaseRawData paspw_raw = { 0, 0, allcast_binary };
	std::string resp;
	std::string req = DTEEncoder::encode(DTECommand::PASPW_REQ, paspw_raw);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$N;PASPW#001;5\r", resp.c_str());

	/* The stored table is untouched by a refused upload. */
	BasePassPredict apres = configuration_store->read_pass_predict();
	CHECK_EQUAL(avant.num_records, apres.num_records);
	for (unsigned int i = 0; i < avant.num_records; i++)
		CHECK_EQUAL(avant.records[i].satHexId, apres.records[i].satHexId);
}

TEST(DTEHandler, DUMPD_REQ_SensorLog) {
	DTECommand command;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;
	std::string req = DTEEncoder::encode(DTECommand::DUMPD_REQ, BaseLogDType::GNSS_SENSOR);
	std::string resp;

	// Empty log file should just output a CSV header
	mock().expectOneCall("num_entries").onObject(mock_sensor_log).andReturnValue(0);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));

	// Decode the response and check the contents
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::DUMPD_RESP == command);
	STRCMP_EQUAL("log_datetime,batt_voltage,iTOW,fix_datetime,valid,onTime,ttff,fixType,flags,flags2,flags3,numSV,lon,"
	             "lat,height,hMSL,hAcc,vAcc,velN,velE,velD,gSpeed,headMot,sAcc,headAcc,pDOP,vDOP,hDOP,headVeh\r\n",
	             std::get<std::string>(arg_list[2]).c_str());

	// Check N entries are retrieved requiring two passes
	mock().expectOneCall("num_entries").onObject(mock_sensor_log).andReturnValue(12);
	for (unsigned int i = 0; i < 8; i++)
		mock().expectOneCall("read").onObject(mock_sensor_log).withIntParameter("index", i).ignoreOtherParameters();
	CHECK_TRUE(DTEAction::AGAIN == dte_handler->handle_dte_message(req, resp));

	arg_list.clear();
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::DUMPD_RESP == command);
	printf(std::get<std::string>(arg_list[2]).c_str());

	mock().expectOneCall("num_entries").onObject(mock_sensor_log).andReturnValue(12);
	for (unsigned int i = 8; i < 12; i++)
		mock().expectOneCall("read").onObject(mock_sensor_log).withIntParameter("index", i).ignoreOtherParameters();
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));

	arg_list.clear();
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::DUMPD_RESP == command);
	printf(std::get<std::string>(arg_list[2]).c_str());

	mock().checkExpectations();
}

TEST(DTEHandler, DUMPD_REQ_InternalLog) {
	DTECommand command;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;
	std::string req = DTEEncoder::encode(DTECommand::DUMPD_REQ, BaseLogDType::INTERNAL);
	std::string resp;

	// Empty log file should just output a CSV header
	mock().expectOneCall("num_entries").onObject(mock_system_log).andReturnValue(0);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));

	// Decode the response and check the contents
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::DUMPD_RESP == command);
	STRCMP_EQUAL("log_datetime,log_level,message\r\n", std::get<std::string>(arg_list[2]).c_str());

	// Check N entries are retrieved requiring two passes
	mock().expectOneCall("num_entries").onObject(mock_system_log).andReturnValue(12);
	for (unsigned int i = 0; i < 8; i++)
		mock().expectOneCall("read").onObject(mock_system_log).withIntParameter("index", i).ignoreOtherParameters();
	CHECK_TRUE(DTEAction::AGAIN == dte_handler->handle_dte_message(req, resp));

	arg_list.clear();
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::DUMPD_RESP == command);
	printf(std::get<std::string>(arg_list[2]).c_str());

	mock().expectOneCall("num_entries").onObject(mock_system_log).andReturnValue(12);
	for (unsigned int i = 8; i < 12; i++)
		mock().expectOneCall("read").onObject(mock_system_log).withIntParameter("index", i).ignoreOtherParameters();
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));

	arg_list.clear();
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::DUMPD_RESP == command);
	printf(std::get<std::string>(arg_list[2]).c_str());

	mock().checkExpectations();
}

TEST(DTEHandler, WritingReadOnlyAttributesIsIgnored) {
	std::string resp;
	std::string req = "$PARMW#007;ART02=1\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	// Read-only param ART02 is rejected, listed in response
	STRCMP_EQUAL("$N;PARMW#005;ART02\r", resp.c_str());

	// Read-only param was not written
	unsigned int tx_counter = configuration_store->read_param<unsigned int>(ParamID::TX_COUNTER);
	CHECK_EQUAL(0, tx_counter);
}

TEST(DTEHandler, WritingOutOfRangeValue) {
	std::string resp;
	std::string req = "$PARMW#009;PPP01=-12\r";
	// Out-of-range param is skipped, valid params still written, rejected key listed
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$N;PARMW#005;PPP01\r", resp.c_str());
}


TEST(DTEHandler, GenerateDefaultPassPredictFile) {
	/*
	 * Developer utility, not an assertion: it prints the decoded table in the
	 * exact form ConfigurationStore::default_prepass expects, so the built-in
	 * AOP can be refreshed by pasting the output.
	 *
	 * It now reads the KINEIS capture. data/default_aop.json is an A-DCS
	 * vector and the codec refuses it, which is correct -- but it left the
	 * only tool for regenerating the default table unusable.
	 */
	std::string allcast_binary = Binascii::unhexlify(read_file_into_string("data/kineis_aop_20260827.hex"));

	BaseRawData paspw_raw = { 0, 0, allcast_binary };

	std::string resp;
	std::string req = DTEEncoder::encode(DTECommand::PASPW_REQ, paspw_raw);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PASPW#000;\r", resp.c_str());

	// Read out the prepass file
	BasePassPredict pp;
	pp = configuration_store->read_pass_predict();
	for (unsigned int i = 0; i < pp.num_records; i++) {
		printf("{ 0x%1x, (SatDownlinkStatus_t)%u, (SatUplinkStatus_t)%u, { %u, %u, %u, %u, %u, %u }, %f, %f, %f, %f, "
		       "%f, %f },\n",
		       pp.records[i].satHexId, pp.records[i].downlinkStatus, pp.records[i].uplinkStatus,
		       pp.records[i].bulletin.year, pp.records[i].bulletin.month, pp.records[i].bulletin.day,
		       pp.records[i].bulletin.hour, pp.records[i].bulletin.minute, pp.records[i].bulletin.second,
		       (double)pp.records[i].semiMajorAxisKm, (double)pp.records[i].inclinationDeg,
		       (double)pp.records[i].ascNodeLongitudeDeg, (double)pp.records[i].ascNodeDriftDeg,
		       (double)pp.records[i].orbitPeriodMin, (double)pp.records[i].semiMajorAxisDriftMeterPerDay);
	}
}

TEST(DTEHandler, SCALW_REQ) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// 0=>AXL
	req = DTEEncoder::encode(DTECommand::SCALW_REQ, 0U, 0U, 0.0);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SCALW_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::INCORRECT_DATA == error_code);

	// 1=>PRS
	req = DTEEncoder::encode(DTECommand::SCALW_REQ, 1U, 0U, 0.0);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SCALW_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::INCORRECT_DATA == error_code);

	// 2=>ALS
	req = DTEEncoder::encode(DTECommand::SCALW_REQ, 2U, 0U, 0.0);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SCALW_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::INCORRECT_DATA == error_code);

	// Invoke AXL sensor
	MockSensor s1("AXL");
	mock()
	    .expectOneCall("calibration_write")
	    .onObject(&s1)
	    .withDoubleParameter("value", 0.0)
	    .withUnsignedIntParameter("offset", 0U);

	req = DTEEncoder::encode(DTECommand::SCALW_REQ, 0U, 0U, 0.0);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SCALW_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::OK == error_code);

	// Invoke PRS sensor
	MockSensor s2("PRS");
	mock()
	    .expectOneCall("calibration_write")
	    .onObject(&s2)
	    .withDoubleParameter("value", 1.0)
	    .withUnsignedIntParameter("offset", 0U);

	req = DTEEncoder::encode(DTECommand::SCALW_REQ, 1U, 0U, 1.0);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SCALW_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::OK == error_code);

	// Invoke PRS sensor
	MockSensor s3("ALS");
	mock()
	    .expectOneCall("calibration_write")
	    .onObject(&s3)
	    .withDoubleParameter("value", 2.0)
	    .withUnsignedIntParameter("offset", 0U);

	req = DTEEncoder::encode(DTECommand::SCALW_REQ, 2U, 0U, 2.0);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SCALW_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::OK == error_code);
}

TEST(DTEHandler, SCALR_REQ_NoSensorRegistered) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// No AXL sensor registered - should get INCORRECT_DATA
	req = DTEEncoder::encode(DTECommand::SCALR_REQ, 0U, 0U);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SCALR_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::INCORRECT_DATA == error_code);

	// No PRS sensor registered
	req = DTEEncoder::encode(DTECommand::SCALR_REQ, 1U, 0U);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SCALR_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::INCORRECT_DATA == error_code);

	// No ALS sensor registered
	req = DTEEncoder::encode(DTECommand::SCALR_REQ, 2U, 0U);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SCALR_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::INCORRECT_DATA == error_code);
}

TEST(DTEHandler, SCALR_REQ_ReadCalibration) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Register AXL sensor and read calibration offset 0
	MockSensor s1("AXL");
	mock().expectOneCall("calibration_read").onObject(&s1).withUnsignedIntParameter("offset", 0U).andReturnValue(1.234);

	req = DTEEncoder::encode(DTECommand::SCALR_REQ, 0U, 0U);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SCALR_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::OK == error_code);
	DOUBLES_EQUAL(1.234, std::get<double>(arg_list[0]), 0.001);
}

TEST(DTEHandler, SCALR_REQ_ReadMultipleSensors) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Register PRS and ALS sensors
	MockSensor prs("PRS");
	MockSensor als("ALS");

	// Read PRS calibration at offset 0
	mock()
	    .expectOneCall("calibration_read")
	    .onObject(&prs)
	    .withUnsignedIntParameter("offset", 0U)
	    .andReturnValue(1013.25);
	req = DTEEncoder::encode(DTECommand::SCALR_REQ, 1U, 0U);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SCALR_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::OK == error_code);
	DOUBLES_EQUAL(1013.25, std::get<double>(arg_list[0]), 0.01);

	// Read ALS calibration at offset 1
	arg_list.clear();
	mock().expectOneCall("calibration_read").onObject(&als).withUnsignedIntParameter("offset", 1U).andReturnValue(42.5);
	req = DTEEncoder::encode(DTECommand::SCALR_REQ, 2U, 1U);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SCALR_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::OK == error_code);
	DOUBLES_EQUAL(42.5, std::get<double>(arg_list[0]), 0.01);
}

TEST(DTEHandler, SENSR_REQ_BatteryOnly) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Set battery values: level=80%, voltage=3700mV
	fake_battery_monitor->set_values(80, 3700, false, false);

	// Request battery only (mask=0x01, timeout=10s)
	req = DTEEncoder::encode(DTECommand::SENSR_REQ, 1U, 10U);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SENSR_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::OK == error_code);
	CHECK_EQUAL(3700U, std::get<unsigned int>(arg_list[0]));  // batt_mv
	CHECK_EQUAL(80U, std::get<unsigned int>(arg_list[1]));    // batt_soc
}

TEST(DTEHandler, SENSR_REQ_PressureOnly) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Register PRS sensor
	MockSensor prs("PRS");
	mock().expectOneCall("read").onObject(&prs).withUnsignedIntParameter("port", 0).andReturnValue(1013.25);
	mock().expectOneCall("read").onObject(&prs).withUnsignedIntParameter("port", 1).andReturnValue(22.5);
	mock()
	    .expectOneCall("calibration_read")
	    .onObject(&prs)
	    .withUnsignedIntParameter("offset", 0U)
	    .andReturnValue(1013.25);

	// Request pressure only (mask=0x02, timeout=10s)
	req = DTEEncoder::encode(DTECommand::SENSR_REQ, 2U, 10U);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SENSR_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::OK == error_code);
	DOUBLES_EQUAL(1013.25, std::get<double>(arg_list[2]), 0.01);  // pressure
	DOUBLES_EQUAL(22.5, std::get<double>(arg_list[3]), 0.01);     // temperature
	CHECK(std::get<double>(arg_list[4]) != 0.0);                  // altitude (computed)
}

TEST(DTEHandler, SENSR_REQ_GNSS_NoFix) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// No GPS fix stored - should return defaults
	req = DTEEncoder::encode(DTECommand::SENSR_REQ, 4U, 10U);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SENSR_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::OK == error_code);
	DOUBLES_EQUAL(0.0, std::get<double>(arg_list[5]), 0.001);  // lat
	DOUBLES_EQUAL(0.0, std::get<double>(arg_list[6]), 0.001);  // lon
	DOUBLES_EQUAL(99.9, std::get<double>(arg_list[7]), 0.1);   // hdop (no fix)
	CHECK_EQUAL(0U, std::get<unsigned int>(arg_list[8]));      // num_sv
}

TEST(DTEHandler, SENSR_REQ_GNSS_ValidFix) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Set a valid GPS fix
	GPSLogEntry gps_entry = {};
	gps_entry.info.valid = 1;
	gps_entry.info.lat = 48.8566;
	gps_entry.info.lon = 2.3522;
	gps_entry.info.hDOP = 1.2f;
	gps_entry.info.numSV = 12;
	configuration_store->notify_gps_location(gps_entry);

	// Request GNSS (mask=0x04, timeout=10s)
	req = DTEEncoder::encode(DTECommand::SENSR_REQ, 4U, 10U);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SENSR_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::OK == error_code);
	DOUBLES_EQUAL(48.8566, std::get<double>(arg_list[5]), 0.001);  // lat
	DOUBLES_EQUAL(2.3522, std::get<double>(arg_list[6]), 0.001);   // lon
	DOUBLES_EQUAL(1.2, std::get<double>(arg_list[7]), 0.1);        // hdop
	CHECK_EQUAL(12U, std::get<unsigned int>(arg_list[8]));         // num_sv
}

TEST(DTEHandler, SENSR_REQ_AllSensors) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Set battery values
	fake_battery_monitor->set_values(95, 4100, false, false);

	// Register PRS sensor
	MockSensor prs("PRS");
	mock().expectOneCall("read").onObject(&prs).withUnsignedIntParameter("port", 0).andReturnValue(1005.0);
	mock().expectOneCall("read").onObject(&prs).withUnsignedIntParameter("port", 1).andReturnValue(21.0);
	mock()
	    .expectOneCall("calibration_read")
	    .onObject(&prs)
	    .withUnsignedIntParameter("offset", 0U)
	    .andReturnValue(1013.25);

	// Register AXL sensor
	MockSensor axl("AXL");
	mock().expectOneCall("read").onObject(&axl).withUnsignedIntParameter("port", 1).andReturnValue(0.01);
	mock().expectOneCall("read").onObject(&axl).withUnsignedIntParameter("port", 2).andReturnValue(-0.02);
	mock().expectOneCall("read").onObject(&axl).withUnsignedIntParameter("port", 3).andReturnValue(1.0);
	mock().expectOneCall("read").onObject(&axl).withUnsignedIntParameter("port", 0).andReturnValue(25.0);
	mock().expectOneCall("read").onObject(&axl).withUnsignedIntParameter("port", 4).andReturnValue(3.0);

	// Set a valid GPS fix
	GPSLogEntry gps_entry = {};
	gps_entry.info.valid = 1;
	gps_entry.info.lat = 43.6;
	gps_entry.info.lon = 1.44;
	gps_entry.info.hDOP = 0.9f;
	gps_entry.info.numSV = 8;
	configuration_store->notify_gps_location(gps_entry);

	// Request all sensors (mask=0x0F, timeout=30s)
	req = DTEEncoder::encode(DTECommand::SENSR_REQ, 15U, 30U);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SENSR_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::OK == error_code);

	// Battery
	CHECK_EQUAL(4100U, std::get<unsigned int>(arg_list[0]));  // batt_mv
	CHECK_EQUAL(95U, std::get<unsigned int>(arg_list[1]));    // batt_soc

	// Pressure
	DOUBLES_EQUAL(1005.0, std::get<double>(arg_list[2]), 0.01);  // pressure
	DOUBLES_EQUAL(21.0, std::get<double>(arg_list[3]), 0.01);    // temperature
	CHECK(std::get<double>(arg_list[4]) != 0.0);                 // altitude (computed)

	// GNSS
	DOUBLES_EQUAL(43.6, std::get<double>(arg_list[5]), 0.001);  // lat
	DOUBLES_EQUAL(1.44, std::get<double>(arg_list[6]), 0.001);  // lon
	DOUBLES_EQUAL(0.9, std::get<double>(arg_list[7]), 0.1);     // hdop
	CHECK_EQUAL(8U, std::get<unsigned int>(arg_list[8]));       // num_sv

	// Accelerometer
	DOUBLES_EQUAL(0.01, std::get<double>(arg_list[9]), 0.001);    // accel_x
	DOUBLES_EQUAL(-0.02, std::get<double>(arg_list[10]), 0.001);  // accel_y
	DOUBLES_EQUAL(1.0, std::get<double>(arg_list[11]), 0.001);    // accel_z
	DOUBLES_EQUAL(25.0, std::get<double>(arg_list[12]), 0.1);     // accel_temp
	CHECK_EQUAL(3U, std::get<unsigned int>(arg_list[13]));        // activity
}

TEST(DTEHandler, SENSR_REQ_NoPressureSensor) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// No PRS sensor registered - pressure request should return defaults
	req = DTEEncoder::encode(DTECommand::SENSR_REQ, 2U, 10U);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SENSR_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::OK == error_code);
	DOUBLES_EQUAL(0.0, std::get<double>(arg_list[2]), 0.001);  // pressure default
	DOUBLES_EQUAL(0.0, std::get<double>(arg_list[3]), 0.001);  // temperature default
	DOUBLES_EQUAL(0.0, std::get<double>(arg_list[4]), 0.001);  // altitude default
}

TEST(DTEHandler, SECUR_REQ_ValidStaticCode) {
	std::string req;
	std::string resp;
	DTECommand command;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// SECUR_REQ with the correct static access code 0x12345678
	req = DTEEncoder::encode(DTECommand::SECUR_REQ, 0x12345678U);
	CHECK_TRUE(DTEAction::SECUR == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SECUR_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::OK, error_code);
}

TEST(DTEHandler, SECUR_REQ_ValidDynamicCode) {
	std::string req;
	std::string resp;
	DTECommand command;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Set device ARGOS_DECID and use it as access code
	configuration_store->write_param(ParamID::ARGOS_DECID, 0xAABBCCDDU);
	req = DTEEncoder::encode(DTECommand::SECUR_REQ, 0xAABBCCDDU);
	CHECK_TRUE(DTEAction::SECUR == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SECUR_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::OK, error_code);
}

TEST(DTEHandler, SECUR_REQ_InvalidCode) {
	std::string req;
	std::string resp;
	DTECommand command;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// SECUR_REQ with an invalid access code (action is still SECUR since decode succeeded)
	req = DTEEncoder::encode(DTECommand::SECUR_REQ, 0xDEADBEEFU);
	CHECK_TRUE(DTEAction::SECUR == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SECUR_RESP == command);
	// Note: decoder %1u only reads single-digit error codes, so 12 decodes as 1
	CHECK_TRUE(error_code != 0);
}

TEST(DTEHandler, RSTVW_REQ_ResetTxCounter) {
	std::string req;
	std::string resp;
	DTECommand command;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Set TX_COUNTER to non-zero
	unsigned int counter = 42;
	configuration_store->write_param(ParamID::TX_COUNTER, counter);

	// Reset TX_COUNTER (variable_id=1)
	req = DTEEncoder::encode(DTECommand::RSTVW_REQ, 1U);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::RSTVW_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::OK, error_code);

	// Verify counter was reset
	CHECK_EQUAL(0U, configuration_store->read_param<unsigned int>(ParamID::TX_COUNTER));
}

TEST(DTEHandler, RSTVW_REQ_ResetRxCounter) {
	std::string req;
	std::string resp;
	DTECommand command;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Set RX_COUNTER to non-zero
	unsigned int counter = 99;
	configuration_store->write_param(ParamID::ARGOS_RX_COUNTER, counter);

	// Reset RX_COUNTER (variable_id=3)
	req = DTEEncoder::encode(DTECommand::RSTVW_REQ, 3U);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::RSTVW_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::OK, error_code);

	CHECK_EQUAL(0U, configuration_store->read_param<unsigned int>(ParamID::ARGOS_RX_COUNTER));
}

TEST(DTEHandler, RSTVW_REQ_ResetRxTime) {
	std::string req;
	std::string resp;
	DTECommand command;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Set RX_TIME to non-zero
	unsigned int rx_time = 500;
	configuration_store->write_param(ParamID::ARGOS_RX_TIME, rx_time);

	// Reset RX_TIME (variable_id=4)
	req = DTEEncoder::encode(DTECommand::RSTVW_REQ, 4U);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::RSTVW_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::OK, error_code);

	CHECK_EQUAL(0U, configuration_store->read_param<unsigned int>(ParamID::ARGOS_RX_TIME));
}

TEST(DTEHandler, PROFW_REQ_WriteProfile) {
	std::string req;
	std::string resp;
	DTECommand command;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Write a profile name
	req = DTEEncoder::encode(DTECommand::PROFW_REQ, std::string("TestProfile"));
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::PROFW_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::OK, error_code);

	// Verify profile was written
	STRCMP_EQUAL("TestProfile", configuration_store->read_param<std::string>(ParamID::PROFILE_NAME).c_str());
}

TEST(DTEHandler, PROFR_REQ_ReadProfile) {
	std::string req;
	std::string resp;
	DTECommand command;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Write a profile name first
	configuration_store->write_param(ParamID::PROFILE_NAME, std::string("MyTracker"));

	// Read it back via DTE
	req = DTEEncoder::encode(DTECommand::PROFR_REQ);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::PROFR_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::OK, error_code);
	STRCMP_EQUAL("MyTracker", std::get<std::string>(arg_list[0]).c_str());
}

TEST(DTEHandler, SCALW_REQ_WriteCalibration) {
	std::string req;
	std::string resp;
	DTECommand command;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Register a sensor that supports calibration
	MockSensor axl("AXL");
	mock().expectOneCall("calibration_write").onObject(&axl).withParameter("value", 1.5).withParameter("offset", 0);

	// SCALW_REQ: device_id=0 (AXL), offset=0, value=1.5
	req = DTEEncoder::encode(DTECommand::SCALW_REQ, 0U, 0U, 1.5);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SCALW_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::OK, error_code);
}

TEST(DTEHandler, SCALW_REQ_NoSensorRegistered) {
	std::string req;
	std::string resp;
	DTECommand command;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// SCALW_REQ with no sensor registered
	req = DTEEncoder::encode(DTECommand::SCALW_REQ, 0U, 0U, 1.0);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SCALW_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::INCORRECT_DATA, error_code);
}

TEST(DTEHandler, DUMPD_REQ_InvalidDType) {
	std::string req;
	std::string resp;
	DTECommand command;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// DUMPD_REQ with d_type that has no registered logger (e.g. ALS=2)
	req = DTEEncoder::encode(DTECommand::DUMPD_REQ, (unsigned int)BaseLogDType::ALS_SENSOR);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::DUMPD_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::INCORRECT_DATA, error_code);
}

TEST(DTEHandler, SENSR_REQ_NoBatteryMonitor) {
	std::string req;
	std::string resp;
	DTECommand command;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Set battery_monitor to nullptr
	BatteryMonitor *saved = battery_monitor;
	battery_monitor = nullptr;

	// Request battery reading (mask=0x01)
	req = DTEEncoder::encode(DTECommand::SENSR_REQ, 1U, 10U);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SENSR_RESP == command);
	CHECK_TRUE((unsigned int)DTEError::OK == error_code);
	// Battery values should be 0 when monitor is not available
	CHECK_EQUAL(0U, std::get<unsigned int>(arg_list[0]));  // batt_mv
	CHECK_EQUAL(0U, std::get<unsigned int>(arg_list[1]));  // batt_soc

	// Restore battery_monitor
	battery_monitor = saved;
}

TEST(DTEHandler, ERASE_REQ_InvalidLogType) {
	std::string req;
	std::string resp;
	DTECommand command;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// ERASE_REQ with d_type that has no registered logger (ALS=4)
	req = DTEEncoder::encode(DTECommand::ERASE_REQ, 4U);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::ERASE_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::INCORRECT_DATA, error_code);
}

TEST(DTEHandler, PARMW_PRP07_PressureSensorFullScale) {
	std::string resp;

	// Write PRP07=0 (FS_1260)
	std::string req = "$PARMW#007;PRP07=0\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMW#000;\r", resp.c_str());

	// Verify value was written
	auto val = configuration_store->read_param<BasePressureSensorFullScale>(ParamID::PRESSURE_SENSOR_FULL_SCALE);
	CHECK_TRUE(BasePressureSensorFullScale::FS_1260 == val);

	// Write PRP07=1 (FS_4060)
	req = "$PARMW#007;PRP07=1\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMW#000;\r", resp.c_str());

	val = configuration_store->read_param<BasePressureSensorFullScale>(ParamID::PRESSURE_SENSOR_FULL_SCALE);
	CHECK_TRUE(BasePressureSensorFullScale::FS_4060 == val);
}

TEST(DTEHandler, PARMR_PRP07_PressureSensorFullScale) {
	std::string resp;

	// Set to FS_4060
	auto fs = BasePressureSensorFullScale::FS_4060;
	configuration_store->write_param(ParamID::PRESSURE_SENSOR_FULL_SCALE, fs);

	// Read back via DTE
	std::string req = "$PARMR#005;PRP07\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMR#007;PRP07=1\r", resp.c_str());
}

// ========================================================================
// LoRa RAK3172 parameter tests (LRP01-LRP15)
// ========================================================================

TEST(DTEHandler, PARMR_LoRa_Defaults) {
	std::string resp;
	// Read all LoRa params at once
	std::string req =
	    "$PARMR#059;LRP01,LRP02,LRP03,LRP04,LRP05,LRP06,LRP07,LRP08,LRP09,LRP10,LRP11,LRP12,LRP13,LRP14,LRP15\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMR#071;LRP01=,LRP02=,LRP03=,LRP04=,LRP05=,LRP06=,LRP07=1,LRP08=4,LRP09=0,LRP10=3,LRP11=0,LRP12="
	             "0,LRP13=0,LRP14=2,LRP15=1\r",
	             resp.c_str());
}

TEST(DTEHandler, PARMW_LoRa_APPEUI) {
	std::string resp;
	std::string req = "$PARMW#016;LRP02=0102030405060708\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMW#000;\r", resp.c_str());
	STRCMP_EQUAL("0102030405060708", configuration_store->read_param<std::string>(ParamID::LORA_APPEUI).c_str());
}

TEST(DTEHandler, PARMW_LoRa_APPKEY) {
	std::string resp;
	std::string req = "$PARMW#026;LRP03=0102030405060708090A0B0C0D0E0F10\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMW#000;\r", resp.c_str());
	STRCMP_EQUAL("0102030405060708090A0B0C0D0E0F10",
	             configuration_store->read_param<std::string>(ParamID::LORA_APPKEY).c_str());
}

TEST(DTEHandler, PARMW_LoRa_DEVEUI_Writable) {
	std::string resp;
	// DevEUI (LRP01) is now writable — write should succeed
	std::string req = "$PARMW#016;LRP01=AABBCCDDEEFF0011\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMW#000;\r", resp.c_str());
	STRCMP_EQUAL("AABBCCDDEEFF0011", configuration_store->read_param<std::string>(ParamID::LORA_DEVEUI).c_str());
}

TEST(DTEHandler, PARMW_LoRa_NJM) {
	std::string resp;
	// NJM: 0=ABP, 1=OTAA (default)
	std::string req = "$PARMW#007;LRP07=0\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMW#000;\r", resp.c_str());
	CHECK_EQUAL(0U, configuration_store->read_param<unsigned int>(ParamID::LORA_NJM));

	// Read back
	req = "$PARMR#005;LRP07\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMR#007;LRP07=0\r", resp.c_str());
}

TEST(DTEHandler, PARMW_LoRa_DR) {
	std::string resp;
	std::string req = "$PARMW#007;LRP10=3\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMW#000;\r", resp.c_str());
	CHECK_EQUAL(3U, configuration_store->read_param<unsigned int>(ParamID::LORA_DR));
}

TEST(DTEHandler, PARMW_LoRa_LP_MODE) {
	std::string resp;
	// LP mode: 0=shutdown, 1=standby (default)
	std::string req = "$PARMW#007;LRP15=0\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMW#000;\r", resp.c_str());
	CHECK_EQUAL(0U, configuration_store->read_param<unsigned int>(ParamID::LORA_LP_MODE));

	// Set back to standby
	req = "$PARMW#007;LRP15=1\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	CHECK_EQUAL(1U, configuration_store->read_param<unsigned int>(ParamID::LORA_LP_MODE));
}

TEST(DTEHandler, PARMW_LoRa_LP_MODE_OutOfRange) {
	std::string resp;
	// LP mode range is 0-1, value 2 should be rejected with key in response
	std::string req = "$PARMW#007;LRP15=2\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$N;PARMW#005;LRP15\r", resp.c_str());
}

TEST(DTEHandler, PARMR_UNP_NewParams) {
	std::string req;
	std::string resp;

	// Read UNP04, UNP05, UNP22, UNP25 together — verify default values
	// "UNP04,UNP05,UNP22,UNP25" = 23 chars = 0x17
	req = "$PARMR#017;UNP04,UNP05,UNP22,UNP25\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	// "UNP04=10,UNP05=1,UNP22=4,UNP25=5" = 32 chars = 0x20
	STRCMP_EQUAL("$O;PARMR#020;UNP04=10,UNP05=1,UNP22=4,UNP25=5\r", resp.c_str());
}

TEST(DTEHandler, PARMW_UNP04_SamplingSurfFreq) {
	std::string req;
	std::string resp;

	// Write and readback UNP04 (SAMPLING_SURF_FREQ)
	// "UNP04=30" = 8 chars = 0x008
	req = "$PARMW#008;UNP04=30\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMW#000;\r", resp.c_str());
	DOUBLES_EQUAL(30.0, configuration_store->read_param<double>(ParamID::SAMPLING_SURF_FREQ), 1e-9);

	// Readback via PARMR
	req = "$PARMR#005;UNP04\r";
	dte_handler->handle_dte_message(req, resp);
	STRCMP_EQUAL("$O;PARMR#008;UNP04=30\r", resp.c_str());
}

TEST(DTEHandler, PARMW_UNP05_UwMaxSamples) {
	std::string req;
	std::string resp;

	// Write UNP05 (UW_MAX_SAMPLES)
	// "UNP05=5" = 7 chars = 0x007
	req = "$PARMW#007;UNP05=5\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMW#000;\r", resp.c_str());
	CHECK_EQUAL(5U, configuration_store->read_param<unsigned int>(ParamID::UW_MAX_SAMPLES));
}

TEST(DTEHandler, PARMW_UNP22_SWSAnalogHysteresis) {
	std::string req;
	std::string resp;

	// Write UNP22 (SWS_ANALOG_HYSTERESIS) in range 0-50
	// "UNP22=20" = 8 chars = 0x008
	req = "$PARMW#008;UNP22=20\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMW#000;\r", resp.c_str());
	CHECK_EQUAL(20U, configuration_store->read_param<unsigned int>(ParamID::SWS_ANALOG_HYSTERESIS));
}

TEST(DTEHandler, PARMW_UNP22_SWSAnalogHysteresis_OutOfRange) {
	std::string req;
	std::string resp;

	// Write UNP22=51 which exceeds max of 50 → rejected key listed in response
	// "UNP22=51" = 8 chars = 0x008
	req = "$PARMW#008;UNP22=51\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$N;PARMW#005;UNP22\r", resp.c_str());
}

TEST(DTEHandler, PARMW_UNP25_UwMinSurfaceTime) {
	std::string req;
	std::string resp;

	// Write UNP25 (UW_MIN_SURFACE_TIME)
	// "UNP25=30" = 8 chars = 0x008
	req = "$PARMW#008;UNP25=30\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMW#000;\r", resp.c_str());
	CHECK_EQUAL(30U, configuration_store->read_param<unsigned int>(ParamID::UW_MIN_SURFACE_TIME));
}

TEST(DTEHandler, ARGOSTX_REQ_StoredMode_NoSatelliteDevice) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Stored mode: SATTX,<modulation>,<size> — "0,10" = 4 chars = 0x004
	req = "$SATTX#004;0,10\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::ARGOSTX_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::INCORRECT_DATA, error_code);
}

TEST(DTEHandler, ARGOSTX_REQ_StoredModeWithTcxo_NoSatelliteDevice) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Stored mode with tcxo: SATTX,<modulation>,<size>,<tcxo> — "1,24,5" = 6 chars = 0x006
	req = "$SATTX#006;1,24,5\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::ARGOSTX_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::INCORRECT_DATA, error_code);
}

TEST(DTEHandler, ARGOSTX_REQ_CustomMode_NoSatelliteDevice) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Custom mode: SATTX,<modulation>,<radioconf_32chars>,<size>
	// "1,0123456789ABCDEF0123456789ABCDEF,24" = 37 chars = 0x025
	req = "$SATTX#025;1,0123456789ABCDEF0123456789ABCDEF,24\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::ARGOSTX_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::INCORRECT_DATA, error_code);
}

TEST(DTEHandler, ARGOSTX_REQ_InvalidSize) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// LDK (mod=0) max size is 16, send 24 → INCORRECT_DATA
	req = "$SATTX#004;0,24\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::ARGOSTX_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::INCORRECT_DATA, error_code);
}

TEST(DTEHandler, PWRON_REQ_PowerOnAll) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	mock().ignoreOtherCalls();

	// PWRON_REQ component=0 (ALL)
	req = "$PWRON#001;0\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::PWRON_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::OK, error_code);
}

TEST(DTEHandler, PWRON_REQ_PowerOff) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	mock().ignoreOtherCalls();

	// PWRON_REQ component=4 (OFF)
	req = "$PWRON#001;4\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::PWRON_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::OK, error_code);
}

TEST(DTEHandler, PWRON_REQ_InvalidComponent) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	mock().ignoreOtherCalls();

	// PWRON_REQ component=99 (invalid)
	req = "$PWRON#002;99\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::PWRON_RESP == command);
	CHECK_TRUE(error_code != 0);
}

TEST(DTEHandler, PWRON_REQ_PowerOnGNSS_NoDevice) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	mock().ignoreOtherCalls();

	// PWRON_REQ component=1 (GNSS) with no GPS device — returns error
	gps_device = nullptr;
	req = "$PWRON#001;1\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::PWRON_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::INCORRECT_DATA, error_code);
}

TEST(DTEHandler, PWRON_REQ_PowerOnSensors) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	mock().ignoreOtherCalls();

	// PWRON_REQ component=2 (SENSORS)
	req = "$PWRON#001;2\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::PWRON_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::OK, error_code);
}

TEST(DTEHandler, PARMW_REQ_OutOfRangeReturnsError) {
	std::string resp;

	// Write an out-of-range value for TR_NOM (valid range: 30-1200)
	// ARP05=29 is rejected, key listed in response
	std::string req = "$PARMW#008;ARP05=29\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$N;PARMW#005;ARP05\r", resp.c_str());

	// Verify TR_NOM was NOT changed (out-of-range value was skipped)
	unsigned int val = configuration_store->read_param<unsigned int>(ParamID::TR_NOM);
	CHECK_EQUAL(60U, val);
}

TEST(DTEHandler, PARMW_REQ_UnknownParamKey) {
	std::string resp;

	// Write with a non-existent param key - unknown key is skipped and listed in response
	// "XXXXX=123" = 9 chars = 0x09
	std::string req = "$PARMW#009;XXXXX=123\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	// Unknown key is rejected, listed in response
	STRCMP_EQUAL("$N;PARMW#005;XXXXX\r", resp.c_str());
}

// PyLinkit does not always push a whole configuration in one PARMW. A large set
// is split into several commands, and the beacon may be reset between them. Each
// batch must land without disturbing what an earlier one wrote.
//
// This works because PARMW_REQ only touches the parameters it was given, while
// serialize_config() always writes the complete in-RAM table to config.dat. The
// test pins that contract: nothing in a later batch may clear an earlier one,
// and everything must survive a reload from flash.
TEST(DTEHandler, PARMW_BatchedWritesDoNotClobberEachOther) {
	std::string resp;

	// Batch 1 -- Argos timing
	std::string b1 = "$PARMW#010;ARP05=90,ARP16=8\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(b1, resp));
	STRCMP_EQUAL("$O;PARMW#000;\r", resp.c_str());
	configuration_store->save_params();

	// Batch 2 -- GNSS, a completely different part of the table
	std::string b2 = "$PARMW#00F;GNP01=0,GNP02=1\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(b2, resp));
	STRCMP_EQUAL("$O;PARMW#000;\r", resp.c_str());
	configuration_store->save_params();

	// Batch 3 -- a string and a low-battery threshold
	std::string b3 = "$PARMW#014;IDP11=FIELD,LBP02=25\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(b3, resp));
	STRCMP_EQUAL("$O;PARMW#000;\r", resp.c_str());
	configuration_store->save_params();

	// Every batch is still there, in RAM
	CHECK_EQUAL(90U, configuration_store->read_param<unsigned int>(ParamID::TR_NOM));
	CHECK_TRUE(BaseDepthPile::DEPTH_PILE_8
	           == configuration_store->read_param<BaseDepthPile>(ParamID::ARGOS_DEPTH_PILE));
	CHECK_FALSE(configuration_store->read_param<bool>(ParamID::GNSS_EN));
	CHECK_TRUE(configuration_store->read_param<bool>(ParamID::GNSS_HDOPFILT_EN));
	STRCMP_EQUAL("FIELD", configuration_store->read_param<std::string>(ParamID::PROFILE_NAME).c_str());
	CHECK_EQUAL(25U, configuration_store->read_param<unsigned int>(ParamID::LB_THRESHOLD));

	// ...and after a reboot, read back from config.dat
	delete store;
	store = new LFSConfigurationStore(*ram_filesystem);
	store->init();
	configuration_store = store;

	CHECK_EQUAL(90U, configuration_store->read_param<unsigned int>(ParamID::TR_NOM));
	CHECK_TRUE(BaseDepthPile::DEPTH_PILE_8
	           == configuration_store->read_param<BaseDepthPile>(ParamID::ARGOS_DEPTH_PILE));
	CHECK_FALSE(configuration_store->read_param<bool>(ParamID::GNSS_EN));
	CHECK_TRUE(configuration_store->read_param<bool>(ParamID::GNSS_HDOPFILT_EN));
	STRCMP_EQUAL("FIELD", configuration_store->read_param<std::string>(ParamID::PROFILE_NAME).c_str());
	CHECK_EQUAL(25U, configuration_store->read_param<unsigned int>(ParamID::LB_THRESHOLD));

	// A parameter no batch touched still holds its factory default
	CHECK_EQUAL(0U, configuration_store->read_param<unsigned int>(ParamID::ARGOS_DECID));
}

TEST(DTEHandler, SATDP_REQ_NoSatelliteDevice) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// SATDP with no satellite device in test build - should return error
	req = "$SATDP#000;\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::SATDP_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::INCORRECT_DATA, error_code);
}

TEST(DTEHandler, GNSSI_REQ_NoGPSDevice) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Ensure gps_device is null (test build default)
	gps_device = nullptr;

	// GNSSI with no GPS device - should return INCORRECT_DATA error
	req = "$GNSSI#000;\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::GNSSI_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::INCORRECT_DATA, error_code);
}

TEST(DTEHandler, GNSSA_REQ_NoGPSDevice) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Ensure gps_device is null (test build default)
	gps_device = nullptr;

	// GNSSA with no GPS device - should return INCORRECT_DATA error
	req = "$GNSSA#000;\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::GNSSA_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::INCORRECT_DATA, error_code);
}

TEST(DTEHandler, GNSSA_REQ_NoAlmanacFile) {
	DTECommand command;
	std::string req;
	std::string resp;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code;

	// Use a minimal fake GPSDevice (base class returns default GNSSAlmanacStatus)
	class FakeGPSDevice : public GPSDevice {
	public:
		void power_on(const GPSNavSettings &) override {}
		void power_off() override {}
	};
	FakeGPSDevice fake_gps;
	gps_device = &fake_gps;

	// GNSSA with GPS device but no almanac file - base class returns all zeros
	req = "$GNSSA#000;\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	DTEDecoder::decode(resp, command, error_code, arg_list, params, param_values);
	CHECK_TRUE(DTECommand::GNSSA_RESP == command);
	CHECK_EQUAL((unsigned int)DTEError::OK, error_code);
	CHECK_EQUAL(0U, std::get<unsigned int>(arg_list[0]));  // present=0
	CHECK_EQUAL(0U, std::get<unsigned int>(arg_list[1]));  // file_size=0
	CHECK_EQUAL(0U, std::get<unsigned int>(arg_list[2]));  // total_records=0
	CHECK_EQUAL(0U, std::get<unsigned int>(arg_list[3]));  // valid_records=0
	CHECK_EQUAL(0U, std::get<unsigned int>(arg_list[4]));  // stale=0

	gps_device = nullptr;
}
