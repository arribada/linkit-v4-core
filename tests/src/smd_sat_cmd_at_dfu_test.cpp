// SmdSatCmdAt DFU against the SMD bootloader's UART protocol: "AT+DFU=<NAME>"
// commands (PING, INFO, ERASE, WRITE,<addr>,<hex>, VERIFY,<crc>, RESET, JUMP),
// "+DFU=OK[,<data>]" or "+DFU=ERR,<NAME>" replies (bl_main.c and bl_uart.c of
// argos-smd-at-kineis-firmware). The layer used to send numeric ids, which the
// bootloader does not know: no UART update ever got past its first PING.

#include "smd_sat_cmd_at.hpp"
#include "nrf_libuarte_async.h"

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include <cstring>
#include <string>
#include <vector>

namespace {
// What the simulated module answers. Each test changes what it needs.
std::string g_ping_reply;
// How many application pings go unanswered first (the application still booting).
int g_ping_silent = 0;
std::string g_boot_reply;
std::string g_info_reply;
std::string g_erase_reply;
std::string g_write_reply;
std::string g_verify_reply;

std::string reply_to(const std::string &cmd) {
	if (cmd == "AT+PING=?") {
		if (g_ping_silent > 0) {
			g_ping_silent--;
			return "";
		}
		return g_ping_reply;
	}
	if (cmd == "AT+BOOT=") return g_boot_reply;
	if (cmd == "AT+DFU=PING") return "+DFU=OK,BL_V1.0.0\r\n";
	if (cmd == "AT+DFU=INFO") return g_info_reply;
	if (cmd == "AT+DFU=ERASE") return g_erase_reply;
	if (cmd.rfind("AT+DFU=WRITE,", 0) == 0) return g_write_reply;
	if (cmd.rfind("AT+DFU=VERIFY,", 0) == 0) return g_verify_reply;
	if (cmd == "AT+DFU=RESET") return "+DFU=OK\r\n";
	if (cmd == "AT+DFU=JUMP") return "+DFU=ERR,NOT_READY\r\n";
	return "";
}

size_t module(const uint8_t *sent, size_t sent_len, uint8_t *reply, size_t reply_max) {
	std::string cmd(reinterpret_cast<const char *>(sent), sent_len);
	if (cmd.size() >= 2 && cmd.compare(cmd.size() - 2, 2, "\r\n") == 0) cmd.resize(cmd.size() - 2);
	const std::string r = reply_to(cmd);
	const size_t n = (r.size() < reply_max) ? r.size() : reply_max;
	memcpy(reply, r.data(), n);
	return n;
}

// Every line the driver sent, in order.
std::vector<std::string> sent_lines() {
	std::string all(reinterpret_cast<const char *>(nrf_libuarte_tx_log_data()), nrf_libuarte_tx_log_size());
	std::vector<std::string> lines;
	size_t start = 0;
	for (size_t end; (end = all.find("\r\n", start)) != std::string::npos; start = end + 2) {
		lines.push_back(all.substr(start, end - start));
	}
	return lines;
}

bool was_sent(const std::string &line) {
	for (const auto &l : sent_lines()) {
		if (l == line) return true;
	}
	return false;
}
}  // namespace

TEST_GROUP(SmdSatCmdAtDfu) {
	SmdSatCmdAt *cmd = nullptr;

	void setup() {
		g_ping_reply = "+OK\r\n";
		g_ping_silent = 0;
		g_boot_reply = "+BOOT=OK\r\n";
		// v1.0.0, app at 0x08000000, 204800 bytes max, 2048-byte pages
		g_info_reply = "+DFU=OK,010000000000080020030000080000\r\n";
		g_erase_reply = "+DFU=OK\r\n";
		g_write_reply = "+DFU=OK\r\n";
		g_verify_reply = "+DFU=OK,VALIDATED\r\n";
		// The waits poll PMU::delay_ms, a mock: let them through.
		mock().ignoreOtherCalls();
		nrf_libuarte_set_responder(module);
		cmd = new SmdSatCmdAt(0);
		cmd->init();
		nrf_libuarte_tx_log_clear();
	}
	void teardown() {
		delete cmd;
		nrf_libuarte_set_responder(nullptr);
		mock().clear();
	}
};

TEST(SmdSatCmdAtDfu, TheBootloaderIsAskedByName) {
	CHECK_TRUE(cmd->dfu_enter());
	const auto lines = sent_lines();
	CHECK_TRUE(lines.size() >= 3);
	STRCMP_EQUAL("AT+PING=?", lines[0].c_str());
	STRCMP_EQUAL("AT+BOOT=", lines[1].c_str());
	STRCMP_EQUAL("AT+DFU=PING", lines[2].c_str());
	CHECK_TRUE(cmd->is_dfu_mode());
}

// SmdSat powers the module up just before dfu_enter(): an AT+BOOT sent while
// the application boots was lost (bench 2026-10-30). It waits for the
// application to answer first.
TEST(SmdSatCmdAtDfu, AnApplicationStillBootingIsWaitedFor) {
	g_ping_silent = 3;
	CHECK_TRUE(cmd->dfu_enter());
	CHECK_TRUE(was_sent("AT+BOOT="));
	CHECK_TRUE(cmd->is_dfu_mode());
}

// An update that failed after the erase leaves the module in its bootloader,
// which knows neither AT+PING nor AT+BOOT: sending the image again must still
// reach it.
TEST(SmdSatCmdAtDfu, AModuleAlreadyInItsBootloaderIsReached) {
	g_ping_reply = "+DFU=ERR,INVALID_CMD\r\n";
	CHECK_TRUE(cmd->dfu_enter());
	CHECK_FALSE(was_sent("AT+BOOT="));
	CHECK_TRUE(was_sent("AT+DFU=PING"));
	CHECK_TRUE(cmd->is_dfu_mode());
}

TEST(SmdSatCmdAtDfu, TheBootloaderInfoIsReadFromHex) {
	CHECK_TRUE(cmd->dfu_enter());
	SmdDfuInfo info;
	CHECK_TRUE(cmd->dfu_get_bootloader_info(&info));
	CHECK_TRUE(was_sent("AT+DFU=INFO"));
	UNSIGNED_LONGS_EQUAL(1, info.version_major);
	UNSIGNED_LONGS_EQUAL(0, info.version_minor);
	UNSIGNED_LONGS_EQUAL(0x08000000, info.app_start_addr);
	UNSIGNED_LONGS_EQUAL(204800, info.app_max_size);
	UNSIGNED_LONGS_EQUAL(2048, info.flash_page_size);
}

// Too short to hold the layout: the image must not go to whatever address an
// uninitialised struct holds.
TEST(SmdSatCmdAtDfu, AShortInfoIsRefused) {
	g_info_reply = "+DFU=OK,010000\r\n";
	CHECK_TRUE(cmd->dfu_enter());
	SmdDfuInfo info;
	CHECK_FALSE(cmd->dfu_get_bootloader_info(&info));
}

TEST(SmdSatCmdAtDfu, AChunkGoesWithItsAddressInUppercaseHex) {
	CHECK_TRUE(cmd->dfu_enter());
	const uint8_t data[8] = { 0xab, 0x01, 0x00, 0xff, 0x10, 0x2f, 0x9c, 0xe0 };
	LONGS_EQUAL(DFU_RSP_OK, cmd->dfu_write_chunk(0x08000F00, data, sizeof(data)));
	CHECK_TRUE(was_sent("AT+DFU=WRITE,08000F00,AB0100FF102F9CE0"));
}

// The bootloader shipped before 2026-10-30 keeps 244 data bytes of a WRITE line
// and drops the rest without an error.
TEST(SmdSatCmdAtDfu, ChunksStayUnderTheOldBootloaderLimit) {
	CHECK_TRUE(cmd->dfu_chunk_size() <= 244);
	LONGS_EQUAL(0, cmd->dfu_chunk_size() % 8);

	CHECK_TRUE(cmd->dfu_enter());
	nrf_libuarte_tx_log_clear();
	uint8_t data[SMDSAT_DFU_CHUNK_SIZE] = { 0 };
	LONGS_EQUAL(DFU_RSP_SIZE_ERROR, cmd->dfu_write_chunk(0x08000000, data, cmd->dfu_chunk_size() + 1));
	UNSIGNED_LONGS_EQUAL(0, nrf_libuarte_tx_log_size());
}

TEST(SmdSatCmdAtDfu, AnUnalignedAddressIsNotSent) {
	CHECK_TRUE(cmd->dfu_enter());
	nrf_libuarte_tx_log_clear();
	const uint8_t data[8] = { 0 };
	LONGS_EQUAL(DFU_RSP_ADDR_ERROR, cmd->dfu_write_chunk(0x08000004, data, sizeof(data)));
	UNSIGNED_LONGS_EQUAL(0, nrf_libuarte_tx_log_size());
}

// VALIDATED: the bootloader has already left for the application, which does
// not know AT+DFU=JUMP.
TEST(SmdSatCmdAtDfu, AVerifiedImageNeedsNoJump) {
	CHECK_TRUE(cmd->dfu_enter());
	LONGS_EQUAL(DFU_RSP_OK, cmd->dfu_erase());
	LONGS_EQUAL(DFU_RSP_OK, cmd->dfu_verify(0x89E50CA3));
	CHECK_TRUE(was_sent("AT+DFU=VERIFY,89E50CA3"));
	LONGS_EQUAL(DFU_RSP_OK, cmd->dfu_jump());
	CHECK_FALSE(was_sent("AT+DFU=JUMP"));
	CHECK_FALSE(cmd->is_dfu_mode());
}

TEST(SmdSatCmdAtDfu, TheBootloaderErrorKeepsItsName) {
	g_verify_reply = "+DFU=ERR,CRC_ERROR\r\n";
	g_write_reply = "+DFU=ERR,ADDR_ERROR\r\n";
	CHECK_TRUE(cmd->dfu_enter());
	const uint8_t data[8] = { 0 };
	LONGS_EQUAL(DFU_RSP_ADDR_ERROR, cmd->dfu_write_chunk(0x08000000, data, sizeof(data)));
	LONGS_EQUAL(DFU_RSP_CRC_ERROR, cmd->dfu_verify(0x12345678));
}

TEST(SmdSatCmdAtDfu, ASilentBootloaderIsATimeout) {
	g_write_reply = "";
	CHECK_TRUE(cmd->dfu_enter());
	const uint8_t data[8] = { 0 };
	LONGS_EQUAL(DFU_RSP_TIMEOUT, cmd->dfu_write_chunk(0x08000000, data, sizeof(data)));
}

// After an erase without a verified image, a restart boots the erased vector
// table on a bootloader without the page-0 trampoline: nothing is sent, the
// module stays in its bootloader for the image to be sent again.
TEST(SmdSatCmdAtDfu, NothingRestartsTheModuleAfterAnErase) {
	g_verify_reply = "+DFU=ERR,CRC_ERROR\r\n";
	CHECK_TRUE(cmd->dfu_enter());
	LONGS_EQUAL(DFU_RSP_OK, cmd->dfu_erase());
	CHECK_TRUE(cmd->dfu_verify(0x12345678) != DFU_RSP_OK);
	nrf_libuarte_tx_log_clear();
	CHECK_FALSE(cmd->dfu_exit());
	UNSIGNED_LONGS_EQUAL(0, nrf_libuarte_tx_log_size());
	CHECK_TRUE(cmd->is_dfu_mode());
}

// The next attempt is a new power-up of the layer: the erase is not forgotten.
TEST(SmdSatCmdAtDfu, TheEraseOutlivesAReinit) {
	CHECK_TRUE(cmd->dfu_enter());
	LONGS_EQUAL(DFU_RSP_OK, cmd->dfu_erase());
	cmd->init();
	CHECK_TRUE(cmd->dfu_enter());
	nrf_libuarte_tx_log_clear();
	CHECK_FALSE(cmd->dfu_exit());
	UNSIGNED_LONGS_EQUAL(0, nrf_libuarte_tx_log_size());
}

// Nothing written: JUMP is refused before a VERIFY, RESET brings the intact
// application back.
TEST(SmdSatCmdAtDfu, LeavingWithoutAnUpdateResets) {
	CHECK_TRUE(cmd->dfu_enter());
	CHECK_TRUE(cmd->dfu_exit());
	CHECK_TRUE(was_sent("AT+DFU=RESET"));
	CHECK_FALSE(was_sent("AT+DFU=JUMP"));
	CHECK_FALSE(cmd->is_dfu_mode());
}
