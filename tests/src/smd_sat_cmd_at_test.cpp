// SmdSatCmdAt: has the frame of the last AT+TX started radiating
// (tx_rf_started)? The module answers +OK on its MAC's KNS_MAC_OK, after the
// TCXO warm-up, when RF starts; +TX=<status> follows at the end of the frame. A
// dive before the +OK cuts the rail with nothing on air, and the host gives the
// position back.

#include "smd_sat_cmd_at.hpp"
#include "nrf_libuarte_async.h"

#include "CppUTest/TestHarness.h"

#include <string>

namespace {
// What the module sends back, handed over as the UART interrupt would.
void module_says(const std::string &bytes) {
	nrf_libuarte_async_evt_t evt;
	evt.type = NRF_LIBUARTE_ASYNC_EVT_RX_DATA;
	evt.data.rxtx.length = bytes.size();
	evt.data.rxtx.p_data = reinterpret_cast<uint8_t *>(const_cast<char *>(bytes.data()));
	nrf_libuarte_inject_event(&evt);
}
}  // namespace

TEST_GROUP(SmdSatCmdAtRfStart) {
	SmdSatCmdAt *cmd = nullptr;

	void setup() {
		cmd = new SmdSatCmdAt(0);
		cmd->init();
	}
	void teardown() {
		delete cmd;
	}

	void start_tx() {
		CHECK_TRUE(cmd->initiate_tx(KineisPacket("\x01\x02\x03\x04", 4)));
	}
};

// The rail cut inside the warm-up: no reply yet, nothing on air.
TEST(SmdSatCmdAtRfStart, NoReplyIsNotOnAir) {
	start_tx();
	CHECK_FALSE(cmd->tx_rf_started());
}

// The +OK is still in the receive buffer when the dive is noticed: asking
// parses it first.
TEST(SmdSatCmdAtRfStart, TheOkOfTheTxIsOnAir) {
	start_tx();
	module_says("+OK\r\n");
	CHECK_TRUE(cmd->tx_rf_started());
}

// A completion with status 0 proves the frame went out, even if its +OK was lost.
TEST(SmdSatCmdAtRfStart, ASuccessfulCompletionIsOnAirWithoutItsOk) {
	start_tx();
	module_says("+TX=0,01020304\r\n");
	CHECK_TRUE(cmd->tx_rf_started());
}

TEST(SmdSatCmdAtRfStart, AFailedCompletionWithoutOkIsNotOnAir) {
	start_tx();
	module_says("+TX=5,01020304\r\n");
	CHECK_FALSE(cmd->tx_rf_started());
}

// The module refused the frame (MAC busy, wrong length...): it never radiated.
TEST(SmdSatCmdAtRfStart, ARefusedTxIsNotOnAir) {
	start_tx();
	module_says("+ERROR=5\r\n");
	CHECK_FALSE(cmd->tx_rf_started());
	CHECK_TRUE(cmd->is_tx_finished());
	CHECK_FALSE(cmd->is_tx_successful());
}

// Once the TX is resolved, an +OK belongs to another command: a refused frame
// stays off the air.
TEST(SmdSatCmdAtRfStart, AnOkAfterTheTxIsResolvedIsAnotherCommands) {
	start_tx();
	module_says("+ERROR=5\r\n+OK\r\n");
	CHECK_FALSE(cmd->tx_rf_started());
}

// A late +OK of an earlier command, received but not parsed yet, is not this
// TX's: initiate_tx() parses it before arming.
TEST(SmdSatCmdAtRfStart, AStaleOkIsNotTheTxs) {
	module_says("+OK\r\n");
	start_tx();
	CHECK_FALSE(cmd->tx_rf_started());
}

TEST(SmdSatCmdAtRfStart, EachTxStartsNotOnAir) {
	start_tx();
	module_says("+OK\r\n+TX=0,01020304\r\n");
	CHECK_TRUE(cmd->tx_rf_started());
	start_tx();
	CHECK_FALSE(cmd->tx_rf_started());
}

// The verdict outlives the link: SmdSat asks once the rail is cut.
TEST(SmdSatCmdAtRfStart, TheVerdictSurvivesTheTeardown) {
	start_tx();
	module_says("+OK\r\n");
	CHECK_TRUE(cmd->tx_rf_started());
	cmd->deinit();
	CHECK_TRUE(cmd->tx_rf_started());
}

// A power-up forgets it: nothing can be in flight on a module just switched on.
// Hence SmdSat latches it before the teardown.
TEST(SmdSatCmdAtRfStart, APowerUpForgetsTheVerdict) {
	start_tx();
	module_says("+OK\r\n");
	CHECK_TRUE(cmd->tx_rf_started());
	cmd->deinit();
	cmd->init();
	CHECK_FALSE(cmd->tx_rf_started());
}
