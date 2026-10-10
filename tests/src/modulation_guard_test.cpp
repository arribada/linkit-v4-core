#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "mock_kineis_device.hpp"
#include "fake_config_store.hpp"
#include "fake_battery_mon.hpp"
#include "fake_rtc.hpp"
#include "fake_timer.hpp"
#include "scheduler.hpp"
#include "argos_tx_service.hpp"
#include "argos_config_check.hpp"
#include "modulation_guard.hpp"

#include <cstring>
#include <string>

// Argos modulation guard (adaptive modulation OFF): the pure helpers the
// nRF-only drivers rely on, the configuration-time evaluator (ART12), and the
// service-start enforcement with the mock Kineis device. The host tests are
// LoRa builds: the evaluator and the enforcement are compiled everywhere, only
// their DTE call sites are gated, so this is where they get their coverage.

extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern RTC *rtc;
extern BatteryMonitor *battery_monitor;

using namespace ModulationGuard;

static const std::string MASTER_A = "2c93600d6be3bac0ccfe9047c02c058e";
static const std::string MASTER_B = "03921fb104b92859209b18abd009de96";

// ============================================================================
// Pure helpers
// ============================================================================

TEST_GROUP(ModulationGuardPure){};

TEST(ModulationGuardPure, CodesFollowTheSmp01Order) {
	CHECK_EQUAL(CODE_LDA2, to_code(KineisModulation::LDA2));
	CHECK_EQUAL(CODE_LDK, to_code(KineisModulation::LDK));
	CHECK_EQUAL(CODE_VLDA4, to_code(KineisModulation::VLDA4));
	CHECK_EQUAL(0U, CODE_LDA2);
	CHECK_EQUAL(1U, CODE_LDK);
	CHECK_EQUAL(2U, CODE_VLDA4);
	CHECK_EQUAL(3U, CODE_UNKNOWN);
	for (unsigned int c = 0; c <= 2; c++)
		CHECK_EQUAL(c, to_code(from_code(c).value()));
	CHECK_FALSE(from_code(CODE_UNKNOWN).has_value());
	CHECK_FALSE(from_code(7).has_value());
}

TEST(ModulationGuardPure, FingerprintEmptyIsZeroAndRealOnesNeverAre) {
	CHECK_EQUAL(0U, rconf_fingerprint(""));
	const uint32_t a = rconf_fingerprint(MASTER_A);
	const uint32_t b = rconf_fingerprint(MASTER_B);
	CHECK(a != 0U);
	CHECK(b != 0U);
	CHECK_EQUAL(1U, a & 1U);
	CHECK(a != b);
	// Hex case does not make another master.
	std::string upper = MASTER_A;
	for (auto &c : upper)
		c = (char)toupper(c);
	CHECK_EQUAL(a, rconf_fingerprint(upper));
	// One character changes it.
	std::string other = MASTER_A;
	other[5] = (other[5] == '0') ? '1' : '0';
	CHECK(a != rconf_fingerprint(other));
}

TEST(ModulationGuardPure, TrustNeedsTheSameMasterAKnownCodeAndAdaptiveOff) {
	const uint32_t fa = rconf_fingerprint(MASTER_A);
	CHECK_TRUE(modulation_cache_trusted(CODE_LDK, fa, MASTER_A, false));
	CHECK_TRUE(modulation_cache_trusted(CODE_LDA2, fa, MASTER_A, false));
	CHECK_TRUE(modulation_cache_trusted(CODE_VLDA4, fa, MASTER_A, false));
	CHECK_FALSE(modulation_cache_trusted(CODE_UNKNOWN, fa, MASTER_A, false));  // never read back
	CHECK_FALSE(modulation_cache_trusted(CODE_LDK, fa, MASTER_A, true));       // adaptive on
	CHECK_FALSE(modulation_cache_trusted(CODE_LDK, fa, MASTER_B, false));      // master changed
	CHECK_FALSE(modulation_cache_trusted(CODE_LDK, 0U, MASTER_A, false));      // factory ART14
	CHECK_FALSE(modulation_cache_trusted(CODE_LDK, 0U, "", false));            // empty master
}

TEST(ModulationGuardPure, InitResyncOncePerMasterAndNeverOnLowBattery) {
	const uint32_t fa = rconf_fingerprint(MASTER_A);
	CHECK_TRUE(should_attempt_init_resync(false, MASTER_A, 0U, false));
	CHECK_TRUE(should_attempt_init_resync(false, MASTER_A, rconf_fingerprint(MASTER_B), false));
	CHECK_FALSE(should_attempt_init_resync(false, MASTER_A, fa, false));  // already attempted
	CHECK_FALSE(should_attempt_init_resync(true, MASTER_A, 0U, false));   // trusted: nothing to do
	CHECK_FALSE(should_attempt_init_resync(false, "", 0U, false));        // nothing to read back
	CHECK_FALSE(should_attempt_init_resync(false, MASTER_A, 0U, true));   // low battery
}

TEST(ModulationGuardPure, SmdSpiCodeOnlyKnownModulationsAreRecognised) {
	CHECK_EQUAL(CODE_LDA2, smd_kns_code_to_code(2).value());  // LDA2
	CHECK_EQUAL(CODE_LDA2, smd_kns_code_to_code(3).value());  // LDA2L
	CHECK_EQUAL(CODE_VLDA4, smd_kns_code_to_code(4).value());
	CHECK_EQUAL(CODE_LDK, smd_kns_code_to_code(6).value());
	// HDA4 and junk: NOT a silent LDA2 any more.
	for (unsigned int k : { 0U, 1U, 5U, 7U, 0xFFU })
		CHECK_FALSE(smd_kns_code_to_code((uint8_t)k).has_value());
}

TEST(ModulationGuardPure, SmdAtReplyOnlyKnownModulationsAreRecognised) {
	CHECK_EQUAL(CODE_LDA2, smd_at_rconf_reply_to_code("401620000,401680000,27,LDA2").value());
	CHECK_EQUAL(CODE_LDA2, smd_at_rconf_reply_to_code("401620000,401680000,27,LDA2L").value());
	CHECK_EQUAL(CODE_LDK, smd_at_rconf_reply_to_code("401620000,401680000,27,LDK").value());
	CHECK_EQUAL(CODE_VLDA4, smd_at_rconf_reply_to_code("401620000,401680000,27,VLDA4").value());
	CHECK_EQUAL(CODE_LDK, smd_at_rconf_reply_to_code("401620000,401680000,27,LDK\r\n").value());
	CHECK_FALSE(smd_at_rconf_reply_to_code("401620000,401680000,27,HDA4").has_value());
	CHECK_FALSE(smd_at_rconf_reply_to_code("401620000,401680000,27,").has_value());
	// The no-comma path used to leave the caller's value and look like a
	// genuine read-back of whatever it held.
	CHECK_FALSE(smd_at_rconf_reply_to_code("LDK").has_value());
	CHECK_FALSE(smd_at_rconf_reply_to_code("garbage").has_value());
	CHECK_FALSE(smd_at_rconf_reply_to_code("").has_value());
}

static KineisPacket bytes(unsigned int n, uint8_t fill = 0xA5) {
	return KineisPacket(n, (char)fill);
}

TEST(ModulationGuardPure, FramingPadsLikeTheDrivers) {
	// LDA2: up to the next 4 B, minimum 4, maximum 24.
	const unsigned int lda2_in[] = { 0, 3, 4, 5, 12, 13, 24 };
	const unsigned int lda2_out[] = { 4, 4, 4, 8, 12, 16, 24 };
	for (unsigned int i = 0; i < sizeof(lda2_in) / sizeof(lda2_in[0]); i++) {
		auto f = frame_for_modulation(bytes(lda2_in[i]), lda2_in[i] * 8, KineisModulation::LDA2);
		CHECK_TRUE(f.has_value());
		CHECK_EQUAL(lda2_out[i], (unsigned int)f->size());
		for (unsigned int b = 0; b < f->size(); b++)
			CHECK_EQUAL(b < lda2_in[i] ? 0xA5 : 0x00, (uint8_t)(*f)[b]);
	}
	// LDK fixed 16 B, VLDA4 fixed 3 B.
	for (unsigned int n : { 1U, 3U, 4U, 12U, 16U }) {
		auto f = frame_for_modulation(bytes(n), n * 8, KineisModulation::LDK);
		CHECK_TRUE(f.has_value());
		CHECK_EQUAL(16U, (unsigned int)f->size());
	}
	for (unsigned int n : { 1U, 3U }) {
		auto f = frame_for_modulation(bytes(n), n * 8, KineisModulation::VLDA4);
		CHECK_TRUE(f.has_value());
		CHECK_EQUAL(3U, (unsigned int)f->size());
	}
}

TEST(ModulationGuardPure, FramingRefusesInsteadOfTruncating) {
	CHECK_FALSE(frame_for_modulation(bytes(24), 192, KineisModulation::LDK).has_value());
	CHECK_FALSE(frame_for_modulation(bytes(17), 136, KineisModulation::LDK).has_value());
	CHECK_FALSE(frame_for_modulation(bytes(4), 32, KineisModulation::VLDA4).has_value());
	CHECK_FALSE(frame_for_modulation(bytes(16), 128, KineisModulation::VLDA4).has_value());
	CHECK_FALSE(frame_for_modulation(bytes(25), 200, KineisModulation::LDA2).has_value());
	CHECK_TRUE(frame_for_modulation(bytes(24), 192, KineisModulation::LDA2).has_value());
}

TEST(ModulationGuardPure, FramingZeroesTheBitsPastThePayload) {
	// KIM2 stuffs with zero BITS, not bytes: a 22-bit payload keeps 6 bits of
	// its third byte and nothing more.
	auto f = frame_for_modulation(bytes(3, 0xFF), 22, KineisModulation::VLDA4);
	CHECK_TRUE(f.has_value());
	CHECK_EQUAL(0xFF, (uint8_t)(*f)[0]);
	CHECK_EQUAL(0xFF, (uint8_t)(*f)[1]);
	CHECK_EQUAL(0xFC, (uint8_t)(*f)[2]);
	// The legacy Doppler 006400 re-padded onto LDK stays 006400 + 13 zero bytes.
	KineisPacket legacy("\x00\x64\x00", 3);
	auto l = frame_for_modulation(legacy, 24, KineisModulation::LDK);
	CHECK_TRUE(l.has_value());
	KineisPacket expect(16, '\0');
	expect[1] = 0x64;
	CHECK_TRUE(expect == l.value());
}

TEST(ModulationGuardPure, SizeTableIsTheServiceOne) {
	CHECK_EQUAL(128U, max_payload_bits(KineisModulation::LDK));
	CHECK_EQUAL(192U, max_payload_bits(KineisModulation::LDA2));
	CHECK_EQUAL(24U, max_payload_bits(KineisModulation::VLDA4));
}

// ============================================================================
// ConfigurationStore helpers (the single SMP01 writer)
// ============================================================================

TEST_GROUP(ModulationGuardStore) {
	FakeConfigurationStore *store;
	void setup() {
		store = new FakeConfigurationStore;
		store->init();
		configuration_store = store;
	}
	void teardown() {
		delete store;
		configuration_store = nullptr;
	}
	unsigned int u(ParamID p) {
		return store->read_param<unsigned int>(p);
	}
};

TEST(ModulationGuardStore, FactoryStateIsUntrusted) {
	CHECK_EQUAL(0U, u(ParamID::ARGOS_CFG_WARNINGS));
	CHECK_EQUAL(CODE_UNKNOWN, u(ParamID::ARGOS_MASTER_MODULATION));
	CHECK_EQUAL(0U, u(ParamID::ARGOS_MASTER_RCONF_FP));
	CHECK_EQUAL(0U, u(ParamID::ARGOS_MASTER_SYNC_ATTEMPT_FP));
	CHECK_EQUAL(CODE_UNKNOWN, store->trusted_master_modulation());
}

TEST(ModulationGuardStore, CachedWriteTouchesSmp01Only) {
	store->write_param(ParamID::ARGOS_RADIOCONF, MASTER_A);
	store->write_cached_modulation(CODE_VLDA4);
	CHECK_EQUAL(CODE_VLDA4, u(ParamID::ARGOS_CACHED_MODULATION));
	CHECK_EQUAL(CODE_UNKNOWN, u(ParamID::ARGOS_MASTER_MODULATION));
	CHECK_EQUAL(0U, u(ParamID::ARGOS_MASTER_RCONF_FP));
	store->write_cached_modulation(CODE_UNKNOWN);  // ignored
	store->write_cached_modulation(9);             // ignored
	CHECK_EQUAL(CODE_VLDA4, u(ParamID::ARGOS_CACHED_MODULATION));
}

TEST(ModulationGuardStore, GenuineReadbackRecordsTheMasterAndItsFingerprint) {
	store->write_param(ParamID::ARGOS_RADIOCONF, MASTER_A);
	store->write_master_modulation_readback(CODE_LDK);
	CHECK_EQUAL(CODE_LDK, u(ParamID::ARGOS_CACHED_MODULATION));
	CHECK_EQUAL(CODE_LDK, u(ParamID::ARGOS_MASTER_MODULATION));
	CHECK_EQUAL(rconf_fingerprint(MASTER_A), u(ParamID::ARGOS_MASTER_RCONF_FP));
	CHECK_EQUAL(CODE_LDK, store->trusted_master_modulation());

	// An escalation afterwards moves SMP01 only; the master stays LDK.
	store->write_cached_modulation(CODE_LDA2);
	CHECK_EQUAL(CODE_LDK, store->trusted_master_modulation());

	// Adaptive on, or another master: no longer trusted.
	store->write_param(ParamID::ARGOS_ADAPTIVE_MODULATION, true);
	CHECK_EQUAL(CODE_UNKNOWN, store->trusted_master_modulation());
	store->write_param(ParamID::ARGOS_ADAPTIVE_MODULATION, false);
	store->write_param(ParamID::ARGOS_RADIOCONF, MASTER_B);
	CHECK_EQUAL(CODE_UNKNOWN, store->trusted_master_modulation());
	store->write_param(ParamID::ARGOS_RADIOCONF, MASTER_A);
	CHECK_EQUAL(CODE_LDK, store->trusted_master_modulation());
	store->invalidate_master_modulation();
	CHECK_EQUAL(CODE_UNKNOWN, u(ParamID::ARGOS_MASTER_MODULATION));
	CHECK_EQUAL(CODE_UNKNOWN, store->trusted_master_modulation());
}

// ============================================================================
// Configuration-time warnings (ART12)
// ============================================================================

TEST_GROUP(ArgosConfigCheck){};

using namespace ArgosConfigCheck;

static Inputs cl(unsigned int format, unsigned int master, bool adaptive = false, bool is_smd = false) {
	Inputs in{};
	in.gnss_en = true;
	in.fastloc_mode = (unsigned int)BaseFastlocMode::CLOUDLOCATE;
	in.cl_format = format;
	in.adaptive = adaptive;
	in.master_empty = false;
	in.master_mod = master;
	in.is_smd = is_smd;
	return in;
}

static constexpr unsigned int MEASC12 = (unsigned int)BaseCloudLocateFormat::MEASC12;
static constexpr unsigned int MEAS20 = (unsigned int)BaseCloudLocateFormat::MEAS20;
static constexpr unsigned int MEAS50 = (unsigned int)BaseCloudLocateFormat::MEAS50;

TEST(ArgosConfigCheck, Meas20OnAnLdkMasterIsBit0WithTheMainText) {
	Inputs in = cl(MEAS20, CODE_LDK);
	CHECK_EQUAL(CL_EXCEEDS_MASTER, evaluate(in));
	STRCMP_EQUAL("ArgosConfigCheck: CloudLocate MEAS20 needs an LDA2 master RCONF, master reads back LDK (adaptive "
	             "off) - never sent",
	             message_for(evaluate(in), in));
	in = cl(MEAS20, CODE_VLDA4);
	CHECK_EQUAL(CL_EXCEEDS_MASTER, evaluate(in));
	CHECK(strstr(message_for(evaluate(in), in), "reads back VLDA4") != nullptr);
}

TEST(ArgosConfigCheck, Meas20OnAnLda2MasterIsClean) {
	CHECK_EQUAL(0U, evaluate(cl(MEAS20, CODE_LDA2)));
	CHECK_EQUAL(0U, evaluate(cl(MEAS20, CODE_LDA2, false, true)));
}

TEST(ArgosConfigCheck, UnverifiedMasterIsBit1Only) {
	Inputs in = cl(MEAS20, CODE_UNKNOWN);
	CHECK_EQUAL(CL_MASTER_UNVERIFIED, evaluate(in));
	STRCMP_EQUAL("ArgosConfigCheck: CloudLocate MEAS20 needs an LDA2 master RCONF; master modulation not verified yet "
	             "- run SATVF",
	             message_for(evaluate(in), in));
}

TEST(ArgosConfigCheck, Measc12FitsLdkButNotVlda4) {
	CHECK_EQUAL(0U, evaluate(cl(MEASC12, CODE_LDK)));
	CHECK_EQUAL(0U, evaluate(cl(MEASC12, CODE_LDA2)));
	Inputs in = cl(MEASC12, CODE_VLDA4);
	CHECK_EQUAL(CL_EXCEEDS_MASTER, evaluate(in));
	CHECK(strstr(message_for(evaluate(in), in), "MEASC12") != nullptr);
}

TEST(ArgosConfigCheck, AdaptiveOnRaisesNoCloudLocateBit) {
	for (unsigned int m = 0; m <= CODE_UNKNOWN; m++)
		for (unsigned int f : { MEASC12, MEAS20 })
			CHECK_EQUAL(0U, evaluate(cl(f, m, true)));
}

TEST(ArgosConfigCheck, NoCloudLocateNoCloudLocateBit) {
	for (unsigned int fl = 0; fl <= 1; fl++) {
		Inputs in = cl(MEAS20, CODE_LDK);
		in.fastloc_mode = fl;
		CHECK_EQUAL(0U, evaluate(in));
	}
	Inputs in = cl(MEAS20, CODE_LDK);
	in.gnss_en = false;
	CHECK_EQUAL(0U, evaluate(in));
	in = cl(MEAS50, CODE_LDK);
	in.gnss_en = false;
	CHECK_EQUAL(0U, evaluate(in));
}

TEST(ArgosConfigCheck, Meas50IsBit3AloneWhateverTheMaster) {
	for (unsigned int m = 0; m <= CODE_UNKNOWN; m++) {
		for (int adaptive = 0; adaptive < 2; adaptive++) {
			Inputs in = cl(MEAS50, m, adaptive);
			CHECK_EQUAL(CL_FORMAT_NOT_ARGOS, evaluate(in));  // exact: never bit0 nor bit1
			STRCMP_EQUAL(MSG_MEAS50, message_for(evaluate(in), in));
		}
	}
}

TEST(ArgosConfigCheck, EmptyMasterOnKim2OnlyWithAdaptiveOffOnSmdAlways) {
	Inputs in{};
	in.master_empty = true;
	in.master_mod = CODE_UNKNOWN;
	in.adaptive = false;
	in.is_smd = false;
	CHECK_EQUAL(MASTER_RCONF_EMPTY, evaluate(in));
	STRCMP_EQUAL(MSG_EMPTY_KIM2, message_for(evaluate(in), in));
	in.adaptive = true;  // KIM2 adaptive uses the per-modulation RCONFs
	CHECK_EQUAL(0U, evaluate(in));
	in.is_smd = true;  // SMD will not transmit at all, adaptive or not
	CHECK_EQUAL(MASTER_RCONF_EMPTY, evaluate(in));
	STRCMP_EQUAL(MSG_EMPTY_SMD, message_for(evaluate(in), in));
	in.adaptive = false;
	CHECK_EQUAL(MASTER_RCONF_EMPTY, evaluate(in));
}

TEST(ArgosConfigCheck, EmptyMasterWinsTheLogLine) {
	Inputs in = cl(MEAS20, CODE_UNKNOWN);
	in.master_empty = true;
	CHECK_EQUAL(MASTER_RCONF_EMPTY | CL_MASTER_UNVERIFIED, evaluate(in));
	STRCMP_EQUAL(MSG_EMPTY_KIM2, message_for(evaluate(in), in));
	CHECK(message_for(0, in) == nullptr);
}

TEST(ArgosConfigCheck, TruthTableAgainstAReferenceWrittenTheOtherWay) {
	// Every combination of the seven inputs, against the rule as the spec
	// states it, written as a different expression.
	for (int gnss = 0; gnss < 2; gnss++)
		for (unsigned int fl = 0; fl <= 2; fl++)
			for (unsigned int f = 0; f <= 2; f++)
				for (int ad = 0; ad < 2; ad++)
					for (int empty = 0; empty < 2; empty++)
						for (unsigned int m = 0; m <= 3; m++)
							for (int smd = 0; smd < 2; smd++) {
								Inputs in{ (bool)gnss, fl, f, (bool)ad, (bool)empty, m, (bool)smd };
								const bool cl_on = gnss && fl == 2;
								uint32_t ref = 0;
								if (cl_on && f == 2) ref |= 8;
								if (cl_on && f != 2 && !ad && m == 3) ref |= 2;
								const unsigned int need = (f == 0) ? 128 : 192;
								const unsigned int cap = (m == 0) ? 192 : (m == 1) ? 128 : (m == 2) ? 24 : 0;
								if (cl_on && f != 2 && !ad && m != 3 && need > cap) ref |= 1;
								if (empty && (!ad || smd)) ref |= 4;
								CHECK_EQUAL(ref, evaluate(in));
								const char *msg = message_for(ref, in);
								CHECK_EQUAL(ref != 0, msg != nullptr);
								if (msg) CHECK(strlen(msg) <= MAX_LOG_PAYLOAD - 1);
							}
}

TEST_GROUP(ArgosConfigCheckRefresh) {
	FakeConfigurationStore *store;
	void setup() {
		store = new FakeConfigurationStore;
		store->init();
		configuration_store = store;
	}
	void teardown() {
		delete store;
		configuration_store = nullptr;
	}
};

TEST(ArgosConfigCheckRefresh, LogsOncePerChangeAndNothingOnTheWayBackToZero) {
	store->write_param(ParamID::GNSS_EN, true);
	store->write_param(ParamID::GNSS_FASTLOC_MODE, 2U);
	store->write_param(ParamID::GNSS_CLOUDLOCATE_FORMAT, 1U);
	store->write_param(ParamID::ARGOS_ADAPTIVE_MODULATION, false);
	store->write_param(ParamID::ARGOS_RADIOCONF, MASTER_A);

	Refresh r = refresh(*store, false);  // unverified
	CHECK_EQUAL(CL_MASTER_UNVERIFIED, r.mask);
	CHECK_TRUE(r.changed);
	CHECK_TRUE(r.logged);
	CHECK_EQUAL(CL_MASTER_UNVERIFIED, store->read_param<unsigned int>(ParamID::ARGOS_CFG_WARNINGS));

	r = refresh(*store, false);  // same mask: silent
	CHECK_FALSE(r.changed);
	CHECK_FALSE(r.logged);

	store->write_master_modulation_readback(CODE_LDK);  // master read back: LDK
	r = refresh(*store, false);
	CHECK_EQUAL(CL_EXCEEDS_MASTER, r.mask);
	CHECK_TRUE(r.logged);

	// The escalation trap: SMP01 bent to LDA2 does not hide the LDK master.
	store->write_cached_modulation(CODE_LDA2);
	r = refresh(*store, false);
	CHECK_EQUAL(CL_EXCEEDS_MASTER, r.mask);
	CHECK_FALSE(r.logged);

	store->write_param(ParamID::GNSS_CLOUDLOCATE_FORMAT, 0U);  // MEASC12 fits LDK
	r = refresh(*store, false);
	CHECK_EQUAL(0U, r.mask);
	CHECK_TRUE(r.changed);
	CHECK_FALSE(r.logged);
	CHECK_EQUAL(0U, store->read_param<unsigned int>(ParamID::ARGOS_CFG_WARNINGS));
}

// ============================================================================
// Service start: enforce_modulation_state() with the mock device
// ============================================================================

TEST_GROUP(ModulationGuardService) {
	FakeConfigurationStore *store;
	MockKineisDevice *dev;
	FakeBatteryMonitor *batt;
	void setup() {
		store = new FakeConfigurationStore;
		store->init();
		configuration_store = store;
		dev = new MockKineisDevice;
		batt = new FakeBatteryMonitor;
		battery_monitor = batt;
		store->write_param(ParamID::ARGOS_ADAPTIVE_MODULATION, false);
		store->write_param(ParamID::ARGOS_RADIOCONF, MASTER_A);
	}
	void teardown() {
		delete batt;
		delete dev;
		delete store;
		battery_monitor = nullptr;
		configuration_store = nullptr;
	}
	unsigned int u(ParamID p) {
		return store->read_param<unsigned int>(p);
	}
	/// Play the driver: the module reads back @p m from the master.
	void module_reads_back(KineisModulation m) {
		dev->test_on_resync = [this, m]() {
			dev->test_set_current_modulation(m);
			store->write_master_modulation_readback(to_code(m));
			return true;
		};
	}
	ArgosTxService::ModulationStateOutcome run(bool is_smd, bool battery_low = false) {
		return ArgosTxService::enforce_modulation_state(*dev, *store, is_smd, battery_low);
	}
};

TEST(ModulationGuardService, AdaptiveOnDoesNothing) {
	store->write_param(ParamID::ARGOS_ADAPTIVE_MODULATION, true);
	for (int smd = 0; smd < 2; smd++) {
		auto o = run((bool)smd);
		CHECK_TRUE(o.adaptive);
		CHECK_EQUAL(0U, dev->resync_calls);
		CHECK_EQUAL(0U, dev->restore_calls);
		CHECK_FALSE(o.changed);
	}
}

TEST(ModulationGuardService, Kim2KeepsItsUnconditionalResync) {
	module_reads_back(KineisModulation::LDK);
	auto o = run(false);
	CHECK_EQUAL(1U, dev->resync_calls);
	CHECK_TRUE(o.resync_ok);
	CHECK_TRUE(o.trusted);
	CHECK_FALSE(o.restored);  // the driver already holds the truth
	CHECK_TRUE(o.changed);    // ART13/ART14 learnt -> the caller saves once
	CHECK_EQUAL(CODE_LDK, u(ParamID::ARGOS_MASTER_MODULATION));
	CHECK_EQUAL((int)KineisModulation::LDK, (int)dev->get_current_modulation());

	// Second start, same config: resync again (KIM2 policy), nothing changed.
	o = run(false);
	CHECK_EQUAL(2U, dev->resync_calls);
	CHECK_FALSE(o.changed);
}

TEST(ModulationGuardService, TrustedRestoresAStickyEscalationWithoutRadioOnSmd) {
	store->write_master_modulation_readback(CODE_LDK);  // genuine LDK earlier
	store->write_cached_modulation(CODE_LDA2);          // then a sticky escalation
	dev->test_set_current_modulation(KineisModulation::LDA2);
	auto o = run(true);
	CHECK_EQUAL(0U, dev->resync_calls);  // trusted: no SPI, no power-on
	CHECK_TRUE(o.trusted);
	CHECK_TRUE(o.restored);
	CHECK_TRUE(o.changed);
	CHECK_EQUAL(1U, dev->restore_calls);
	CHECK_EQUAL((int)KineisModulation::LDK, (int)dev->get_current_modulation());
	CHECK_EQUAL(CODE_LDK, u(ParamID::ARGOS_CACHED_MODULATION));
	CHECK_EQUAL(0U, store->get_saved_count());  // the caller saves, once

	// Next start: already right, nothing to do.
	o = run(true);
	CHECK_FALSE(o.restored);
	CHECK_FALSE(o.changed);
	CHECK_EQUAL(1U, dev->restore_calls);
}

TEST(ModulationGuardService, Kim2ResyncFailureFallsBackToTheTrustedRestore) {
	store->write_master_modulation_readback(CODE_LDK);
	dev->test_set_current_modulation(KineisModulation::LDA2);
	dev->test_resync_throws = true;
	ArgosTxService::ModulationStateOutcome o;
	try {
		o = run(false);
	} catch (...) {
		FAIL("enforce_modulation_state must not throw");
	}
	CHECK_EQUAL(1U, dev->resync_calls);
	CHECK_TRUE(o.restored);
	CHECK_EQUAL((int)KineisModulation::LDK, (int)dev->get_current_modulation());
}

TEST(ModulationGuardService, SmdUntrustedAttemptsOncePerMasterAndPersistsTheAttemptFirst) {
	unsigned int saves_seen_by_module = 0;
	unsigned int attempt_fp_seen_by_module = 0;
	dev->test_on_resync = [&]() {
		// What the module sees when it is powered: the attempt is ALREADY on flash.
		saves_seen_by_module = store->get_saved_count();
		attempt_fp_seen_by_module = u(ParamID::ARGOS_MASTER_SYNC_ATTEMPT_FP);
		return false;  // e.g. the module did not answer
	};
	auto o = run(true);
	CHECK_TRUE(o.attempted);
	CHECK_FALSE(o.resync_ok);
	CHECK_FALSE(o.trusted);
	CHECK_EQUAL(1U, dev->resync_calls);
	CHECK_EQUAL(1U, saves_seen_by_module);
	CHECK_EQUAL(rconf_fingerprint(MASTER_A), attempt_fp_seen_by_module);

	// Every later boot with the same master: no second attempt, whatever the outcome.
	for (int boot = 0; boot < 3; boot++) {
		o = run(true);
		CHECK_FALSE(o.attempted);
	}
	CHECK_EQUAL(1U, dev->resync_calls);

	// A new master earns one attempt of its own.
	store->write_param(ParamID::ARGOS_RADIOCONF, MASTER_B);
	module_reads_back(KineisModulation::LDA2);
	o = run(true);
	CHECK_TRUE(o.attempted);
	CHECK_TRUE(o.resync_ok);
	CHECK_TRUE(o.trusted);
	CHECK_TRUE(o.changed);
	CHECK_EQUAL(2U, dev->resync_calls);
	CHECK_EQUAL(CODE_LDA2, u(ParamID::ARGOS_MASTER_MODULATION));
	CHECK_EQUAL(rconf_fingerprint(MASTER_B), u(ParamID::ARGOS_MASTER_RCONF_FP));
	o = run(true);
	CHECK_FALSE(o.attempted);
	CHECK_EQUAL(2U, dev->resync_calls);
}

TEST(ModulationGuardService, SmdResyncThrowIsSwallowedAndStillCountsAsTheAttempt) {
	dev->test_resync_throws = true;
	ArgosTxService::ModulationStateOutcome o;
	try {
		o = run(true);
	} catch (...) {
		FAIL("enforce_modulation_state must not throw");
	}
	CHECK_TRUE(o.attempted);
	CHECK_EQUAL(rconf_fingerprint(MASTER_A), u(ParamID::ARGOS_MASTER_SYNC_ATTEMPT_FP));
	o = run(true);
	CHECK_FALSE(o.attempted);
	CHECK_EQUAL(1U, dev->resync_calls);
}

TEST(ModulationGuardService, SmdNeverAttemptsOnLowBatteryOrWithoutAMaster) {
	auto o = run(true, true);
	CHECK_FALSE(o.attempted);
	CHECK_EQUAL(0U, u(ParamID::ARGOS_MASTER_SYNC_ATTEMPT_FP));  // still free for a later boot
	store->write_param(ParamID::ARGOS_RADIOCONF, std::string(""));
	o = run(true);
	CHECK_FALSE(o.attempted);
	CHECK_EQUAL(0U, dev->resync_calls);
}

// ============================================================================
// Through Service::start(): service_init wires it, and never throws
// ============================================================================

TEST_GROUP(ModulationGuardServiceInit) {
	FakeBatteryMonitor *batt;
	FakeConfigurationStore *store;
	MockKineisDevice *dev;
	FakeRTC *fake_rtc;
	FakeTimer *fake_timer;
	void setup() {
		ServiceManager::reset();
		batt = new FakeBatteryMonitor;
		battery_monitor = batt;
		dev = new MockKineisDevice;
		store = new FakeConfigurationStore;
		configuration_store = store;
		store->init();
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		fake_rtc->settime(1580083200);
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		system_scheduler = new Scheduler(system_timer);
		fake_timer->start();
		store->write_param(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S, 0U);
		mock().ignoreOtherCalls();
	}
	void teardown() {
		mock().clear();
		delete system_scheduler;
		delete fake_timer;
		delete fake_rtc;
		delete store;
		delete dev;
		delete batt;
		system_scheduler = nullptr;
		system_timer = nullptr;
		rtc = nullptr;
		configuration_store = nullptr;
		battery_monitor = nullptr;
		ServiceManager::reset();
	}
};

TEST(ModulationGuardServiceInit, StartRestoresTheTrustedMasterAndRefreshesTheWarning) {
	store->write_param(ParamID::ARGOS_RADIOCONF, MASTER_A);
	store->write_param(ParamID::GNSS_FASTLOC_MODE, 2U);
	store->write_param(ParamID::GNSS_CLOUDLOCATE_FORMAT, 1U);
	store->write_master_modulation_readback(CODE_LDK);
	dev->test_set_current_modulation(KineisModulation::LDA2);  // stale / escalated
	{
		const unsigned int saves = store->get_saved_count();
		ArgosTxService serv(*dev);
		serv.start();
		// Host build = KIM2 flavour: the unconditional resync ran (mock: no
		// answer), and the trusted LDK master was restored without it.
		CHECK_EQUAL(1U, dev->resync_calls);
		CHECK_EQUAL((int)KineisModulation::LDK, (int)dev->get_current_modulation());
		CHECK_EQUAL(CODE_LDK, store->read_param<unsigned int>(ParamID::ARGOS_CACHED_MODULATION));
		CHECK_EQUAL(ArgosConfigCheck::CL_EXCEEDS_MASTER, store->read_param<unsigned int>(ParamID::ARGOS_CFG_WARNINGS));
		CHECK_EQUAL(saves + 1, store->get_saved_count());  // one save for all of it
		serv.stop();
	}
	{
		// Restart with nothing new: no save at all.
		const unsigned int saves = store->get_saved_count();
		ArgosTxService serv(*dev);
		serv.start();
		CHECK_EQUAL(2U, dev->resync_calls);
		CHECK_EQUAL(saves, store->get_saved_count());
		serv.stop();
	}
}

TEST(ModulationGuardServiceInit, AResyncThatThrowsNeverEscapesServiceStart) {
	dev->test_resync_throws = true;
	ArgosTxService serv(*dev);
	try {
		serv.start();
	} catch (...) {
		FAIL("service start must not throw");
	}
	CHECK_EQUAL(1U, dev->resync_calls);
	serv.stop();
}
