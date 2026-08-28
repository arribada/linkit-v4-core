#include "CppUTest/TestHarness.h"

#include "kim2/kim2_modulation.hpp"

// Pure host-side coverage for the KIM2 read-back modulation logic that backs
// the +ERROR=5 latch fix. The KIM2Device driver (kim2.cpp) is nRF-port-only and
// excluded from this build, so the decision is exercised here directly.

TEST_GROUP(Kim2Modulation){};

TEST(Kim2Modulation, ModFromNameKnown) {
	CHECK_TRUE(KIM2::mod_from_name("LDK").has_value());
	CHECK_EQUAL((int)KineisModulation::LDK, (int)KIM2::mod_from_name("LDK").value());
	CHECK_EQUAL((int)KineisModulation::LDA2, (int)KIM2::mod_from_name("LDA2").value());
	CHECK_EQUAL((int)KineisModulation::VLDA4, (int)KIM2::mod_from_name("VLDA4").value());
}

TEST(Kim2Modulation, ModFromNameUnknownIsNullopt) {
	// Names not in KineisModulation, or malformed, must NOT be guessed.
	CHECK_FALSE(KIM2::mod_from_name("LDA2L").has_value());  // exists on KIM2 but not in our enum
	CHECK_FALSE(KIM2::mod_from_name("HDA4").has_value());
	CHECK_FALSE(KIM2::mod_from_name("UNKNOWN").has_value());
	CHECK_FALSE(KIM2::mod_from_name("").has_value());
	CHECK_FALSE(KIM2::mod_from_name("ldk").has_value());    // case-sensitive (module reports upper-case)
	CHECK_FALSE(KIM2::mod_from_name("LDA2 ").has_value());  // trailing space not tolerated
}

TEST(Kim2Modulation, VerifyMatchingModulation) {
	// Read-back equals the requested modulation -> no mismatch, no abort.
	for (auto m : { KineisModulation::LDK, KineisModulation::LDA2, KineisModulation::VLDA4 }) {
		const char *name = (m == KineisModulation::LDK) ? "LDK" : (m == KineisModulation::LDA2) ? "LDA2" : "VLDA4";
		KIM2::ModulationVerdict v = KIM2::verify_modulation(m, name);
		CHECK_TRUE(v.recognized);
		CHECK_FALSE(v.mismatch);
		CHECK_EQUAL((int)m, (int)v.actual);
	}
}

TEST(Kim2Modulation, VerifyMismatchExposesActual) {
	// THE field bug: service requests LDA2, the RCONF physically encodes LDK.
	// verify_modulation must flag the mismatch and surface the ACTUAL (LDK) so
	// the caller caches the truth and aborts instead of TXing a 12-byte LDA2
	// frame on a 16-byte-fixed LDK radio (+ERROR=5 / BAD_LEN).
	KIM2::ModulationVerdict v = KIM2::verify_modulation(KineisModulation::LDA2, "LDK");
	CHECK_TRUE(v.recognized);
	CHECK_TRUE(v.mismatch);
	CHECK_EQUAL((int)KineisModulation::LDK, (int)v.actual);

	// Symmetric case: requested LDK, module actually VLDA4.
	KIM2::ModulationVerdict v2 = KIM2::verify_modulation(KineisModulation::LDK, "VLDA4");
	CHECK_TRUE(v2.recognized);
	CHECK_TRUE(v2.mismatch);
	CHECK_EQUAL((int)KineisModulation::VLDA4, (int)v2.actual);
}

TEST(Kim2Modulation, VerifyUnrecognizedMakesNoDecision) {
	// Unrecognized read-back (LDA2L/HDA4/empty) -> recognized=false, mismatch=false:
	// we cannot prove a conflict, so the caller keeps its current modulation and
	// does NOT spuriously abort a working TX. actual falls back to requested.
	KIM2::ModulationVerdict v = KIM2::verify_modulation(KineisModulation::LDK, "LDA2L");
	CHECK_FALSE(v.recognized);
	CHECK_FALSE(v.mismatch);
	CHECK_EQUAL((int)KineisModulation::LDK, (int)v.actual);

	KIM2::ModulationVerdict v2 = KIM2::verify_modulation(KineisModulation::LDA2, "");
	CHECK_FALSE(v2.recognized);
	CHECK_FALSE(v2.mismatch);
}
