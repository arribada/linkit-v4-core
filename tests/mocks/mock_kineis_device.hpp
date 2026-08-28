#pragma once

#include "kineis_device.hpp"
#include "CppUTestExt/MockSupport.h"

class MockKineisDevice : public KineisDevice {
public:
	MockKineisDevice() {}
	virtual ~MockKineisDevice() {}

	void send(const KineisModulation mode, const KineisPacket & /*packet*/, const unsigned int size_bits) override {
		mock()
		    .actualCall("send")
		    .onObject(this)
		    .withUnsignedIntParameter("mode", (unsigned int)mode)
		    .withUnsignedIntParameter("size_bits", size_bits);
	}

	void stop_send() override { mock().actualCall("stop_send").onObject(this); }

	void power_off_immediate() override { mock().actualCall("power_off_immediate").onObject(this); }

	void start_receive(const KineisModulation mode) override {
		mock().actualCall("start_receive").onObject(this).withUnsignedIntParameter("mode", (unsigned int)mode);
	}

	bool stop_receive() override {
		mock().actualCall("stop_receive").onObject(this);
		return true;
	}

	void set_frequency(double freq_mhz) override {
		mock().actualCall("set_frequency").onObject(this).withDoubleParameter("freq", freq_mhz);
	}

	void set_tcxo_warmup_time(unsigned int ms) override {
		mock().actualCall("set_tcxo_warmup_time").onObject(this).withUnsignedIntParameter("time", ms);
	}

	bool switch_modulation(KineisModulation mode, const std::string &rconf_hex) override {
		mock()
		    .actualCall("switch_modulation")
		    .onObject(this)
		    .withUnsignedIntParameter("mode", (unsigned int)mode)
		    .withStringParameter("rconf", rconf_hex.c_str());
		m_current_mod = mode;
		return true;
	}

	KineisModulation get_current_modulation() const override { return m_current_mod; }

	// Test helper: emulate the non-adaptive "current" modulation that the real
	// KIM2 state_init() reads back from the master RCONF (AT+RCONF=?). Lets a
	// legacy/non-adaptive test pin the device to LDK/VLDA4 without driving the
	// switch_modulation() mock-call path.
	void test_set_current_modulation(KineisModulation mode) { m_current_mod = mode; }

	// Test helper: emulate the SmdSat autofallback cooldown window. It is what
	// ArgosTxService::react uses to tell "module refused, try later" apart from
	// a real fault -- a distinction with no coverage until now. Defaults to 0,
	// i.e. the base-class behaviour, so existing tests are unaffected.
	unsigned int cooldown_remaining_ms() const override { return m_cooldown_remaining_ms; }
	void test_set_cooldown_remaining_ms(unsigned int ms) { m_cooldown_remaining_ms = ms; }

	void set_lpm_mode(uint8_t lpm_bitmap) override {
		(void)lpm_bitmap;  // LPM is SMD-specific, not verified in generic TX tests
	}

private:
	KineisModulation m_current_mod = KineisModulation::LDA2;
	unsigned int m_cooldown_remaining_ms = 0;
};
