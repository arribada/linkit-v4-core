#pragma once

/**
 * @file ad5933.hpp
 * @brief AD5933 impedance network analyser driver (I2C, 12-bit).
 *
 * The AD5933 measures complex impedance by injecting a known frequency
 * and reading back the real/imaginary DFT components.  Used by the CDT
 * (Conductivity-Depth-Temperature) sensor for water conductivity measurement.
 */

#include <cstdint>

/// @brief Output voltage range and PGA gain combinations.
enum class VRange {
	V1_GAIN1X,
	V2_GAIN1X,
	V200MV_GAIN1X,
	V400MV_GAIN1X,
	V1_GAIN0_5X,
	V2_GAIN0_5X,
	V200MV_GAIN0_5X,
	V400MV_GAIN0_5X,
};

/// @brief Abstract AD5933 interface (mockable for tests).
class AD5933 {
public:
	virtual ~AD5933() = default;
	virtual void start(unsigned int frequency, VRange vrange) = 0;
	virtual void stop() = 0;
	// Note: "impedence" is a legacy typo — kept for API compatibility across callers.
	virtual double get_impedence(unsigned int averaging, double gain) = 0;
	virtual void get_real_imaginary(int16_t &real, int16_t &imag) = 0;
};

/// @brief AD5933 low-level I2C driver.
class AD5933LL : public AD5933 {
public:
	/// @param bus   I2C bus index (BSP::I2C enum).
	/// @param addr  7-bit I2C address (typically 0x0D).
	/// @throws ErrorCode::I2C_COMMS_ERROR if device not responding.
	AD5933LL(unsigned int bus, unsigned char addr);

	void start(unsigned int frequency, VRange vrange) override;
	void stop() override;
	double get_impedence(unsigned int averaging, double gain) override;
	void get_real_imaginary(int16_t &real, int16_t &imag) override;

private:
	unsigned int m_bus;
	unsigned char m_addr;
	uint8_t m_gain_setting;

	enum class Reg : uint8_t {
		CONTROL_HIGH = 0x80,
		CONTROL_LOW = 0x81,
		START_FREQ_23_16 = 0x82,
		START_FREQ_15_8 = 0x83,
		START_FREQ_7_0 = 0x84,
		FREQ_INC_23_16 = 0x85,
		FREQ_INC_15_8 = 0x86,
		FREQ_INC_7_0 = 0x87,
		NUM_INC_15_8 = 0x88,
		NUM_INC_7_0 = 0x89,
		SETTLING_15_8 = 0x8A,
		SETTLING_7_0 = 0x8B,
		STATUS = 0x8F,
		REAL_15_8 = 0x94,
		REAL_7_0 = 0x95,
		IMAG_15_8 = 0x96,
		IMAG_7_0 = 0x97,
	};

	enum class CtrlHigh : uint8_t {
		INIT_START_FREQ = (1 << 4),
		START_SWEEP = (2 << 4),
		INC_FREQ = (3 << 4),
		REPEAT_FREQ = (4 << 4),
		MEASURE_TEMP = (9 << 4),
		POWER_DOWN = (10 << 4),
		STANDBY = (11 << 4),
		OPV_2V = (0 << 1),
		OPV_200MV = (1 << 1),
		OPV_400MV = (2 << 1),
		OPV_1V = (3 << 1),
		PGA_GAIN_1X = (1 << 0),
	};

	enum class CtrlLow : uint8_t {
		RESET = (1 << 4),
		EXT_CLK = (1 << 3),
	};

	enum class Status : uint8_t {
		VALID_TEMP = (1 << 0),
		VALID_IQ = (1 << 1),
		SWEEP_COMPLETE = (1 << 2),
	};

	uint8_t read_reg(Reg reg);
	void write_reg(Reg reg, uint8_t value);
	void set_start_frequency(unsigned int frequency);
	void set_number_of_increments(unsigned int num);
	void set_frequency_increment(unsigned int inc);
	void set_settling_times(unsigned int settling);
	void initialize();
	void reset();
	void standby();
	void startsweep();

	/// @brief Wait for IQ data ready (bounded — times out after ~500 ms).
	[[nodiscard]] bool wait_iq_data_ready();

	void powerdown();
	void gain(VRange setting);
	uint8_t status();
	int16_t read_real();
	int16_t read_imag();
	void dump_regs();
};
