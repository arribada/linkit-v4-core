/**
 * @file bma400.cpp
 * @brief BMA400 accelerometer driver — I2C wrapper over Bosch C library + FIFO support.
 */

#include <cmath>
#include <algorithm>

#include "bma400.h"
#include "bma400.hpp"
#include "bma400_defs.h"

#include "debug.hpp"
#include "nrf_delay.h"
#include "nrf_i2c.hpp"
#include "bsp.hpp"
#include "pmu.hpp"
#include "error.hpp"
#include "config_store.hpp"
#include "gpio.hpp"
#include "interrupt_lock.hpp"

extern ConfigurationStore *configuration_store;

// ═══════════════════════════════════════════════════════
//  Device manager (static table — replaces std::map, ISR-safe)
// ═══════════════════════════════════════════════════════

/// @brief Static device table for Bosch C callback lookup (replaces std::map, ISR-safe).
static BMA400LL *s_devices[BMA400_MAX_DEVICES] = {};

/// @brief Register a BMA400LL instance and return its lookup ID.
/// @param dev  Pointer to the device to register.
/// @return Lookup ID (0..BMA400_MAX_DEVICES-1).
static uint8_t device_register(BMA400LL *dev) {
	for (uint8_t i = 0; i < BMA400_MAX_DEVICES; i++) {
		if (s_devices[i] == nullptr) {
			s_devices[i] = dev;
			return i;
		}
	}
	DEBUG_ERROR("BMA400: device table full (%u max)", BMA400_MAX_DEVICES);
	return 0;  // Fallback — shouldn't happen with 1-2 instances
}

/// @brief Remove a device from the lookup table.
static void device_unregister(uint8_t id) {
	if (id < BMA400_MAX_DEVICES) s_devices[id] = nullptr;
}

/// @brief Lookup a device by ID.
/// @param id  Device ID from device_register().
/// @return Pointer to the device, or nullptr if not found.
static BMA400LL *device_lookup(uint8_t id) {
	return (id < BMA400_MAX_DEVICES) ? s_devices[id] : nullptr;
}


// ═══════════════════════════════════════════════════════
//  BMA400LL — Constructor / Destructor
// ═══════════════════════════════════════════════════════

BMA400LL::BMA400LL(unsigned int bus, unsigned char addr, int wakeup_pin)
	: m_bus(bus)
	, m_addr(addr)
	, m_irq(NrfIRQ(wakeup_pin))
	, m_unique_id(device_register(this))
	, m_irq_pending(false)
	, m_g_range(BMA400_RANGE_2G)
	, m_power_mode(0)
	, m_wakeup_threshold(0.1)
	, m_wakeup_duration(1)
	, m_cal_x(0)
	, m_cal_y(0)
	, m_cal_z(1.0)
{
	DEBUG_TRACE("BMA400LL(%u, 0x%02X, pin=%d)", bus, addr, wakeup_pin);
	try {
		init();
	} catch (...) {
		DEBUG_ERROR("BMA400LL: init failed — unregistering");
		device_unregister(m_unique_id);
		throw;
	}
}

BMA400LL::~BMA400LL()
{
	device_unregister(m_unique_id);
}

/// @brief I2C probe, soft reset, enter SLEEP mode.
void BMA400LL::init()
{
	SensorsPowerGuard power_guard;
	int8_t rslt;

	m_bma400_dev.intf = BMA400_I2C_INTF;
	m_bma400_dev.intf_ptr = &m_unique_id;
	m_bma400_dev.read = reinterpret_cast<bma400_read_fptr_t>(i2c_read);
	m_bma400_dev.write = reinterpret_cast<bma400_write_fptr_t>(i2c_write);
	m_bma400_dev.delay_us = reinterpret_cast<bma400_delay_us_fptr_t>(delay_us);
	m_bma400_dev.read_write_len = BMA400_RW_LENGTH;
	m_bma400_dev.resolution = 12;

	rslt = bma400_init(&m_bma400_dev);
	check_result("bma400_init", rslt);

	rslt = bma400_soft_reset(&m_bma400_dev);
	check_result("bma400_soft_reset", rslt);

	setup_sleep_mode();
	DEBUG_TRACE("BMA400LL::init complete");
}


// ═══════════════════════════════════════════════════════
//  I2C callbacks (static, called from Bosch C driver)
// ═══════════════════════════════════════════════════════

/// @brief Bosch C driver I2C write callback.
int8_t BMA400LL::i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	BMA400LL *dev = device_lookup(*static_cast<uint8_t *>(intf_ptr));
	if (!dev) return BMA400_E_NULL_PTR;

	if (!length) return BMA400_OK;
	if (length > BMA400_MAX_LEN) return BMA400_E_INVALID_CONFIG;

	uint8_t buffer[BMA400_MAX_LEN + 1];
	buffer[0] = reg_addr;
	memcpy(&buffer[1], reg_data, length);

	NrfI2C::write(dev->m_bus, dev->m_addr, buffer, length + 1, false);
	return BMA400_OK;
}

/// @brief Bosch C driver I2C read callback.
int8_t BMA400LL::i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	BMA400LL *dev = device_lookup(*static_cast<uint8_t *>(intf_ptr));
	if (!dev) return BMA400_E_NULL_PTR;

	NrfI2C::write(dev->m_bus, dev->m_addr, &reg_addr, 1, true);
	NrfI2C::read(dev->m_bus, dev->m_addr, reg_data, length);
	return BMA400_OK;
}

/// @brief Bosch C driver delay callback.
void BMA400LL::delay_us(uint32_t period, void *)
{
	PMU::delay_us(period);
}


// ═══════════════════════════════════════════════════════
//  Utility
// ═══════════════════════════════════════════════════════

/// @brief Convert raw 12-bit LSB to m/s² for the given range.
/// @param accel_data  Raw accelerometer value (signed 12-bit).
/// @param g_range     Range in g-force (2, 4, 8, or 16).
/// @param bit_width   ADC resolution (12 for BMA400).
/// @return Acceleration in m/s².
double BMA400LL::lsb_to_ms2(int16_t accel_data, uint8_t g_range, uint8_t bit_width)
{
	constexpr double GRAVITY = 9.80665;
	int16_t half_scale = 1 << (bit_width - 1);
	return (GRAVITY * accel_data * g_range) / half_scale;
}

/// @brief Convert raw 12-bit LSB to g-force using current range setting.
double BMA400LL::lsb_to_g(int16_t raw) const
{
	uint8_t g = range_to_g(m_g_range);
	int16_t half_scale = 1 << (12 - 1);
	return static_cast<double>(raw * g) / half_scale;
}

/// @brief Convert range register value to g-force (2, 4, 8, or 16).
uint8_t BMA400LL::range_to_g(uint8_t range_reg) const
{
	static constexpr uint8_t g_table[] = { 2, 4, 8, 16 };
	return (range_reg < 4) ? g_table[range_reg] : 4;
}

/// @brief Convert g-force threshold to BMA400 register value (0-255).
uint8_t BMA400LL::calculate_threshold_reg(double threshold_g, uint8_t acc_range)
{
	double lsb = static_cast<double>(1 << (2 + acc_range)) / 4096.0;
	uint16_t threshold_raw = static_cast<uint16_t>(threshold_g / lsb);
	return static_cast<uint8_t>(std::min<uint16_t>(255, threshold_raw));
}

/// @brief Check Bosch API result — log and throw on error.
void BMA400LL::check_result(const char *api_name, int8_t rslt)
{
	if (rslt == BMA400_OK) return;

	const char *msg;
	switch (rslt) {
	case BMA400_E_NULL_PTR:       msg = "Null pointer"; break;
	case BMA400_E_COM_FAIL:       msg = "Communication failure"; break;
	case BMA400_E_INVALID_CONFIG: msg = "Invalid configuration"; break;
	case BMA400_E_DEV_NOT_FOUND:  msg = "Device not found"; break;
	default:                      msg = "Unknown error"; break;
	}

	DEBUG_ERROR("BMA400 [%s] Error %d: %s", api_name, rslt, msg);
	throw ErrorCode::I2C_COMMS_ERROR;
}


// ═══════════════════════════════════════════════════════
//  Power modes
// ═══════════════════════════════════════════════════════

/// @brief Enter SLEEP mode (~0.2 µA, no readings, wakeup via auto-wakeup engine).
void BMA400LL::setup_sleep_mode()
{
	int8_t rslt = bma400_set_power_mode(BMA400_MODE_SLEEP, &m_bma400_dev);
	check_result("set_power_mode(SLEEP)", rslt);
}

/// @brief NORMAL mode at 100 Hz — used for on-demand readings and calibration.
/// @note Named "active" to avoid confusion with the BMA400 LOW_POWER mode
///       which we don't use (NORMAL gives consistent readings with calibration).
void BMA400LL::setup_active_mode()
{
	int8_t rslt;

	rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &m_bma400_dev);
	check_result("set_power_mode(NORMAL)", rslt);

	m_sensor_conf[0].type = BMA400_ACCEL;
	m_sensor_conf[0].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_2;
	m_sensor_conf[0].param.accel.odr = BMA400_ODR_100HZ;
	m_sensor_conf[0].param.accel.range = m_g_range;
	m_sensor_conf[0].param.accel.osr = BMA400_ACCEL_OSR_SETTING_0;

	rslt = bma400_set_sensor_conf(m_sensor_conf, 1, &m_bma400_dev);
	check_result("set_sensor_conf(active)", rslt);
}

/// @brief Enter NORMAL mode with GEN1 interrupt config for motion wakeup.
void BMA400LL::setup_normal_mode()
{
	int8_t rslt;

	rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &m_bma400_dev);
	check_result("set_power_mode(NORMAL)", rslt);

	rslt = bma400_get_sensor_conf(m_sensor_conf, 1, &m_bma400_dev);
	check_result("get_sensor_conf", rslt);

	m_sensor_conf[0].type = BMA400_ACCEL;
	m_sensor_conf[0].param.accel.odr = BMA400_ODR_100HZ;
	m_sensor_conf[0].param.accel.range = m_g_range;
	m_sensor_conf[0].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;

	rslt = bma400_set_sensor_conf(m_sensor_conf, 1, &m_bma400_dev);
	check_result("set_sensor_conf(normal)", rslt);
}


// ═══════════════════════════════════════════════════════
//  Sensor reading
// ═══════════════════════════════════════════════════════

/// @brief Read calibrated XYZ + temperature.  Wakes from SLEEP, reads, returns to SLEEP.
void BMA400LL::read_xyz(double& x, double& y, double& z, int16_t& temperature)
{
	SensorsPowerGuard power_guard;
	int8_t rslt;
	struct bma400_sensor_data data;
	uint8_t g_force = range_to_g(m_g_range);

	if (m_power_mode == 0)
		setup_active_mode();
	else
		setup_normal_mode();

	// Enable DRDY interrupt and wait for stabilization
	struct bma400_int_enable int_en = { .type = BMA400_DRDY_INT_EN, .conf = BMA400_ENABLE };
	rslt = bma400_enable_interrupt(&int_en, 1, &m_bma400_dev);
	check_result("enable_interrupt(DRDY)", rslt);

	// Clear any stale DRDY status from previous read, then wait for a FRESH sample
	uint16_t int_status = 0;
	int8_t __attribute__((unused)) clr = bma400_get_interrupt_status(&int_status, &m_bma400_dev);  // Clear stale
	PMU::delay_ms(20);  // Wait for new sample at ODR

	// Poll for fresh data ready with timeout
	bool data_ready = false;
	for (unsigned int i = 0; i < BMA400_DRDY_MAX_POLLS; i++) {
		rslt = bma400_get_interrupt_status(&int_status, &m_bma400_dev);
		if (int_status & BMA400_ASSERTED_DRDY_INT) {
			rslt = bma400_get_accel_data(BMA400_DATA_ONLY, &data, &m_bma400_dev);
			data_ready = true;
			break;
		}
		PMU::delay_ms(1);
	}
	if (!data_ready) {
		DEBUG_WARN("BMA400::read_xyz: DRDY timeout");
		setup_sleep_mode();
		// Don't leave x/y/z (= m_last_x/y/z) at the previous cycle's values: a
		// frozen at-rest sample would feed mortality as a fresh LOW-activity
		// reading (false death signal). Zero them so compute_activity's
		// implausible-g_mag guard treats it as "no valid reading" (biased ALIVE).
		x = 0.0; y = 0.0; z = 0.0;
		return;
	}

	// Convert to g-force and apply calibration
	constexpr double G = 9.80665;
	double x_raw = lsb_to_ms2(data.x, g_force, 12) / G;
	double y_raw = lsb_to_ms2(data.y, g_force, 12) / G;
	double z_raw = lsb_to_ms2(data.z, g_force, 12) / G;

	x = x_raw - m_cal_x;
	y = y_raw - m_cal_y;
	z = z_raw - (m_cal_z - 1.0);

	DEBUG_INFO("BMA400: raw(%d|%d|%d) g(%.2f|%.2f|%.2f) cal(%.2f|%.2f|%.2f)",
	           data.x, data.y, data.z, x_raw, y_raw, z_raw, x, y, z);

	// Read temperature while sensor is in active mode (doesn't work in SLEEP)
	bma400_get_temperature_data(&temperature, &m_bma400_dev);

	setup_sleep_mode();
}

/// @brief Read temperature — requires sensor to be in active mode.
/// @note Caller must ensure the sensor is NOT in SLEEP mode before calling.
int16_t BMA400LL::read_temperature()
{
	SensorsPowerGuard power_guard;
	// Briefly enter active mode to get a valid temperature reading
	setup_active_mode();
	PMU::delay_ms(12);  // Temperature conversion time ~11.2 ms

	int16_t temperature_data;
	bma400_get_temperature_data(&temperature_data, &m_bma400_dev);

	setup_sleep_mode();
	return temperature_data;
}


// ═══════════════════════════════════════════════════════
//  Configuration
// ═══════════════════════════════════════════════════════

void BMA400LL::set_wakeup_threshold(double threshold) { m_wakeup_threshold = threshold; }
void BMA400LL::set_wakeup_duration(double duration) { m_wakeup_duration = duration; }
void BMA400LL::set_range(unsigned int g_force) { m_g_range = static_cast<uint8_t>(g_force); }

void BMA400LL::set_power_mode(unsigned int power_mode)
{
	DEBUG_INFO("BMA400LL::set_power_mode: %u (%s wakeup)",
	           power_mode, power_mode == 0 ? "LOW_POWER" : "NORMAL");
	m_power_mode = static_cast<uint8_t>(power_mode);
}

void BMA400LL::set_x_calibration(double x) { m_cal_x = x; }
void BMA400LL::set_y_calibration(double y) { m_cal_y = y; }
void BMA400LL::set_z_calibration(double z) { m_cal_z = z; }


// ═══════════════════════════════════════════════════════
//  Calibration
// ═══════════════════════════════════════════════════════

/// @brief Auto-calibrate: average 200 samples at rest → X/Y/Z offsets in g.
void BMA400LL::calibrate_offset(uint8_t g_range, double& offset_x, double& offset_y, double& offset_z)
{
	SensorsPowerGuard power_guard;
	int8_t rslt;
	constexpr uint8_t N_SAMPLES = 200;
	constexpr double GRAVITY = 9.80665;
	double acc_x = 0, acc_y = 0, acc_z = 0;
	uint8_t g_force = range_to_g(g_range);

	// NORMAL mode at 100 Hz for calibration
	rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &m_bma400_dev);
	check_result("calibrate: set_power_mode", rslt);

	struct bma400_sensor_conf conf = {};
	conf.type = BMA400_ACCEL;
	rslt = bma400_get_sensor_conf(&conf, 1, &m_bma400_dev);
	conf.param.accel.odr = BMA400_ODR_100HZ;
	conf.param.accel.range = g_range;
	conf.param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_2;
	rslt = bma400_set_sensor_conf(&conf, 1, &m_bma400_dev);
	check_result("calibrate: set_sensor_conf", rslt);

	struct bma400_int_enable int_en = { .type = BMA400_DRDY_INT_EN, .conf = BMA400_ENABLE };
	rslt = bma400_enable_interrupt(&int_en, 1, &m_bma400_dev);
	check_result("calibrate: enable_interrupt", rslt);
	PMU::delay_ms(100);

	DEBUG_TRACE("BMA400: calibrating (%u samples)...", N_SAMPLES);
	unsigned int n_ok = 0;
	for (uint8_t i = 0; i < N_SAMPLES; ++i) {
		// Wait for DRDY with timeout
		uint16_t int_status = 0;
		bool ready = false;
		for (unsigned int j = 0; j < BMA400_DRDY_MAX_POLLS; j++) {
			rslt = bma400_get_interrupt_status(&int_status, &m_bma400_dev);
			if (int_status & BMA400_ASSERTED_DRDY_INT) { ready = true; break; }
			PMU::delay_ms(1);
		}
		if (!ready) {
			DEBUG_WARN("BMA400: calibrate DRDY timeout at sample %u", i);
			continue;  // Skip this sample, don't use stale data
		}

		struct bma400_sensor_data data;
		rslt = bma400_get_accel_data(BMA400_DATA_ONLY, &data, &m_bma400_dev);
		if (rslt != BMA400_OK) {
			DEBUG_WARN("BMA400: calibrate accel read failed at sample %u", i);
			continue;
		}
		acc_x += lsb_to_ms2(data.x, g_force, 12) / GRAVITY;
		acc_y += lsb_to_ms2(data.y, g_force, 12) / GRAVITY;
		acc_z += lsb_to_ms2(data.z, g_force, 12) / GRAVITY;
		n_ok++;
	}

	setup_sleep_mode();

	// Divide by the number of samples that ACTUALLY landed, not the constant
	// N_SAMPLES: if DRDY was failing most iterations `continue` above, and
	// dividing by 200 yields garbage (all-skipped -> offset_z=0 instead of ~1g)
	// that would be written to AXL.CAL. If nothing landed, keep identity offsets
	// (no calibration) rather than write zeros.
	if (n_ok == 0) {
		DEBUG_ERROR("BMA400: calibration failed — no valid samples; using identity offsets");
		offset_x = 0.0;
		offset_y = 0.0;
		offset_z = 1.0;
	} else {
		if (n_ok < N_SAMPLES / 2)
			DEBUG_WARN("BMA400: calibration used only %u/%u samples", n_ok, (unsigned)N_SAMPLES);
		offset_x = acc_x / n_ok;
		offset_y = acc_y / n_ok;
		offset_z = acc_z / n_ok;
	}

	DEBUG_TRACE("BMA400: calibrated x=%.4f y=%.4f z=%.4f g", offset_x, offset_y, offset_z);
}


// ═══════════════════════════════════════════════════════
//  Wakeup / Interrupt
// ═══════════════════════════════════════════════════════

/// @brief Enable motion wakeup: re-init device, configure mode, register IRQ callback.
void BMA400LL::enable_wakeup(std::function<void()> func)
{
	SensorsPowerGuard power_guard;
	int8_t rslt;

	DEBUG_INFO("BMA400::enable_wakeup: mode=%u (%s)",
	           m_power_mode, m_power_mode == 0 ? "LOW_POWER" : "NORMAL");

	rslt = bma400_init(&m_bma400_dev);
	check_result("enable_wakeup: init", rslt);

	rslt = bma400_soft_reset(&m_bma400_dev);
	check_result("enable_wakeup: soft_reset", rslt);

	if (m_power_mode == 0) {
		setup_active_mode();
		enable_wakeup_low_power(func);
	} else {
		setup_normal_mode();
		enable_wakeup_normal(func);
	}
}

/// @brief Configure BMA400 auto-wakeup engine with threshold detection (mode 0).
void BMA400LL::enable_wakeup_low_power(std::function<void()> func)
{
	int8_t rslt;

	struct bma400_device_conf dev_conf = {};
	dev_conf.type = BMA400_AUTOWAKEUP_INT;

	rslt = bma400_get_device_conf(&dev_conf, 1, &m_bma400_dev);
	if (rslt == BMA400_OK) {
		dev_conf.param.wakeup.wakeup_axes_en = BMA400_AXIS_XYZ_EN;
		dev_conf.param.wakeup.wakeup_ref_update = BMA400_UPDATE_EVERY_TIME;
		dev_conf.param.wakeup.sample_count = BMA400_SAMPLE_COUNT_4;
		dev_conf.param.wakeup.int_wkup_threshold = calculate_threshold_reg(m_wakeup_threshold, m_g_range);
		dev_conf.param.wakeup.int_chan = BMA400_MAP_BOTH_INT_PINS;

		rslt = bma400_set_device_conf(&dev_conf, 1, &m_bma400_dev);
		if (rslt == BMA400_OK) {
			m_int_enable.type = BMA400_GEN1_INT_EN;
			m_int_enable.conf = BMA400_ENABLE;
			rslt = bma400_enable_interrupt(&m_int_enable, 1, &m_bma400_dev);
		}
	}

	m_irq.enable([this, func]() {
		if (!m_irq_pending) {
			m_irq_pending = true;
			func();
		}
	});
}

/// @brief Configure GEN1 activity interrupt for motion detection (mode 1).
void BMA400LL::enable_wakeup_normal(std::function<void()> func)
{
	int8_t rslt;

	m_sensor_conf[1].type = BMA400_GEN1_INT;
	m_sensor_conf[1].param.gen_int.gen_int_thres = calculate_threshold_reg(m_wakeup_threshold, m_g_range);
	m_sensor_conf[1].param.gen_int.gen_int_dur = static_cast<uint8_t>(m_wakeup_duration - 1);
	m_sensor_conf[1].param.gen_int.axes_sel = BMA400_AXIS_XYZ_EN;
	m_sensor_conf[1].param.gen_int.data_src = BMA400_DATA_SRC_ACC_FILT2;
	m_sensor_conf[1].param.gen_int.criterion_sel = BMA400_ACTIVITY_INT;
	m_sensor_conf[1].param.gen_int.evaluate_axes = BMA400_ANY_AXES_INT;
	m_sensor_conf[1].param.gen_int.ref_update = BMA400_UPDATE_EVERY_TIME;
	m_sensor_conf[1].param.gen_int.hysteresis = BMA400_HYST_96_MG;
	m_sensor_conf[1].param.gen_int.int_thres_ref_x = 0;
	m_sensor_conf[1].param.gen_int.int_thres_ref_y = 0;
	m_sensor_conf[1].param.gen_int.int_thres_ref_z = 32;
	m_sensor_conf[1].param.gen_int.int_chan = BMA400_INT_CHANNEL_1;

	rslt = bma400_set_sensor_conf(m_sensor_conf, 2, &m_bma400_dev);
	check_result("enable_wakeup_normal: set_sensor_conf", rslt);

	m_int_enable.type = BMA400_GEN1_INT_EN;
	m_int_enable.conf = BMA400_ENABLE;
	rslt = bma400_enable_interrupt(&m_int_enable, 1, &m_bma400_dev);
	check_result("enable_wakeup_normal: enable_interrupt", rslt);

	m_irq.enable([this, func]() {
		if (!m_irq_pending) {
			m_irq_pending = true;
			func();
		}
	});
}

/// @brief Disable wakeup interrupt, return to SLEEP mode.
void BMA400LL::disable_wakeup()
{
	SensorsPowerGuard power_guard;
	int8_t rslt;

	m_irq.disable();

	rslt = bma400_set_power_mode(BMA400_MODE_SLEEP, &m_bma400_dev);
	check_result("disable_wakeup: set_power_mode", rslt);

	m_int_enable.type = BMA400_GEN1_INT_EN;
	m_int_enable.conf = BMA400_DISABLE;
	rslt = bma400_enable_interrupt(&m_int_enable, 1, &m_bma400_dev);
	check_result("disable_wakeup: disable_interrupt", rslt);

	DEBUG_INFO("BMA400::disable_wakeup complete");
}

/// @brief Atomically check and clear the wakeup IRQ pending flag.
bool BMA400LL::check_and_clear_wakeup()
{
	InterruptLock lock;
	bool value = m_irq_pending;
	m_irq_pending = false;
	return value;
}


// ═══════════════════════════════════════════════════════
//  FIFO batch reading
// ═══════════════════════════════════════════════════════

/// @brief Enable hardware FIFO in stream mode at the given ODR.
/// @note Caller must ensure VSENSORS is ON (no local SensorsPowerGuard here —
///       the FIFO needs power to persist between calls).
void BMA400LL::enable_fifo(uint8_t odr)
{
	int8_t rslt;

	// Enter NORMAL mode with requested ODR
	rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &m_bma400_dev);
	check_result("enable_fifo: set_power_mode", rslt);

	m_sensor_conf[0].type = BMA400_ACCEL;
	m_sensor_conf[0].param.accel.odr = odr;
	m_sensor_conf[0].param.accel.range = m_g_range;
	m_sensor_conf[0].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_2;
	m_sensor_conf[0].param.accel.osr = BMA400_ACCEL_OSR_SETTING_0;
	rslt = bma400_set_sensor_conf(m_sensor_conf, 1, &m_bma400_dev);
	check_result("enable_fifo: set_sensor_conf", rslt);

	// Configure FIFO via device conf: XYZ data enabled
	struct bma400_device_conf dev_conf = {};
	dev_conf.type = BMA400_FIFO_CONF;
	dev_conf.param.fifo_conf.conf_regs = BMA400_FIFO_X_EN | BMA400_FIFO_Y_EN | BMA400_FIFO_Z_EN;
	dev_conf.param.fifo_conf.conf_status = BMA400_ENABLE;
	rslt = bma400_set_device_conf(&dev_conf, 1, &m_bma400_dev);
	check_result("enable_fifo: set_device_conf", rslt);

	// Flush any stale data
	bma400_set_fifo_flush(&m_bma400_dev);

	m_fifo_enabled = true;
	DEBUG_INFO("BMA400: FIFO enabled (ODR=%u, range=%uG)", odr, range_to_g(m_g_range));
}

/// @brief Disable FIFO, return to SLEEP mode.
/// @note Caller releases VSENSORS after this call.
void BMA400LL::disable_fifo()
{

	struct bma400_device_conf dev_conf = {};
	dev_conf.type = BMA400_FIFO_CONF;
	dev_conf.param.fifo_conf.conf_status = BMA400_DISABLE;
	int8_t rslt = bma400_set_device_conf(&dev_conf, 1, &m_bma400_dev);
	if (rslt != BMA400_OK)
		DEBUG_WARN("BMA400: disable_fifo set_device_conf failed: %d", rslt);

	setup_sleep_mode();
	m_fifo_enabled = false;
	DEBUG_INFO("BMA400: FIFO disabled");
}

/// @brief Read and parse all available FIFO frames into the output array.
/// @return Number of samples read (0 if empty or error).
/// @note Caller must ensure VSENSORS is ON.
unsigned int BMA400LL::read_fifo(BMA400FifoSample *samples, unsigned int max_samples)
{
	if (!m_fifo_enabled || !samples || max_samples == 0)
		return 0;

	// Read raw FIFO data (1024 bytes max + overhead)
	static constexpr uint16_t FIFO_BUF_SIZE = 1028;
	uint8_t fifo_buffer[FIFO_BUF_SIZE];

	struct bma400_fifo_data fifo_data = {};
	fifo_data.data = fifo_buffer;
	fifo_data.length = FIFO_BUF_SIZE;

	int8_t rslt = bma400_get_fifo_data(&fifo_data, &m_bma400_dev);
	if (rslt != BMA400_OK) {
		DEBUG_WARN("BMA400: FIFO read failed (%d)", rslt);
		return 0;
	}

	// Parse frames — use Bosch's bma400_fifo_sensor_data struct
	static constexpr uint16_t MAX_FRAMES = 170;  // 1024 / 6
	struct bma400_fifo_sensor_data accel_data[MAX_FRAMES];
	uint16_t frame_count = std::min<uint16_t>(max_samples, MAX_FRAMES);

	rslt = bma400_extract_accel(&fifo_data, accel_data, &frame_count, &m_bma400_dev);
	if (rslt != BMA400_OK) {
		DEBUG_WARN("BMA400: FIFO extract failed (%d)", rslt);
		return 0;
	}

	for (uint16_t i = 0; i < frame_count; i++) {
		samples[i].x = accel_data[i].x;
		samples[i].y = accel_data[i].y;
		samples[i].z = accel_data[i].z;
	}

	return frame_count;
}


// ═══════════════════════════════════════════════════════
//  BMA400 high-level Sensor class
// ═══════════════════════════════════════════════════════

BMA400::BMA400()
	: Sensor("AXL")
	, m_bma400(BMA400LL(BMA400_DEVICE, BMA400_ADDRESS, BMA400_WAKEUP_PIN))
	, m_cal(Calibration("AXL"))
{
	if (configuration_store) {
		unsigned int g_range = configuration_store->read_param<unsigned int>(ParamID::AXL_SENSOR_MEASUREMENT_RANGE);
		unsigned int power_mode = configuration_store->read_param<unsigned int>(ParamID::AXL_SENSOR_POWER_MODE);
		m_bma400.set_range(g_range);
		m_bma400.set_power_mode(power_mode);

		m_fifo_enabled = configuration_store->read_param<bool>(ParamID::AXL_FIFO_ENABLE);
		m_fifo_sample_count = configuration_store->read_param<unsigned int>(ParamID::AXL_FIFO_SAMPLE_COUNT);
		if (m_fifo_sample_count == 0 || m_fifo_sample_count > 170)
			m_fifo_sample_count = 50;

		DEBUG_INFO("BMA400: config g_range=%u power_mode=%u fifo=%u samples=%u",
		           g_range, power_mode, m_fifo_enabled, m_fifo_sample_count);
	}

	load_calibration();
}

/// @brief Load X/Y/Z calibration from persistent file.  Falls back to defaults (0, 0, 1g).
void BMA400::load_calibration()
{
	try {
		double x = m_cal.read(static_cast<unsigned int>(CalibrationPoint::X));
		double y = m_cal.read(static_cast<unsigned int>(CalibrationPoint::Y));
		double z = m_cal.read(static_cast<unsigned int>(CalibrationPoint::Z));
		m_bma400.set_x_calibration(x);
		m_bma400.set_y_calibration(y);
		m_bma400.set_z_calibration(z);
		DEBUG_INFO("BMA400: calibration loaded X=%.4f Y=%.4f Z=%.4f g", x, y, z);
	} catch (...) {
		m_bma400.set_x_calibration(0.0);
		m_bma400.set_y_calibration(0.0);
		m_bma400.set_z_calibration(1.0);
		DEBUG_INFO("BMA400: using default calibration X=0 Y=0 Z=1 g");
	}
}

/// @brief Read sensor value by channel.  Channel 1 triggers a new XYZ reading (single or FIFO batch).
double BMA400::read(unsigned int offset)
{
	switch (offset) {
	case 0: return static_cast<double>(m_last_temperature) / 10.0;
	case 1:
		// Channel 1 triggers a new reading — single sample or FIFO batch
		if (m_fifo_enabled)
			read_fifo_batch();
		else
			m_bma400.read_xyz(m_last_x, m_last_y, m_last_z, m_last_temperature);
		return m_last_x;
	case 2: return m_last_y;
	case 3: return m_last_z;
	case 4: return compute_activity();
	case 5: return static_cast<double>(m_bma400.check_and_clear_wakeup());
	default: return 0.0;
	}
}

/// @brief Read a FIFO batch, average the samples, apply calibration.
///
/// On first call, enables the FIFO at 100 Hz.  Subsequent calls read
/// whatever has accumulated since the last read.  If fewer than
/// m_fifo_sample_count are available, uses what's there (minimum 1).
/// Falls back to single-sample read_xyz() if FIFO read returns 0.
void BMA400::read_fifo_batch()
{
	// Start FIFO on first read — keep VSENSORS ON for the entire FIFO session
	if (!m_fifo_started) {
		GPIOPins::acquire_sensors_pwr();  // Hold VSENSORS ON until FIFO disabled
		m_bma400.enable_fifo(BMA400_ODR_100HZ);
		m_fifo_started = true;
		// Wait for samples to accumulate (sample_count / 100 Hz)
		unsigned int wait_ms = (m_fifo_sample_count * 10) + 50;  // +50 ms margin
		PMU::delay_ms(wait_ms);
	}

	// NOTE (2026-07 audit): FIFO batch mode does NOT refresh die temperature —
	// port 0 (temperature) keeps its last single-read value. Mortality uses the
	// thermistor body temp (not this), so this only affects the AXL log's temp
	// column when FIFO mode is used exclusively. Documented, not corrected.
	BMA400FifoSample samples[170];
	unsigned int count = m_bma400.read_fifo(samples, m_fifo_sample_count);

	if (count == 0) {
		// FIFO empty or error — fallback to single sample
		DEBUG_WARN("BMA400: FIFO empty, fallback to single read");
		m_bma400.read_xyz(m_last_x, m_last_y, m_last_z, m_last_temperature);
		return;
	}

	// Average raw samples then convert to g + apply calibration
	double sum_x = 0, sum_y = 0, sum_z = 0;
	for (unsigned int i = 0; i < count; i++) {
		sum_x += m_bma400.lsb_to_g(samples[i].x);
		sum_y += m_bma400.lsb_to_g(samples[i].y);
		sum_z += m_bma400.lsb_to_g(samples[i].z);
	}

	double avg_x = sum_x / count;
	double avg_y = sum_y / count;
	double avg_z = sum_z / count;

	// Apply calibration (same as read_xyz)
	m_last_x = avg_x - m_bma400.get_x_calibration();
	m_last_y = avg_y - m_bma400.get_y_calibration();
	m_last_z = avg_z - (m_bma400.get_z_calibration() - 1.0);

	// Skip temperature read during FIFO — read_temperature() puts BMA400
	// into sleep mode which kills the FIFO. Temperature from last single
	// read or init is still valid (BMA400 die temp changes slowly).

	DEBUG_INFO("BMA400: FIFO batch %u samples avg(%.3f|%.3f|%.3f) g",
	           count, m_last_x, m_last_y, m_last_z);
}

/// @brief Compute activity metric: deviation from 1g (rest) scaled to 0-255.
double BMA400::compute_activity()
{
	double g_mag = std::sqrt(m_last_x * m_last_x + m_last_y * m_last_y + m_last_z * m_last_z);

	// M3 (2026-07): a healthy accelerometer always senses ~1g total magnitude.
	// g_mag ~= 0 means every axis read ~0 (stuck bus / disconnected sensor). The
	// |g_mag - 1| metric would turn that into MAX activity (255) via fragile
	// accidental arithmetic. Make the failure EXPLICIT and biased toward ALIVE
	// (max activity => not immobile), matching the mortality design's rule that
	// a missing/failed sensor must never push a LIVE bird toward a false death.
	if (g_mag < 0.1) {
		DEBUG_WARN("BMA400: implausible g_mag=%.3f (stuck/disconnected?) — activity biased toward ALIVE", g_mag);
		m_last_activity = 255;
		return 255.0;
	}

	double activity_g = std::abs(g_mag - 1.0);

	uint8_t g_range = m_bma400.range_to_g(m_bma400.get_range());
	double max_dev = std::max(1.0, static_cast<double>(g_range) - 1.0);

	if (activity_g > max_dev) activity_g = max_dev;
	m_last_activity = static_cast<uint8_t>((activity_g / max_dev) * 255.0);
	return static_cast<double>(m_last_activity);
}

/// @brief Write calibration value or trigger action (see SCALW offset table in .hpp).
void BMA400::calibration_write(const double value, const unsigned int offset)
{
	DEBUG_TRACE("BMA400::calibration_write: value=%.2f offset=%u", value, offset);

	switch (offset) {
	case 0:
		m_bma400.set_x_calibration(value);
		m_cal.write(static_cast<unsigned int>(CalibrationPoint::X), value);
		break;
	case 1:
		m_bma400.set_y_calibration(value);
		m_cal.write(static_cast<unsigned int>(CalibrationPoint::Y), value);
		break;
	case 2:
		m_bma400.set_z_calibration(value);
		m_cal.write(static_cast<unsigned int>(CalibrationPoint::Z), value);
		break;
	case 3: {
		double ox, oy, oz;
		m_bma400.calibrate_offset(m_bma400.get_range(), ox, oy, oz);
		m_bma400.set_x_calibration(ox);
		m_bma400.set_y_calibration(oy);
		m_bma400.set_z_calibration(oz);
		m_cal.write(static_cast<unsigned int>(CalibrationPoint::X), ox);
		m_cal.write(static_cast<unsigned int>(CalibrationPoint::Y), oy);
		m_cal.write(static_cast<unsigned int>(CalibrationPoint::Z), oz);
		DEBUG_INFO("BMA400: auto-calibrated X=%.4f Y=%.4f Z=%.4f g", ox, oy, oz);
		break;
	}
	case 4:
		m_bma400.read_xyz(m_last_x, m_last_y, m_last_z, m_last_temperature);
		DEBUG_INFO("BMA400: calibrated X=%.4f Y=%.4f Z=%.4f g", m_last_x, m_last_y, m_last_z);
		break;
	case 5:
		DEBUG_INFO("BMA400: cal coeff X=%.4f Y=%.4f Z=%.4f g",
		           m_bma400.get_x_calibration(), m_bma400.get_y_calibration(), m_bma400.get_z_calibration());
		break;
	case 6:
		m_cal.save();
		DEBUG_INFO("BMA400: calibration saved");
		break;
	case 7:  m_bma400.set_wakeup_threshold(value); break;
	case 8:  m_bma400.set_wakeup_duration(value); break;
	case 9:  m_bma400.set_range(static_cast<unsigned int>(value)); break;
	case 10: m_bma400.set_power_mode(static_cast<unsigned int>(value)); break;
	default: DEBUG_WARN("BMA400: invalid calibration_write offset %u", offset); break;
	}
}

/// @brief Read calibrated sensor value or configuration by offset (see SCALR table in .hpp).
void BMA400::calibration_read(double& value, const unsigned int offset)
{
	if (offset >= 1 && offset <= 3)
		m_bma400.read_xyz(m_last_x, m_last_y, m_last_z, m_last_temperature);

	switch (offset) {
	case 1: value = m_last_x; break;
	case 2: value = m_last_y; break;
	case 3: value = m_last_z; break;
	case 4: value = m_bma400.get_x_calibration(); break;
	case 5: value = m_bma400.get_y_calibration(); break;
	case 6: value = m_bma400.get_z_calibration(); break;
	case 7: value = m_bma400.get_wakeup_threshold(); break;
	case 8: value = m_bma400.get_wakeup_duration(); break;
	case 9: value = static_cast<double>(m_bma400.get_range()); break;
	case 10: value = static_cast<double>(m_bma400.get_power_mode()); break;
	default: value = 0.0; DEBUG_WARN("BMA400: invalid calibration_read offset %u", offset); break;
	}

	DEBUG_INFO("BMA400::calibration_read: offset=%u value=%.4f", offset, value);
}

/// @brief Register wakeup motion interrupt handler on the BMA400.
void BMA400::install_event_handler(unsigned int, std::function<void()> handler)
{
	m_bma400.enable_wakeup(handler);
}

/// @brief Unregister wakeup handler, stop FIFO if running, return to SLEEP.
void BMA400::remove_event_handler(unsigned int)
{
	m_bma400.disable_wakeup();
	if (m_fifo_started) {
		m_bma400.disable_fifo();
		m_fifo_started = false;
		GPIOPins::release_sensors_pwr();  // Release VSENSORS held since FIFO start
	}
}

/// @brief Persist calibration offsets to flash (AXL.CAL file).
void BMA400::calibration_save(bool force)
{
	m_cal.save(force);
}
