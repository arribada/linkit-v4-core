#pragma once

/**
 * @file bma400.hpp
 * @brief BMA400 3-axis accelerometer driver (I2C, 12-bit, Bosch).
 *
 * Two-layer architecture:
 *  - BMA400LL: low-level I2C driver wrapping the Bosch C library.
 *    Handles power modes, calibration, wakeup interrupts, and raw readings.
 *  - BMA400: high-level Sensor class for the service framework.
 *    Exposes read(offset), calibration, and event handlers.
 *
 * Power modes:
 *  - SLEEP: lowest power (~0.2 µA), no readings, wakeup via auto-wakeup engine
 *  - LOW_POWER: not directly used — we use NORMAL at 100 Hz for consistent readings
 *  - NORMAL: 100 Hz ODR, used for active reading and calibration
 *
 * Wakeup modes:
 *  - LOW_POWER (power_mode=0): BMA400 auto-wakeup engine with threshold detection
 *  - NORMAL (power_mode=1): GEN1 activity interrupt with configurable threshold/duration
 *
 * FIFO:
 *  - The BMA400 has a 1024-byte hardware FIFO. When enabled, samples accumulate
 *    in the FIFO and can be read in bulk, reducing I2C wakeup frequency.
 *  - See enable_fifo() / read_fifo() for batch reading support.
 */

#include <functional>
#include <cstdint>
#include "sensor.hpp"
#include "nrf_irq.hpp"
#include "calibration.hpp"

extern "C" {
#include "bma400_defs.h"
}

/// @name BMA400 driver constants
/// @{
static constexpr uint8_t BMA400_RW_LENGTH = 64;             ///< Max I2C transfer length
static constexpr unsigned int BMA400_DRDY_MAX_POLLS = 100;  ///< Max DRDY polling attempts (1 ms each)
static constexpr unsigned int BMA400_MAX_DEVICES = 4;       ///< Max simultaneous BMA400 instances
/// @}

/// @brief FIFO sample: raw 12-bit XYZ accelerometer data.
struct BMA400FifoSample {
	int16_t x;
	int16_t y;
	int16_t z;
};

/**
 * @brief Low-level BMA400 I2C driver (wraps Bosch C library).
 */
class BMA400LL {
public:
	/// @param bus         I2C bus index.
	/// @param addr        7-bit I2C address (typically 0x14).
	/// @param wakeup_pin  BSP GPIO enum index for interrupt pin.
	/// @throws ErrorCode::I2C_COMMS_ERROR if device not responding.
	BMA400LL(unsigned int bus, unsigned char addr, int wakeup_pin);
	~BMA400LL();

	/// @name Sensor reading
	/// @{

	/**
	 * @brief Read calibrated XYZ acceleration (g) and temperature.
	 * @param[out] x,y,z        Calibrated acceleration in g-force.
	 * @param[out] temperature   Raw temperature (degrees C × 10).
	 * @note Wakes sensor from SLEEP, reads, then returns to SLEEP.
	 */
	void read_xyz(double &x, double &y, double &z, int16_t &temperature);

	/// @brief Read temperature only.  Briefly enters NORMAL mode then sleeps.
	int16_t read_temperature();
	/// @}

	/// @name Configuration setters
	/// @{
	void set_wakeup_threshold(double threshold);   ///< Wakeup threshold in g-force
	void set_wakeup_duration(double duration);     ///< Wakeup duration in samples
	void set_range(unsigned int g_force);          ///< Range register (BMA400_RANGE_2G/4G/8G/16G)
	void set_power_mode(unsigned int power_mode);  ///< 0=LOW_POWER wakeup, 1=NORMAL wakeup
	/// @}

	/// @name Calibration
	/// @{
	void set_x_calibration(double x);  ///< Set X-axis offset in g (measured at rest)
	void set_y_calibration(double y);  ///< Set Y-axis offset in g
	void set_z_calibration(double z);  ///< Set Z-axis offset in g (typically ~1.0 at rest)
	double get_x_calibration() const { return m_cal_x; }
	double get_y_calibration() const { return m_cal_y; }
	double get_z_calibration() const { return m_cal_z; }

	/**
	 * @brief Auto-calibrate by averaging 200 samples at rest.
	 * @param g_range        Range register value to use during calibration.
	 * @param[out] offset_x  Measured average X in g (target: 0).
	 * @param[out] offset_y  Measured average Y in g (target: 0).
	 * @param[out] offset_z  Measured average Z in g (target: 1.0).
	 */
	void calibrate_offset(uint8_t g_range, double &offset_x, double &offset_y, double &offset_z);
	/// @}

	/// @name Configuration getters
	/// @{
	uint8_t get_power_mode() const { return m_power_mode; }  ///< Current wakeup mode
	uint8_t get_range() const { return m_g_range; }          ///< Current range register
	double get_wakeup_threshold() const { return m_wakeup_threshold; }
	double get_wakeup_duration() const { return m_wakeup_duration; }
	uint8_t range_to_g(uint8_t range_reg) const;  ///< Register value → g-force (2/4/8/16)
	/// @}

	/// @name Wakeup / interrupt
	/// @{

	/// @brief Configure and enable motion wakeup interrupt with callback.
	void enable_wakeup(std::function<void()> func);

	/// @brief Disable wakeup interrupt and return to SLEEP mode.
	void disable_wakeup();

	/// @brief Check if a wakeup IRQ has fired since last check (clears the flag).
	[[nodiscard]] bool check_and_clear_wakeup();
	/// @}

	/// @name FIFO batch reading
	/// @{

	/**
	 * @brief Enable FIFO in stream mode at the given ODR.
	 * @param odr  Output data rate (BMA400_ODR_xxx constant).
	 *
	 * Configures the BMA400 FIFO to collect XYZ samples continuously.
	 * The sensor enters NORMAL mode.  Call read_fifo() periodically
	 * to drain the FIFO before it overflows (1024 bytes ≈ 170 samples).
	 */
	void enable_fifo(uint8_t odr = BMA400_ODR_100HZ);

	/// @brief Disable FIFO and return to sleep mode.
	void disable_fifo();

	/**
	 * @brief Read all available samples from the hardware FIFO.
	 * @param[out] samples     Output array (caller-provided).
	 * @param      max_samples Array capacity.
	 * @return Number of samples actually read (0 if FIFO empty or error).
	 */
	unsigned int read_fifo(BMA400FifoSample *samples, unsigned int max_samples);

	/// @brief Convert raw 12-bit LSB to g-force using the current range setting.
	double lsb_to_g(int16_t raw) const;
	/// @}

private:
	/// @name Hardware state
	/// @{
	unsigned int m_bus;    ///< I2C bus index
	unsigned char m_addr;  ///< 7-bit I2C address
	NrfIRQ m_irq;          ///< GPIOTE interrupt for wakeup pin
	uint8_t m_unique_id;   ///< Device manager lookup ID (for Bosch C callbacks)
	/// @}

	/// @name Bosch driver state
	/// @{
	struct bma400_dev m_bma400_dev;              ///< Bosch driver device handle
	struct bma400_sensor_conf m_sensor_conf[2];  ///< [0]=accel config, [1]=GEN1 interrupt config
	struct bma400_int_enable m_int_enable;       ///< Current interrupt enable state
	bool m_irq_pending;                          ///< Set by ISR, cleared by check_and_clear_wakeup()
	/// @}

	/// @name Configuration (set via calibration_write from service layer)
	/// @{
	uint8_t m_g_range;          ///< Range register (BMA400_RANGE_2G/4G/8G/16G)
	uint8_t m_power_mode;       ///< Wakeup mode: 0=LOW_POWER, 1=NORMAL
	double m_wakeup_threshold;  ///< Motion threshold in g-force
	double m_wakeup_duration;   ///< Motion duration in samples
	/// @}

	/// @name Calibration offsets (g-force, measured at rest)
	/// @{
	double m_cal_x;  ///< X offset (target: 0g at rest)
	double m_cal_y;  ///< Y offset (target: 0g at rest)
	double m_cal_z;  ///< Z offset (target: 1g at rest, gravity)
	/// @}

	bool m_fifo_enabled = false;  ///< True when hardware FIFO is active

	/// @name Power mode helpers
	/// @{
	void init();               ///< I2C probe + soft reset + SLEEP
	void setup_sleep_mode();   ///< Enter SLEEP (~0.2 µA)
	void setup_active_mode();  ///< Enter NORMAL 100 Hz for readings + calibration
	void setup_normal_mode();  ///< Enter NORMAL with GEN1 interrupt config
	/// @}

	/// @name Wakeup mode internals
	/// @{
	void enable_wakeup_low_power(std::function<void()> func);  ///< Auto-wakeup engine (mode 0)
	void enable_wakeup_normal(std::function<void()> func);     ///< GEN1 activity interrupt (mode 1)
	/// @}

	/// @name I2C callbacks for the Bosch C driver (static, use device manager for lookup)
	/// @{
	static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
	static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
	static void delay_us(uint32_t period, void *intf_ptr);
	/// @}

	/// @name Conversion / utility
	/// @{
	double lsb_to_ms2(int16_t accel_data, uint8_t g_range, uint8_t bit_width);  ///< Raw LSB → m/s²
	void check_result(const char *api_name, int8_t rslt);                       ///< Log + throw on Bosch API error
	uint8_t calculate_threshold_reg(double threshold_g, uint8_t acc_range);     ///< g → register value
	                                                                            /// @}
};


/**
 * @brief High-level BMA400 sensor for the service framework.
 *
 * Read offsets:
 *   0: Temperature (degrees C / 10)
 *   1: X axis (g-force, triggers read_xyz)
 *   2: Y axis (g-force, cached)
 *   3: Z axis (g-force, cached)
 *   4: Activity (0-255 computed from magnitude)
 *   5: Wakeup IRQ pending flag
 *
 * Calibration write offsets (SCALW):
 *   0-2: X/Y/Z calibration coefficient (g)
 *   3: Auto-calibrate (X=0, Y=0, Z=1g)
 *   4: Read calibrated X, Y, Z values
 *   5: Read calibration coefficients
 *   6: Save calibration to file
 *   7-10: Threshold, duration, range, power mode
 */
class BMA400 : public Sensor {
public:
	/// @brief Init from config store (range, power mode, FIFO params, calibration).
	BMA400();
	~BMA400() = default;

	/// @brief Read sensor value by channel.  Channel 1 triggers a new XYZ reading.
	/// @param offset  Channel: 0=temp, 1=X (triggers read), 2=Y, 3=Z, 4=activity, 5=IRQ pending.
	/// @return Sensor value for the requested channel.
	double read(unsigned int offset) override;

	/// @brief Write calibration value or trigger action.
	/// @param value   Value to write (interpretation depends on offset).
	/// @param offset  SCALW offset (see table above: 0-2=cal, 3=auto-cal, 6=save, 7-10=config).
	void calibration_write(const double value, const unsigned int offset) override;

	/// @brief Read calibrated value or configuration.
	/// @param[out] value  Read-back value.
	/// @param offset      SCALR offset (see table above: 1-3=cal XYZ, 4-6=coefficients, 7-10=config).
	void calibration_read(double &value, const unsigned int offset) override;

	/// @brief Persist calibration offsets to flash (AXL.CAL file).
	/// @param force  If true, write even if no changes detected.
	void calibration_save(bool force) override;

	/// @brief Register wakeup motion interrupt handler.
	void install_event_handler(unsigned int, std::function<void()>) override;

	/// @brief Unregister wakeup handler and stop FIFO if active.
	void remove_event_handler(unsigned int) override;

	/// @brief Access the low-level driver (e.g. for direct FIFO control).
	BMA400LL &get_ll() { return m_bma400; }

private:
	BMA400LL m_bma400;  ///< Low-level I2C driver
	Calibration m_cal;  ///< Persistent calibration file (AXL.CAL)

	/// @name Cached readings (updated by read(1) or read_fifo_batch())
	/// @{
	double m_last_x = 0;             ///< Last calibrated X in g
	double m_last_y = 0;             ///< Last calibrated Y in g
	double m_last_z = 0;             ///< Last calibrated Z in g
	int16_t m_last_temperature = 0;  ///< Last temperature (°C × 10)
	uint8_t m_last_activity = 0;     ///< Last activity metric (0-255)
	/// @}

	/// @name FIFO batch mode state
	/// @{
	bool m_fifo_enabled = false;            ///< From AXL_FIFO_ENABLE param (default false)
	unsigned int m_fifo_sample_count = 50;  ///< From AXL_FIFO_SAMPLE_COUNT param (1-170)
	bool m_fifo_started = false;            ///< True once enable_fifo() has been called
	/// @}

	/// @brief Calibration file point indices.
	enum class CalibrationPoint : unsigned int { X = 0, Y = 1, Z = 2 };

	/// @brief Load X/Y/Z calibration from persistent file, or use defaults.
	void load_calibration();

	/// @brief Compute activity metric (deviation from 1g) scaled to 0-255.
	double compute_activity();

	/// @brief Read FIFO batch, average, apply calibration, store in m_last_x/y/z.
	void read_fifo_batch();
};
