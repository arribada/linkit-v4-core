/**
 * @file stc3117_gasgauge.cpp
 * @brief STC3117 fuel gauge battery monitor — I2C driver with SOC hysteresis.
 */

#include <cstdint>

#include "stc3117_gasgauge.hpp"
extern "C" {
	#include "Generic_I2C.h"
 	#include "stc311x_BatteryInfo.h"
	#include "stc311x_gasgauge.h"
}
#include "bsp.hpp"
#include "error.hpp"
#include "debug.hpp"
#include "nrf_delay.h"
#include "gpio.hpp"  // For SensorsPowerGuard (VSENSORS management - needed for I2C bus stability)
#include "nrf_gpio.h"
#include "nrf_i2c.hpp"
#include "timer.hpp"

#ifndef CPPUTEST
#include "crc16.h"
#else
#define crc16_compute(x, y, z)  0xFFFF
#endif

/// @name SOC hysteresis thresholds (percentage points)
/// @{
static constexpr uint8_t CRITICAL_SOC_HYSTERESIS = 3;
static constexpr uint8_t LOW_BATT_THRESHOLD      = 5;
/// @}

/// @name Power-optimized timing
/// @{
static constexpr uint32_t STC3117_CACHE_TTL_MS       = 30000;  ///< Skip I2C read if last measurement < 30s ago
static constexpr uint32_t STC3117_STARTUP_WAIT_MS    = 50;     ///< Minimal wait after gauge start
static constexpr uint32_t STC3117_COUNTER_CHECK_MS   = 50;     ///< Poll interval for ready check
static constexpr unsigned int STC3117_MAX_STARTUP_CHECKS = 10; ///< Max ready-check iterations
/// @}

extern Timer *system_timer;

static void GasGauge_DefaultInit(GasGauge_DataTypeDef * GG_struct);

// These filtered values should be retained in noinit RAM so they can survive a reset
static __attribute__((section(".noinit"))) volatile uint16_t m_filtered_values[2];
static __attribute__((section(".noinit"))) volatile uint16_t m_crc;

// Function to register the I2C functions
static void setup_i2c() {
    RegisterI2C_Write(GaugeBatteryMonitor::i2c_write);
    RegisterI2C_Read(GaugeBatteryMonitor::i2c_read);
}


/// @brief Init STC3117: probe I2C, configure gas gauge, start monitoring.
/// @param critical_level  SOC % below which battery is critical.
/// @param low_level       SOC % below which battery is low.
GaugeBatteryMonitor::GaugeBatteryMonitor(uint8_t critical_level,
		uint8_t low_level
		) :
		BatteryMonitor(low_level, critical_level)
{
    DEBUG_INFO("STC3117: Low-power battery monitor initialized");

    // Setup I2C functions but DON'T start the gauge yet (power saving)
    setup_i2c();

    // Set default values until first read (conservative, not 100%)
    m_last_voltage_mv = 3700;
    m_last_level = 50;
    m_is_init = false;  // Will init on first update()
}

/// @brief Configure and start the STC3117 gas gauge driver.
/// @return 0 on success, negative on error.
int GaugeBatteryMonitor::init() {
    SensorsPowerGuard power_guard;  // Acquire VSENSORS for I2C bus stability (other sensors on same bus)
    DEBUG_TRACE("STC3117: Starting gauge for measurement");

    if (m_is_init) {
        return STC3117_OK;
    }

    // Quick device check
    int status = STC31xx_CheckI2cDeviceId();
    if (status != 0) {
        DEBUG_ERROR("STC3117: Device not found (status=%d)", status);
        return status;
    }

    // Config init
    GasGauge_DefaultInit(&STC3117_GG_struct);

    // Start the gas gauge
    status = GasGauge_Start(&STC3117_GG_struct);
    if (status != 0 && status != -2) {
        DEBUG_ERROR("STC3117: Start failed (status=%d)", status);
        m_is_init = false;
        return status;
    }

    // Wait for gauge to be ready (reduced from 500ms + 2s check loop)
    nrf_delay_ms(STC3117_STARTUP_WAIT_MS);

    // Quick ready check
    for (unsigned int i = 0; i < STC3117_MAX_STARTUP_CHECKS; i++) {
        int counter = STC31xx_GetRunningCounter();
        if (counter >= 3) {
            break;  // Ready
        }
        if (counter < 0) {
            DEBUG_ERROR("STC3117: Communication error during startup — stopping gauge");
            // M7 (2026-07): GasGauge_Start already set GG_RUN=1 (~70 uA vs ~3 uA
            // standby). shutdown() is a no-op here because m_is_init is still
            // false, so stop the gauge directly (best-effort) to avoid leaking
            // ~70 uA until the next TPL5111 power cut — a recurring cost on every
            // glitchy wake over a year-long deployment.
            {
                SensorsPowerGuard power_guard;  // VSENSORS for I2C bus stability
                GasGauge_Stop();
            }
            return -1;
        }
        nrf_delay_ms(STC3117_COUNTER_CHECK_MS);
    }

    m_is_init = true;
    DEBUG_TRACE("STC3117: Gauge started successfully");
    return STC3117_OK;
}


/// @brief Shut down the STC3117 to minimise current draw.
/// @return 0 on success, negative on error.
int GaugeBatteryMonitor::shutdown() {
    SensorsPowerGuard power_guard;  // Acquire VSENSORS for I2C bus stability (other sensors on same bus)
    if (!m_is_init) {
        return STC3117_OK;
    }

    DEBUG_TRACE("STC3117: Shutting down gauge for power saving");

    // Put gauge in standby/powerdown mode
    int status = GasGauge_Stop();
    if (status != 0) {
        DEBUG_ERROR("STC3117: Shutdown failed (status=%d)", status);
        return status;
    }

    m_is_init = false;
    return STC3117_OK;
}

// Forwarding C++ I2C functions to the C driver
/// @brief I2C write callback for the ST C driver.
/// @param I2cSlaveAddr   7-bit I2C address.
/// @param RegAddress     Register address.
/// @param TxBuffer       Data to write.
/// @param NumberOfBytes  Number of bytes.
/// @return 0 on success, negative on error.
int GaugeBatteryMonitor::i2c_write(int I2cSlaveAddr, int RegAddress, unsigned char* TxBuffer, int NumberOfBytes)
{
    uint8_t reg_addr = static_cast<uint8_t>(RegAddress);

    if (!NumberOfBytes) {
        return STC3117_OK;
    }

    if (NumberOfBytes > static_cast<int>(STC3117_MAX_BUFFER_LEN + sizeof(reg_addr))) {
        return STC3117_E_OUT_OF_RANGE;
    }

    uint8_t buffer[STC3117_MAX_BUFFER_LEN];
    buffer[0] = reg_addr;
    memcpy(&buffer[1], TxBuffer, NumberOfBytes);

    if (!NrfI2C::write_safe(STC3117_DEVICE, I2cSlaveAddr, buffer, NumberOfBytes + sizeof(reg_addr), false))
        return STC3117_E_COM_FAIL;
    return STC3117_OK;
}

/// @brief I2C read callback for the ST C driver.
/// @param I2cSlaveAddr   7-bit I2C address.
/// @param RegAddress     Register address.
/// @param[out] RxBuffer  Buffer for read data.
/// @param NumberOfBytes  Number of bytes to read.
/// @return 0 on success, negative on error.
int GaugeBatteryMonitor::i2c_read(int I2cSlaveAddr, int RegAddress, unsigned char* RxBuffer, int NumberOfBytes)
{
    uint8_t reg_addr = static_cast<uint8_t>(RegAddress);
    if (!NrfI2C::write_safe(STC3117_DEVICE, I2cSlaveAddr, &reg_addr, sizeof(reg_addr), true))
        return STC3117_E_COM_FAIL;
    if (!NrfI2C::read_safe(STC3117_DEVICE, I2cSlaveAddr, RxBuffer, NumberOfBytes))
        return STC3117_E_COM_FAIL;
    return STC3117_OK;
}

/// @brief Check STC3117 responds on I2C by reading a known register.
/// @return 0 on success, -1 if device not found.
int GaugeBatteryMonitor::check_i2c_device() {
    SensorsPowerGuard power_guard;  // Acquire VSENSORS for I2C bus stability (other sensors on same bus)
    int status = STC31xx_CheckI2cDeviceId();
    if (status != 0) {
        DEBUG_ERROR("STC3117: I2C device check failed (status=%d)", status);
    }
    return status;
}

// Voltage-based SOC estimate using 4V20_MAX LiPo OCV curve (420mAh 3.7V)
// Used as sanity check when STC3117 reports implausible SOC after power cycle
static uint8_t estimate_soc_from_voltage(uint16_t mv) {
    // OCV table: voltage (mV) → SOC (%) for typical 4.2V LiPo
    static const struct { uint16_t mv; uint8_t soc; } ocv[] = {
        {3300,  0}, {3541,  3}, {3618,  6}, {3658, 10},
        {3695, 15}, {3721, 20}, {3747, 25}, {3761, 30},
        {3778, 40}, {3802, 50}, {3863, 60}, {3899, 65},
        {3929, 70}, {3991, 80}, {4076, 90}, {4176,100},
    };
    if (mv <= ocv[0].mv) return 0;
    if (mv >= ocv[15].mv) return 100;
    for (int i = 1; i < 16; i++) {
        if (mv <= ocv[i].mv) {
            uint32_t range_mv = ocv[i].mv - ocv[i-1].mv;
            uint32_t range_soc = ocv[i].soc - ocv[i-1].soc;
            return ocv[i-1].soc + static_cast<uint8_t>((mv - ocv[i-1].mv) * range_soc / range_mv);
        }
    }
    return 100;
}

/// @brief Periodic update: read STC3117 via gas gauge task, apply SOC hysteresis, persist to .noinit RAM.
void GaugeBatteryMonitor::internal_update() {
    // Cache: skip I2C read if last measurement is still fresh
    static uint64_t last_read_time = 0;
    uint64_t now = system_timer->get_counter();
    if (last_read_time > 0 && (now - last_read_time) < STC3117_CACHE_TTL_MS) {
        return;  // Use cached values
    }

    // Acquire VSENSORS for I2C bus stability (other sensors on same bus)
    SensorsPowerGuard power_guard;

    // Power-optimized: init → read → shutdown cycle.
    // STC3117 runs at ~70uA vs ~3uA standby — must minimize run time for long deployment.

    // 1. Initialize gauge (wake from standby)
    int status = this->init();
    if (status != STC3117_OK) {
        DEBUG_ERROR("STC3117: Failed to init for reading — assuming LOW battery (fail-safe)");
        // H3 (2026-07): a dead/erroring gauge must NOT masquerade as nominal.
        // The old behavior kept the optimistic default (~50%), so a genuinely
        // near-empty tag with a flaky gauge ran full GNSS+TX to hardware
        // brown-out. Degrade to low-battery (Doppler) mode — conservative — but
        // never assert CRITICAL on a read failure alone (that would power the
        // tag down on every wake on a transient I2C glitch). A real critical
        // cell is still caught the moment the gauge reads successfully.
        m_is_low_level = true;
        m_is_critical_voltage = false;
        return;
    }

    // 2. Read battery data
    uint16_t mv = 0;
    uint8_t level = 0;

    status = GasGauge_Task(&STC3117_GG_struct);

    if (status > 0) {
        // New data available
        mv = static_cast<uint16_t>(STC3117_GG_struct.Voltage);
        level = static_cast<uint8_t>(STC3117_GG_struct.SOC / 10);

        // Sanity check: if STC3117 reports implausible SOC, override with
        // voltage-based estimate using the OCV curve (4V20_MAX LiPo profile)
        uint8_t volt_soc = estimate_soc_from_voltage(mv);
        if (level > volt_soc + 15) {
            DEBUG_WARN("STC3117: SOC %u%% implausible for %umV, using voltage estimate %u%%",
                       level, mv, volt_soc);
            level = volt_soc;
        }

        DEBUG_INFO("STC3117: V=%umV SOC=%u%% I=%imA",
            mv, level, STC3117_GG_struct.Current);
    }
    else if (status == 0) {
        mv = static_cast<uint16_t>(STC3117_GG_struct.Voltage);
        level = static_cast<uint8_t>(STC3117_GG_struct.SOC / 10);
    }
    else {
        DEBUG_ERROR("STC3117: Read error (status=%d) — assuming LOW battery (fail-safe)", status);
        // H3: see the init-fail path above — degrade to low-battery mode,
        // never brick (critical powerdown) on a transient read error.
        m_is_low_level = true;
        m_is_critical_voltage = false;
        this->shutdown();
        return;
    }

    // 3. Shutdown immediately for power saving
    this->shutdown();

    // Update cache timestamp
    last_read_time = system_timer->get_counter();

    // 4. Apply filtering to prevent value bouncing
    uint16_t crc = crc16_compute(reinterpret_cast<const uint8_t *>(const_cast<const uint16_t *>(m_filtered_values)), sizeof(m_filtered_values), nullptr);
    if (crc == m_crc) {
        // Previously filtered values are valid - voltage (no hysteresis needed, just store)
        m_filtered_values[0] = mv;
        // SOC critical hysteresis
        if (m_filtered_values[1] < m_critical_level) {
            if (level >= (m_critical_level + CRITICAL_SOC_HYSTERESIS))
                m_filtered_values[1] = level;
        } else if (m_filtered_values[1] < m_low_level) {
            if (level >= (m_low_level + LOW_BATT_THRESHOLD))
                m_filtered_values[1] = level;
        } else {
            m_filtered_values[1] = level;
        }
    } else {
        // No previous values - set new values
        m_filtered_values[0] = mv;
        m_filtered_values[1] = level;
    }

    // Update CRC in noinit RAM
    m_crc = crc16_compute(reinterpret_cast<const uint8_t *>(const_cast<const uint16_t *>(m_filtered_values)), sizeof(m_filtered_values), nullptr);

    // Apply new values
    m_last_voltage_mv = mv;
    m_last_level = level;

    // Set flags. CRITICAL requires BOTH the fuel-gauge SOC AND the OCV voltage
    // estimate to be below critical (H3, 2026-07). A gauge glitch (unsettled OCV
    // at cold power-up, PORDET/BATFAIL) can report a low SOC on a HEALTHY cell;
    // the old SOC-only critical check then powered the tag down on every wake,
    // silently bricking the mission (recoverable only with a magnet). The
    // voltage cross-check vetoes that false-critical while still catching a
    // genuinely empty cell (both indicators agree). Low-battery (Doppler) mode
    // stays SOC-only — a false "low" only costs one Doppler wake, never a brick.
    uint8_t volt_soc_crit = estimate_soc_from_voltage(m_filtered_values[0]);
    m_is_critical_voltage = (m_filtered_values[1] < m_critical_level) &&
                            (volt_soc_crit < m_critical_level);
    m_is_low_level = m_filtered_values[1] < m_low_level;
}

static void GasGauge_DefaultInit(GasGauge_DataTypeDef * GG_struct)
{
    int Rint;

    // Structure initialization
    GG_struct->Cnom = BATT_CAPACITY;        // Nominal battery capacity in mAh
    GG_struct->Vmode = MONITORING_MODE;     // 1=Voltage mode, 0=mixed mode

    Rint = BATT_RINT;
    if (Rint == 0) Rint = 200;  // Force default

    GG_struct->VM_cnf = static_cast<int>((Rint * BATT_CAPACITY) / 977.78);

    if (MONITORING_MODE == MIXED_MODE) {
        GG_struct->CC_cnf = static_cast<int>((RSENSE * BATT_CAPACITY) / 49.556);
    }

    // SOC curve adjustment
    GG_struct->SoctabValue[0] = 0;
    GG_struct->SoctabValue[1] = 3 * 2;
    GG_struct->SoctabValue[2] = 6 * 2;
    GG_struct->SoctabValue[3] = 10 * 2;
    GG_struct->SoctabValue[4] = 15 * 2;
    GG_struct->SoctabValue[5] = 20 * 2;
    GG_struct->SoctabValue[6] = 25 * 2;
    GG_struct->SoctabValue[7] = 30 * 2;
    GG_struct->SoctabValue[8] = 40 * 2;
    GG_struct->SoctabValue[9] = 50 * 2;
    GG_struct->SoctabValue[10] = 60 * 2;
    GG_struct->SoctabValue[11] = 65 * 2;
    GG_struct->SoctabValue[12] = 70 * 2;
    GG_struct->SoctabValue[13] = 80 * 2;
    GG_struct->SoctabValue[14] = 90 * 2;
    GG_struct->SoctabValue[15] = 100 * 2;

#ifdef DEFAULT_BATTERY_4V20_MAX
    GG_struct->OcvValue[0] = 0x1770;
    GG_struct->OcvValue[1] = 0x1926;
    GG_struct->OcvValue[2] = 0x19B2;
    GG_struct->OcvValue[3] = 0x19FB;
    GG_struct->OcvValue[4] = 0x1A3E;
    GG_struct->OcvValue[5] = 0x1A6D;
    GG_struct->OcvValue[6] = 0x1A9D;
    GG_struct->OcvValue[7] = 0x1AB6;
    GG_struct->OcvValue[8] = 0x1AD5;
    GG_struct->OcvValue[9] = 0x1B01;
    GG_struct->OcvValue[10] = 0x1B70;
    GG_struct->OcvValue[11] = 0x1BB1;
    GG_struct->OcvValue[12] = 0x1BE8;
    GG_struct->OcvValue[13] = 0x1C58;
    GG_struct->OcvValue[14] = 0x1CF3;
    GG_struct->OcvValue[15] = 0x1DA9;
#endif

#ifdef DEFAULT_BATTERY_4V35_MAX
    GG_struct->OcvValue[0] = 0x1770;
    GG_struct->OcvValue[1] = 0x195D;
    GG_struct->OcvValue[2] = 0x19EE;
    GG_struct->OcvValue[3] = 0x1A1A;
    GG_struct->OcvValue[4] = 0x1A59;
    GG_struct->OcvValue[5] = 0x1A95;
    GG_struct->OcvValue[6] = 0x1AB6;
    GG_struct->OcvValue[7] = 0x1AC7;
    GG_struct->OcvValue[8] = 0x1AEB;
    GG_struct->OcvValue[9] = 0x1B2B;
    GG_struct->OcvValue[10] = 0x1BCC;
    GG_struct->OcvValue[11] = 0x1C13;
    GG_struct->OcvValue[12] = 0x1C57;
    GG_struct->OcvValue[13] = 0x1D09;
    GG_struct->OcvValue[14] = 0x1DCF;
    GG_struct->OcvValue[15] = 0x1EA2;
#endif

    // Capacity derating by temperature
    GG_struct->CapDerating[6] = 0;  // -20°C
    GG_struct->CapDerating[5] = 0;  // -10°C
    GG_struct->CapDerating[4] = 0;  // 0°C
    GG_struct->CapDerating[3] = 0;  // 10°C
    GG_struct->CapDerating[2] = 0;  // 25°C
    GG_struct->CapDerating[1] = 0;  // 40°C
    GG_struct->CapDerating[0] = 0;  // 60°C

    // Alarm thresholds
    GG_struct->Alm_SOC = 1;         // SOC alarm level %
    GG_struct->Alm_Vbat = 3300;     // Vbat alarm level mV

    // Hardware configuration
    GG_struct->Rsense = RSENSE;
    GG_struct->RelaxCurrent = GG_struct->Cnom / 20;  // Relaxation current < C/20 mA
    GG_struct->ForceExternalTemperature = 0;         // Use internal temperature
}
