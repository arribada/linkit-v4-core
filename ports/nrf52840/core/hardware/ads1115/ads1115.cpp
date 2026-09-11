#include <map>

#include "bsp.hpp"
#include "nrf_i2c.hpp"
#include "error.hpp"
#include "pmu.hpp"
#include "debug.hpp"
#include "ads1115.hpp"

ads1115LL::ads1115LL(unsigned int bus, unsigned char addr) : m_bus(bus), m_addr(addr) {

}

void ads1115LL::configure(void)
{
    DEBUG_TRACE("ads1115LL::configure");
    uint8_t write_buffer[3] = {ads1115Command::CONFIG_REG,ads1115Command::MSB_CONFIG,ads1115Command::LSB_CONFIG};
    NrfI2C::write(m_bus, m_addr, write_buffer, 3, false);
    PMU::delay_ms(5);
}

void ads1115LL::read(double& digital_value)
{
    digital_value = (double)sample_adc();;

    digital_value = (digital_value*0.000125); // Convert LSB to V - 1LSB=125uV for this configuration
    DEBUG_TRACE("ads1115LL::read: %f V", digital_value);

    digital_value = 800 +((1060-800)/2.5)*digital_value; // Convert V to hPa
    DEBUG_TRACE("ads1115LL::read: %f hPa", digital_value);
}

uint16_t ads1115LL::sample_adc(void)
{
    DEBUG_TRACE("ads1115LL::sample_adc");
    uint8_t write_buffer = ads1115Command::CONV_REG;
    NrfI2C::write(m_bus, m_addr, &write_buffer, 1, false);
    PMU::delay_ms(50);

    uint8_t read_buffer_b[2] = {0,0};
    NrfI2C::read(m_bus,m_addr,read_buffer_b,2);

    return ((uint16_t)read_buffer_b[0] << 8 ) + ((uint16_t)read_buffer_b[1]);
}

ads1115::ads1115() : Sensor("EXT_BARO"), m_ads1115(ads1115LL(ADS1115_DEVICE, ADS1115_ADDRESS)),m_digital_value(0) {
    DEBUG_TRACE("ads1115::ads1115");
    m_ads1115.configure();
}

double ads1115::read(unsigned int port) {
    DEBUG_TRACE("ads1115::read");
    m_ads1115.read(m_digital_value);
    return m_digital_value;
}
