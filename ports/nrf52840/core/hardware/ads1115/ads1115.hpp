#pragma once

#include <functional>
#include <cstdint>
#include <string>
#include "sensor.hpp"
#include "bsp.hpp"

class ads1115LL {
public:
	unsigned int m_bus;
	unsigned char m_addr;
    
	ads1115LL(unsigned int bus, unsigned char addr);
	void configure(void);
	void read(double& digital_value);

private:

    enum ads1115Command : uint8_t {
		CONV_REG = (0x00),
		CONFIG_REG = (0x01),
		MSB_CONFIG = (0XE2), // AIN2-GND - 4.096V - single conversion
		LSB_CONFIG =(0xE3), // 860 SPS
	};

	void send_command(uint8_t command);
	uint16_t sample_adc(void);
};



class ads1115 : public Sensor {
public:
    ads1115(); 
	double read(unsigned int port = 0) override;

private:
	ads1115LL m_ads1115;
	double m_digital_value;
};