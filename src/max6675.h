#pragma once

#include <cstdint>

#define MAX6675_THERMOCOUPLE_OPEN -1.0
#define MAX6675_READ_PERIOD 250

class MAX6675 {
	unsigned long m_last_call_time;
	float    	  m_current_temp;
	uint8_t  	  m_pin_SS;
			
public:
	MAX6675(uint8_t SS);
	float readTempC();
};