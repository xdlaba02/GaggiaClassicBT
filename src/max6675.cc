#include "max6675.h"

#include <SPI.h>

#include <Arduino.h>

#define MAX6675_THERMOCOUPLE_OPEN_BIT 0x04
#define MAX6675_CONVERSION_RATIO 0.25

MAX6675::MAX6675(uint8_t pin_SS) {
	m_pin_SS = pin_SS;
	pinMode(pin_SS, OUTPUT);
	digitalWrite(pin_SS, HIGH);
	SPI.begin();
	m_last_call_time = 0;
}

float MAX6675::readTempC() {
	if (millis() - m_last_call_time >= MAX6675_READ_PERIOD) {
		SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

		digitalWrite(m_pin_SS, LOW);
		uint16_t incoming_message = SPI.transfer16(0x00);		
		digitalWrite(m_pin_SS, HIGH);

		SPI.endTransaction();

		m_last_call_time = millis();
		
		if (incoming_message & MAX6675_THERMOCOUPLE_OPEN_BIT) {
			return MAX6675_THERMOCOUPLE_OPEN;
		}

		m_current_temp = (incoming_message >> 3) * MAX6675_CONVERSION_RATIO;
	}

	return m_current_temp;
}