#pragma once

#include <EEPROM.h>

struct PersistentData {
    float target_temp = 0.;
    float kP          = 0.;
    float kI          = 0.;
    float kD          = 0.;

    void load() {
        EEPROM.begin(sizeof(PersistentData));
        EEPROM.get(0, *this);
    }

    void save() {
        EEPROM.put(0, *this);
        EEPROM.commit();
    }
};