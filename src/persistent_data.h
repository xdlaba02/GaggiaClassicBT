#ifndef MAIN_H
#define MAIN_H

#include <EEPROM.h>

struct PersistentData {

    double target_temp = 0.;
    double kP          = 0.;
    double kI          = 0.;
    double kD          = 0.;

    void load() {
        EEPROM.begin(sizeof(PersistentData));
        EEPROM.get(0, *this);
    }

    void save() {
        EEPROM.put(0, *this);
        EEPROM.commit();
    }
};

#endif