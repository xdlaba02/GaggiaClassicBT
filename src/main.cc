#include "persistent_data.h"
#include "pid.h"

#include <SPI.h>
#include <MAX6675.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <esp_system.h>

#include <Arduino.h>

#include <sstream>
#include <iomanip>

#define RELAY_OUTPUT 22 
#define RELAY_GROUND 21

#define SERVICE_UUID                "74ab2f66-bd28-11ea-b3de-0242ac130004"
#define CHARACTERISTIC_UUID_KPID    "74ab31b4-bd28-11ea-b3de-0242ac130004"
#define CHARACTERISTIC_UUID_TEMP    "1ef4a71e-c1ce-11ea-b3de-0242ac130004"
#define CHARACTERISTIC_UUID_TARGET  "1ef4a818-c1ce-11ea-b3de-0242ac130004"

#define WATCHDOG_TIMEOUT 3000

#define HEATING_WINDOW 1000

hw_timer_t *watchdog {};
hw_timer_t *pid_loop {};

bool new_temp_ready = true;

double prev_measured_value {};
double temp_input {};
double PID_output {};

double proportional {};
double integral     {};
double derivative  {};

PersistentData persistent_data {};

PID myPID {};

unsigned long heatingWindowStartTime {};

MAX6675 thermocouple(SS);

BLECharacteristic *kPIDCharacteristic   {};
BLECharacteristic *tempCharacteristic   {};
BLECharacteristic *targetCharacteristic {};

bool deviceConnected = false;

class BTServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer *) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer *) {
      deviceConnected = false;
    }
};

class KPIDCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *receiveCharacteristic) {
      std::string msg = receiveCharacteristic->getValue();

      double kP {};
      double kI {};
      double kD {};

      if (std::stringstream(msg) >> kP >> kI >> kD) {
        persistent_data.kP = kP;
        persistent_data.kI = kI;
        persistent_data.kD = kD;
        persistent_data.save();

        myPID.setTunings(persistent_data.kP, persistent_data.kI, persistent_data.kD);
      }

      kPIDCharacteristic->notify();
    }

    void onRead(BLECharacteristic *sendCharacteristic) {
      std::stringstream msg {};
      msg << persistent_data.kP << " " << persistent_data.kI << " " << persistent_data.kD;
      sendCharacteristic->setValue(msg.str());
    }

    void onNotify(BLECharacteristic *notifyCharacteristic) {
      onRead(notifyCharacteristic);
    }
};

class TargetCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *receiveCharacteristic) {
      std::string msg = receiveCharacteristic->getValue();
    
        double val {};
        if (std::stringstream(msg) >> val) {
          if (val >= 0.0 && val <= 150.0) {
            persistent_data.target_temp = val;
            persistent_data.save();
          }
        }

        targetCharacteristic->notify();
    }

    void onRead(BLECharacteristic *sendCharacteristic) {
      std::stringstream msg {};
      msg << persistent_data.target_temp;
      sendCharacteristic->setValue(msg.str());
    }

    void onNotify(BLECharacteristic *notifyCharacteristic) {
      onRead(notifyCharacteristic);
    }
};

class TempCallbacks: public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic *sendCharacteristic) {
      std::stringstream msg {};
      msg << temp_input << " " 
          << proportional / HEATING_WINDOW << " " 
          << integral / HEATING_WINDOW << " " 
          << derivative  / HEATING_WINDOW << " " 
          << PID_output / HEATING_WINDOW << " "
          << millis() / 1000.0;

      sendCharacteristic->setValue(msg.str());
    }
    
    void onNotify(BLECharacteristic *notifyCharacteristic) {
      onRead(notifyCharacteristic);
    }
};

void IRAM_ATTR watchdogTimeout() {
  persistent_data.target_temp = -1.0;
  persistent_data.save();
  esp_restart();
}

void IRAM_ATTR newTempReady() {
  new_temp_ready = true;
}


void setup() {
  pinMode(RELAY_OUTPUT, OUTPUT);
  pinMode(RELAY_GROUND, OUTPUT);
  digitalWrite(RELAY_OUTPUT, LOW);
  digitalWrite(RELAY_GROUND, LOW);

  persistent_data.load();

  myPID.setTunings(persistent_data.kP, persistent_data.kI, persistent_data.kD);
  myPID.setOutputRange(0, HEATING_WINDOW);

  BLEDevice::init("Gaggia Classic BT");

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BTServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);

  kPIDCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_KPID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  kPIDCharacteristic->addDescriptor(new BLE2902());
  kPIDCharacteristic->setCallbacks(new KPIDCallbacks());

  targetCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TARGET, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  targetCharacteristic->addDescriptor(new BLE2902());
  targetCharacteristic->setCallbacks(new TargetCallbacks());

  tempCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TEMP, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  tempCharacteristic->addDescriptor(new BLE2902());
  tempCharacteristic->setCallbacks(new TempCallbacks());

  pService->start();
  pServer->getAdvertising()->start();


  watchdog = timerBegin(0, 80, true);
  timerAttachInterrupt(watchdog, &watchdogTimeout, true);
  timerAlarmWrite(watchdog, WATCHDOG_TIMEOUT * 1000, false);
  timerAlarmEnable(watchdog);

  pid_loop = timerBegin(1, 80, true);
  timerAttachInterrupt(pid_loop, &newTempReady, true);
  timerAlarmWrite(pid_loop, MAX6675_READ_PERIOD * 1000, true);
  timerAlarmEnable(pid_loop);
}

void loop() {
  timerWrite(watchdog, 0);

  if (persistent_data.target_temp < 0.0) {
    digitalWrite(RELAY_OUTPUT, LOW);
  }
  else {
    if (new_temp_ready) {
      double measured_value = thermocouple.readTempC();

      temp_input = (measured_value + prev_measured_value) / 2;
      prev_measured_value = measured_value;
      
      if (std::isnan(temp_input) || (temp_input < 0.0)) {
        temp_input = std::numeric_limits<double>::infinity();
      }

      PID_output = myPID.compute(temp_input, persistent_data.target_temp, proportional, integral, derivative);

      if (deviceConnected) {
        tempCharacteristic->notify();  
      }

      new_temp_ready = false;
    }

    if (PID_output > (millis() % HEATING_WINDOW)) {
      digitalWrite(RELAY_OUTPUT, HIGH);
    }
    else {
      digitalWrite(RELAY_OUTPUT, LOW);
    }
  }
}