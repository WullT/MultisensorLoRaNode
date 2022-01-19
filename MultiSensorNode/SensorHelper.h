// https://fhnw.mit-license.org/

#include <Arduino.h>
#include <Multichannel_Gas_GMXXX.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_SHT31.h>

#define DISTANCE_READ_TIMEOUT_MS 3000
#define MAX_SENSORS 8

#ifndef SERIAL_DEBUG
#define SERIAL_DEBUG false
#endif  // !SERIAL_DEBUG

enum {
    GAS_NO2 = 0,
    GAS_C2H5OH = 1,
    GAS_VOC = 2,
    GAS_CO = 3,
};

void initMCGasSensor(uint8_t addr);

bool initSHT31(uint8_t addr, bool heater = true);

bool initDS18B20(uint8_t pin);

bool initDistanceSensor();

uint8_t getBatteryLevel(uint8_t batteryPin);

uint16_t measureSoundLevel(uint8_t micPin);

uint16_t measureDistance();

uint16_t measureAnalogLevel(uint8_t analogPin);

uint16_t getGas(uint8_t gasNr);

uint16_t getSHTTemp();
uint16_t getSHTHumi();

uint16_t getDs18b21Temp();

int getUVIndex(uint8_t analogPin);
