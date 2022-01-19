// https://fhnw.mit-license.org/

#include "SensorHelper.h"

OneWire oneWire;
DallasTemperature ds18b20;
GAS_GMXXX<TwoWire> gas;
Adafruit_SHT31 sht31 = Adafruit_SHT31();

void initMCGasSensor(uint8_t addr) {
    gas.begin(Wire, addr);
}

bool initSHT31(uint8_t addr, bool heater) {
    long starttime = millis();

    if (sht31.begin(addr)) {
        sht31.heater(heater);
        if (SERIAL_DEBUG) {
            Serial.print("SHT31 Init done, Heater = ");
            Serial.println(heater);
            Serial.print("SHT31 Waiting for data ");
        }
        float t = sht31.readTemperature();

        while (isnan(t)) {
            delay(100);
            t = sht31.readTemperature();
            if (SERIAL_DEBUG) Serial.print(".");
            if (millis() > starttime + DISTANCE_READ_TIMEOUT_MS) {
                if (SERIAL_DEBUG) Serial.println("INIT TIMEOUT!");
                return false;
            }
        }
        if (SERIAL_DEBUG) Serial.println(" ok.");

        return true;
    }
    return false;
}

bool initDS18B20(uint8_t pin) {
    oneWire.begin(pin);
    ds18b20.setOneWire(&oneWire);
    ds18b20.begin();
    uint8_t devices = ds18b20.getDeviceCount();
    if (SERIAL_DEBUG) Serial.printf("DS18B20 Devices: %d\n", devices);

    if (devices > 0) {
        return true;
    }
    return false;
}

bool initDistanceSensor() {
    long starttime = millis();
    Serial1.begin(9600);
    while (!Serial1.available()) {
        if (millis() > starttime + DISTANCE_READ_TIMEOUT_MS) {
            if (SERIAL_DEBUG) Serial.println("INIT TIMEOUT!");
            return false;
        }
    }
    return true;
}

uint16_t getGas(uint8_t gasNr) {
    // https://wiki.seeedstudio.com/Grove-Multichannel-Gas-Sensor-V2/
    uint32_t val = 0;
    switch (gasNr) {
        case GAS_NO2: {
            val = gas.measure_NO2();
            if (SERIAL_DEBUG) Serial.print("GAS_NO2: ");
            break;
        }
        case GAS_C2H5OH: {
            val = gas.measure_C2H5OH();
            if (SERIAL_DEBUG) Serial.print("GAS_C2H5OH: ");
            break;
        }
        case GAS_VOC: {
            val = gas.measure_VOC();
            if (SERIAL_DEBUG) Serial.print("GAS_VOC: ");
            break;
        }
        case GAS_CO: {
            val = gas.measure_CO();
            if (SERIAL_DEBUG) Serial.print("GAS_CO: ");
            break;
        }
        default:
            return 0;
            break;
    }

    if (SERIAL_DEBUG) Serial.println(gas.calcVol(val));
    return (uint16_t)(gas.calcVol(val) * 1000);  // transmit in mV
}

uint8_t getBatteryLevel(uint8_t batteryPin) {
    // https://learn.adafruit.com/adafruit-feather-m4-express-atsamd51/power-management#measuring-battery-3010518-8
    float b = analogRead(batteryPin);
    b *= 2;
    b *= 3.3;
    b /= 1024;
    if (SERIAL_DEBUG) Serial.printf("Battery voltage = %.2f\n", b);
    if (b > 5.1) {
        return 255;
    }
    // voltage divided by 50 to fit in uint8_t
    return (uint8_t)(b * 50);
}

uint16_t measureSoundLevel(uint8_t micPin) {
    // https://wiki.seeedstudio.com/Grove-Sound_Sensor/#play-with-arduino
    long sum = 0;
    for (int i = 0; i < 32; i++) {
        sum += analogRead(micPin);
        delay(2);
    }
    sum /= 32;
    if (SERIAL_DEBUG) Serial.printf("Sound Sensor: %d\n", (uint16_t)sum);

    return (uint16_t)sum;
}

uint16_t measureDistance() {
    //https://wiki.dfrobot.com/_A02YYUW_Waterproof_Ultrasonic_Sensor_SKU_SEN0311
    long starttime = millis();
    unsigned char data[4] = {};
    float distance;
    while (1) {
        do {
            for (int i = 0; i < 4; i++) {
                data[i] = Serial1.read();
            }

        } while (Serial1.read() == 0xff);
        Serial1.flush();

        if (data[0] == 0xff) {
            int sum;
            sum = (data[0] + data[1] + data[2]) & 0x00FF;
            if (sum == data[3]) {
                distance = (data[1] << 8) + data[2];

                if (distance <= 30) {
                    if (SERIAL_DEBUG) Serial.println("Below the lower limit");
                }
                break;
            }
        }
        if (millis() > starttime + DISTANCE_READ_TIMEOUT_MS) {
            if (SERIAL_DEBUG) Serial.println("READING TIMEOUT!");
            distance = 0;
            break;
        }
        delay(10);
    }
    Serial1.end();
    delay(10);

    if (SERIAL_DEBUG) Serial.printf("DISTANCE = %d mm\n", (uint16_t)distance);

    return (uint16_t)distance;
}

uint16_t measureAnalogLevel(uint8_t analogPin) {
    return (uint16_t)analogRead(analogPin);
}

uint16_t getSHTTemp() {
    float t = sht31.readTemperature();

    if (SERIAL_DEBUG) Serial.printf("SHT31 Temperature = %.2f\n", t);
    return (uint16_t)(t * 100);
}
uint16_t getSHTHumi() {
    float h = sht31.readHumidity();
    if (SERIAL_DEBUG) Serial.printf("SHT31 Humidity = %.2f\n", h);

    return (uint16_t)(h * 100);
}

uint16_t getDs18b21Temp() {
    ds18b20.requestTemperatures();

    float tempC = ds18b20.getTempCByIndex(0);
    if (isnan(tempC)) {
        if (SERIAL_DEBUG) Serial.printf("DS18B20 ISNAN: %.2f\n", tempC);
    }
    if (SERIAL_DEBUG) Serial.printf("DS18B20 Temperature = %.2f\n", tempC);
    return (uint16_t)(tempC * 100);
}

int getUVIndex(uint8_t analogPin) {
    // https://wiki.seeedstudio.com/Grove-UV_Sensor/
    int sensorValue;
    long sum = 0;
    for (int i = 0; i < 1024; i++)  // accumulate readings for 1024 times
    {
        sensorValue = analogRead(analogPin);
        sum = sensorValue + sum;
        delay(1);
    }
    float meanVal = sum / 1024.0;  // get mean value
    //Serial.print("The current UV index is:");
    //Serial.print((meanVal * 1000 / 4.3 - 83) / 21);// get a detailed calculating expression for UV index in schematic files.
    //Serial.print("\n");
    //return (int)(((meanVal * 1000 / 4.3 - 83) / 21) * 100);

    float sensorVoltage = meanVal * 3.3 / 1024;  // get voltage

    return (int)((sensorVoltage / 0.1) * 100);
}