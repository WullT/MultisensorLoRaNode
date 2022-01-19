// https://fhnw.mit-license.org/

#include "NodeConfiguration.h"

uint8_t getPinNumberFromA(uint8_t analogPin) {
    uint8_t pinNr = 0;
    switch (analogPin) {
        case 0: {
            pinNr = A0;
            break;
        }
        case 1: {
            pinNr = A1;
            break;
        }
        case 2: {
            pinNr = A2;
            break;
        }
        case 3: {
            pinNr = A3;
            break;
        }
        case 4: {
            pinNr = A4;
            break;
        }
        case 5: {
            pinNr = A5;
            break;
        }
        case 6: {
            pinNr = A6;
            break;
        }
        default:
            break;
    }
    return pinNr;
}

NodeConfiguration::NodeConfiguration() {}
NodeConfiguration::~NodeConfiguration() {}

void NodeConfiguration::addLine(String line) {
    int term_index = line.indexOf('\n');
    if (term_index > 0) {
        line = line.substring(0, line.indexOf('\n'));
    }
    int index = line.indexOf("=");
    if (index > 0) {
        String key = line.substring(0, index);
        line = line.substring(index + 1);
        if (key == "name") {
            conf.name = line.c_str();
        }
        if (key == "tx_interval") {
            conf.txInterval = line.toInt();
        }
        if (key == "activation") {
            if (line.indexOf("otaa") >= 0) {
                conf.use_otaa = true;
            } else {
                conf.use_otaa = false;
            }
        }
        // abp
        if (key == "device_address") {
            conf.devAddr = strtol(line.c_str(), NULL, 16);
        }
        if (key == "nwkskey") {
            for (int i = 0; i < 16; ++i) {
                conf.nwkskey[i] = strtol(line.substring(2 * i, 2 * i + 2).c_str(), NULL, 16);
            }
        }
        if (key == "appskey") {
            for (int i = 0; i < 16; ++i) {
                conf.appskey[i] = strtol(line.substring(2 * i, 2 * i + 2).c_str(), NULL, 16);
            }
        }

        // otaa
        // MSB2LSB!
        if (key == "appeui") {
            if ((line.length() == 16) && validateHex(line)) {
                for (int i = 0; i < 8; ++i) {
                    conf.appeui[7 - i] = strtol(line.substring(2 * i, 2 * i + 2).c_str(), NULL, 16);
                }
                conf.appeui_valid = true;
            } else {
                if (SERIAL_DEBUG) Serial.println("appeui wrong");
            }
        }
        // MSB2LSB!
        if (key == "deveui") {
            if ((line.length() == 16) && validateHex(line)) {
                for (int i = 0; i < 8; ++i) {
                    conf.deveui[7 - i] = strtol(line.substring(2 * i, 2 * i + 2).c_str(), NULL, 16);
                }
            } else {
                if (SERIAL_DEBUG) Serial.println("deveui wrong");
            }
        }
        if (key == "appkey") {
            if ((line.length() == 32) && validateHex(line)) {
                for (int i = 0; i < 16; ++i) {
                    conf.appkey[i] = strtol(line.substring(2 * i, 2 * i + 2).c_str(), NULL, 16);
                }
            } else {
                if (SERIAL_DEBUG) Serial.println("appkey wrong");
            }
        }

        if (key == "lora_sf") {
            conf.loraSf = line.toInt();
        }

        if (key == "use_adr") {
            if (line.indexOf("true") >= 0 || line.indexOf("1") >= 0 || line.indexOf("t") >= 0) {
                conf.use_adr = true;
            } else {
                conf.use_adr = false;
            }
        }

        // Sensors

        if (key == "has_sht31") {
            if (line.indexOf("true") >= 0 || line.indexOf("1") >= 0 || line.indexOf("t") >= 0) {
                conf.has_sht31 = true;
            } else {
                conf.has_sht31 = false;
            }
        }
        if (key == "has_moisture_sensor") {
            if (line.indexOf("true") >= 0 || line.indexOf("1") >= 0 || line.indexOf("t") >= 0) {
                conf.has_moisture = true;
            } else {
                conf.has_moisture = false;
            }
        }
        if (key == "has_ds18b20") {
            if (line.indexOf("true") >= 0 || line.indexOf("1") >= 0 || line.indexOf("t") >= 0) {
                conf.has_ds18b20 = true;
            } else {
                conf.has_ds18b20 = false;
            }
        }
        if (key == "has_mc_gas_sensor") {
            if (line.indexOf("true") >= 0 || line.indexOf("1") >= 0 || line.indexOf("t") >= 0) {
                conf.has_mcgas = true;
            } else {
                conf.has_mcgas = false;
            }
        }
        if (key == "has_uv_sensor") {
            if (line.indexOf("true") >= 0 || line.indexOf("1") >= 0 || line.indexOf("t") >= 0) {
                conf.has_uv = true;
            } else {
                conf.has_uv = false;
            }
        }
        if (key == "has_turbidity_sensor") {
            if (line.indexOf("true") >= 0 || line.indexOf("1") >= 0 || line.indexOf("t") >= 0) {
                conf.has_turbidity = true;
            } else {
                conf.has_turbidity = false;
            }
        }
        if (key == "has_sound_sensor") {
            if (line.indexOf("true") >= 0 || line.indexOf("1") >= 0 || line.indexOf("t") >= 0) {
                conf.has_sound = true;
            } else {
                conf.has_sound = false;
            }
        }
        if (key == "has_distance_sensor") {
            if (line.indexOf("true") >= 0 || line.indexOf("1") >= 0 || line.indexOf("t") >= 0) {
                conf.has_distance = true;
            } else {
                conf.has_distance = false;
            }
        }

        if (key == "sht31_i2c_addr") {
            conf.sht31Address = strtol(line.c_str(), NULL, 16);
        }

        if (key == "cap_moist_pin") {
            if (line.startsWith("A")) {
                uint8_t analogPin = line.substring(line.indexOf("A") + 1).toInt();
                conf.moisturePin = getPinNumberFromA(analogPin);

            } else {
                conf.moisturePin = line.toInt();
            }
        }

        if (key == "ds18b20_pin") {
            if (line.startsWith("A")) {
                uint8_t analogPin = line.substring(line.indexOf("A") + 1).toInt();
                conf.ds18b20Pin = getPinNumberFromA(analogPin);

            } else {
                conf.ds18b20Pin = line.toInt();
            }
        }

        if (key == "mc_gas_i2c_addr") {
            conf.mcGasAddress = strtol(line.c_str(), NULL, 16);
        }
        if (key == "uv_sensor_pin") {
            if (line.startsWith("A")) {
                uint8_t analogPin = line.substring(line.indexOf("A") + 1).toInt();
                conf.uvPin = getPinNumberFromA(analogPin);

            } else {
                conf.uvPin = line.toInt();
            }
        }

        if (key == "turbidity_sensor_pin") {
            if (line.startsWith("A")) {
                uint8_t analogPin = line.substring(line.indexOf("A") + 1).toInt();
                conf.turbidityPin = getPinNumberFromA(analogPin);
            } else {
                conf.turbidityPin = line.toInt();
            }
        }

        if (key == "sound_sensor_pin") {
            if (line.startsWith("A")) {
                uint8_t analogPin = line.substring(line.indexOf("A") + 1).toInt();
                conf.soundPin = getPinNumberFromA(analogPin);
            } else {
                conf.soundPin = line.toInt();
            }
        }
    }
}

void NodeConfiguration::printConfiguration() {
    Serial.println("############################################################################################");
    Serial.println("#                                     Configuration                                         ");
    Serial.println("# [LoRa]");
    Serial.print("#   Activation: ");
    if (conf.use_otaa) {
        Serial.println("otaa");
        Serial.print("#   appeui: ");
        for (int i = 0; i < 8; ++i) {
            Serial.printf("0x%02X ", conf.appeui[i]);
        }
        Serial.println();
        Serial.print("#   deveui: ");
        for (int i = 0; i < 8; ++i) {
            Serial.printf("0x%02X ", conf.deveui[i]);
        }
        Serial.println();
        Serial.print("#   appkey: ");
        for (int i = 0; i < 16; ++i) {
            Serial.printf("0x%02X ", conf.appkey[i]);
        }
        Serial.println();
    } else {
        Serial.println("abp");
        Serial.printf("#   DeviceAddress: % 04X\n", conf.devAddr);
        Serial.print("#   nwkskey: ");
        for (int i = 0; i < 16; ++i) {
            Serial.printf("0x%02X ", conf.nwkskey[i]);
        }
        Serial.println();
        Serial.print("#   appskey: ");
        for (int i = 0; i < 16; ++i) {
            Serial.printf("0x%02X ", conf.appskey[i]);
        }
        Serial.println();
    }

    Serial.print("#   TX interval: ");
    Serial.printf("%d s =~ %.2f m\n", conf.txInterval, float(conf.txInterval / 60.0));
    Serial.print("#   LoRa SF: ");
    Serial.printf("%d\n", conf.loraSf);
    Serial.print("#   ADR: ");
    conf.use_adr ? Serial.println("True") : Serial.println("False");
    Serial.println("# [Sensors]");
    if (conf.has_sht31) {
        Serial.print("#   SHT31 Temperature+Humidity Sensor, Address= ");
        Serial.printf("0x% 02X\n", conf.sht31Address);
    }

    if (conf.has_moisture) {
        Serial.print("#   Capacitive Moisture Sensor, Pin= ");
        Serial.printf("%d\n", conf.moisturePin);
    }
    if (conf.has_ds18b20) {
        Serial.print("#   DS18B20 Temperature Sensor, Pin= ");
        Serial.printf("%d\n", conf.ds18b20Pin);
    }
    if (conf.has_mcgas) {
        Serial.print("#   Multichannel Gas Sensor, Address= ");
        Serial.printf("0x% 02X\n", conf.mcGasAddress);
    }
    if (conf.has_uv) {
        Serial.print("#   UV Sensor, Pin= ");
        Serial.printf("%d\n", conf.uvPin);
    }
    if (conf.has_turbidity) {
        Serial.print("#   Turbidity Sensor, Pin= ");
        Serial.printf("%d\n", conf.turbidityPin);
    }
    if (conf.has_distance) {
        Serial.print("#   Distance Sensor, Serial Port=  ");
        Serial.println("Serial1");
    }
    if (conf.has_sound) {
        Serial.print("#   Sound Sensor, Pin= ");
        Serial.printf("%d\n", conf.soundPin);
    }
    Serial.println("#");

    Serial.println("############################################################################################");
}

bool NodeConfiguration::validate() {
    // check LoRa Parameters
    if (conf.use_otaa) {
        if (conf.deveui[0] == 0x00) {
            return false;
        }
        if (conf.appkey[0] == 0x00) {
            return false;
        }
        if (!conf.appeui_valid) {
            return false;
        }
    } else {
        if (conf.appskey[0] == 0x00) {
            return false;
        }
        if (conf.nwkskey[0] == 0x00) {
            return false;
        }
        if (conf.devAddr == 0x00) {
            return false;
        }
    }
    if (conf.txInterval == 0x00) {
        return false;
    }
    return true;
}

void NodeConfiguration::clearConfig() {
    conf = config();
}

bool NodeConfiguration::validateHex(String tovalidate) {
    for (int i = 0; i < tovalidate.length(); ++i) {
        if (!isHexadecimalDigit(tovalidate.charAt(i))) {
            return false;
        }
    }
    return true;
}