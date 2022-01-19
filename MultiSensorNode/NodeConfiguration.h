// https://fhnw.mit-license.org/

#include <Arduino.h>
#ifndef SERIAL_DEBUG
#define SERIAL_DEBUG false
#endif  // !SERIAL_DEBUG

enum {
    ID_AIR_TEMP_HUMI = 0x01,
    ID_SOIL_MOIST = 0x02,
    ID_SOIL_TEMP = 0x03,
    ID_UV = 0x04,
    ID_SOUND = 0x05,
    ID_TURBIDITY = 0x06,
    ID_DISTANCE = 0x07,
    ID_MC_GAS = 0x08,
};

struct config {
    String name;
    // LoRa Parameters
    uint32_t txInterval;
    uint8_t loraSf = 7;
    bool use_adr = true;
    bool use_otaa = false;

    // abp
    uint32_t devAddr;
    uint8_t appskey[16];
    uint8_t nwkskey[16];

    // otaa
    uint8_t appeui[8];
    uint8_t deveui[8];
    uint8_t appkey[16];
    bool appeui_valid = false;

    // Sensors
    bool has_sht31 = false;
    uint8_t sht31Address = 0x44;
    bool has_ds18b20 = false;
    uint8_t ds18b20Pin = 2;
    bool has_moisture = false;
    uint8_t moisturePin = 0;
    bool has_uv = false;
    uint8_t uvPin = 0;
    bool has_mcgas = false;
    uint8_t mcGasAddress = 0x08;
    bool has_turbidity = false;
    uint8_t turbidityPin = 0;
    bool has_sound = false;
    uint8_t soundPin = 0;

    bool has_distance = false;
};

uint8_t getPinNumberFromA(uint8_t analogPin);

class NodeConfiguration {
   public:
    NodeConfiguration();
    ~NodeConfiguration();
    void addLine(String line);
    void printConfiguration();
    bool validate();
    void clearConfig();
    config conf;
    bool validateHex(String tovalidate);
};
