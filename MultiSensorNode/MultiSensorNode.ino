// https://fhnw.mit-license.org/

// based on 
// - https://github.com/mcci-catena/arduino-lmic/tree/master/examples
// - https://github.com/adafruit/Adafruit_TinyUSB_Arduino/tree/master/examples

#define DISABLE_PING     // https://github.com/mcci-catena/arduino-lmic#disabling-ping
#define DISABLE_BEACONS  // https://github.com/mcci-catena/arduino-lmic#disabling-beacons
#define LMIC_ENABLE_arbitrary_clock_error 1

#define SERIAL_DEBUG true

#include <Adafruit_TinyUSB.h>

#include <SysCall.h>
#include <sdios.h>
#include <SdFatConfig.h>
#include <SdFat.h>
#include <MinimumSerial.h>
#include <FreeStack.h>
#include <BlockDriver.h>
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>

#include <Adafruit_SleepyDog.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_NeoPixel.h>

#include "NodeConfiguration.h"
#include "SensorHelper.h"

#define VBATPIN A6
#define CONFIG_ENABLE_PIN 4
#define NEOPIXEL_PIN 8
#define VCC_EN_PIN 12
#define SENSOR_STARTUP_DELAY_MS 2000

#define NEOPIXEL_BRIGHTNESS 1



// Configuration
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash flash(&flashTransport);
FatFileSystem fatfs;
Adafruit_USBD_MSC usb_msc;
NodeConfiguration nodeConfig;

bool config_mode = false;
String CONFIG_FILE = "/configuration.conf";
bool fs_changed = false;
bool debug = true;
long last_fs_change = 0;
bool otaa_joined = false;

// LoRa
// Pin mapping, see https://github.com/tamberg/fhnw-iot/wiki/FeatherWing-RFM95W
// Feather M4 Express with FeatherWing RFM95W
const lmic_pinmap lmic_pins = {
    .nss = 5,  // E = CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 6,  // D = RST
    .dio = {
        10,  // B = DIO0 = IRQ
        9,   // C = DIO1
        LMIC_UNUSED_PIN},
};

void os_getArtEui(u1_t* buf) { memcpy(buf, nodeConfig.conf.appeui, 8); }

void os_getDevEui(u1_t* buf) { memcpy(buf, nodeConfig.conf.deveui, 8); }

void os_getDevKey(u1_t* buf) { memcpy(buf, nodeConfig.conf.appkey, 16); }

static uint8_t payload[40];
static osjob_t sendjob;

Adafruit_NeoPixel neoPixel = Adafruit_NeoPixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

uint16_t shttemp = 0;
uint16_t shthumi = 0;
uint16_t moisture = 0;
uint16_t ds18b21temp = 0;
uint16_t gas1 = 0;
uint16_t gas2 = 0;
uint16_t gas3 = 0;
uint16_t gas4 = 0;
uint16_t uv = 0;
uint16_t turbidity = 0;
uint16_t distance = 0;
uint16_t sound = 0;

int payloadSize = 0;
long measurementStart = 0;
int measurementDuration = 0;
int wakeupMillis = 0;
long periodic_debug_msg = 0;

// the setup function runs once when you press reset or power the board
void setup() {
    pinMode(VCC_EN_PIN, OUTPUT);
    digitalWrite(VCC_EN_PIN, LOW);
    if (SERIAL_DEBUG) Serial.begin(115200);

    initNeoPixel();
    initFlash();
    pinMode(CONFIG_ENABLE_PIN, INPUT_PULLUP);
    delay(50);
    if (digitalRead(CONFIG_ENABLE_PIN) == LOW) {
        config_mode = true;
        if (initUsbMsc()) {
            setNeoPixelColor(0, 0, 255);
        } else {
            setNeoPixelColor(255, 0, 0);
        }
    } else {
        delay(3000);
        parseConfigFile();
        if (SERIAL_DEBUG) nodeConfig.printConfiguration();

        // invalid config
        if (!nodeConfig.validate()) {
            int i = 0;
            // blink and print config
            while (1) {
                if (i == 10) {
                    if (SERIAL_DEBUG) nodeConfig.printConfiguration();
                    i = 0;
                }
                setNeoPixelColor(255, 0, 0);
                delay(100);
                setNeoPixelColor(0, 0, 0);
                delay(100);
                i++;
            }
        } else {
            if (nodeConfig.conf.use_otaa) {
                setupLoRaOtaa();
            } else {
                setupLoRaAbp();
            }
            do_send(&sendjob);
        }
    }
}

void loop() {
    if (config_mode) {
        if (fs_changed) {
            if (millis() > last_fs_change + 100) {
                parseConfigFile();
                if (SERIAL_DEBUG) nodeConfig.printConfiguration();
                if (nodeConfig.validate()) {
                    setNeoPixelColor(0, 255, 0);
                } else {
                    setNeoPixelColor(255, 0, 0);
                }
                fs_changed = false;
            }
        }
    } else {
        os_runloop_once();
        if (SERIAL_DEBUG) {
            if (millis() > periodic_debug_msg) {
                Serial.print("Waking up in ");
                Serial.print(wakeupMillis - millis());
                Serial.println(" ms.");
                periodic_debug_msg = millis() + 1000;
            }
        }
    }
}

void updateMeasurement() {
    measurementStart = millis();
    if (SERIAL_DEBUG) Serial.println("update measurement");
    uint8_t sensor_error[MAX_SENSORS + 1];
    digitalWrite(VCC_EN_PIN, HIGH);
    delay(SENSOR_STARTUP_DELAY_MS);

    if (nodeConfig.conf.has_sht31) {
        if (initSHT31(nodeConfig.conf.sht31Address, false)) {
            sensor_error[ID_AIR_TEMP_HUMI] = 0;
        } else {
            sensor_error[ID_AIR_TEMP_HUMI] = 1;
            if (SERIAL_DEBUG) Serial.println("SHT Init failed!");
        }
    }
    if (nodeConfig.conf.has_ds18b20) {
        if (initDS18B20(nodeConfig.conf.ds18b20Pin)) {
            sensor_error[ID_SOIL_TEMP] = 0;
        } else {
            sensor_error[ID_SOIL_TEMP] = 1;
            if (SERIAL_DEBUG) Serial.println("DS18 Init failed!");
        }
    }
    if (nodeConfig.conf.has_mcgas) {
        initMCGasSensor(nodeConfig.conf.mcGasAddress);
    }
    if (nodeConfig.conf.has_distance) {
        if (initDistanceSensor()) {
            sensor_error[ID_DISTANCE] = 0;
        } else {
            if (SERIAL_DEBUG) Serial.println("DISTANCE Init failed!");
            sensor_error[ID_DISTANCE] = 1;
        }
    }
    if (SERIAL_DEBUG) Serial.println("Sensor init done");

    int i = 0;
    payload[0] = getBatteryLevel(VBATPIN);
    i += 1;
    if ((nodeConfig.conf.has_sht31) && (sensor_error[ID_AIR_TEMP_HUMI] == 0)) {
        payload[i] = (uint8_t)ID_AIR_TEMP_HUMI;
        shttemp = getSHTTemp();
        shthumi = getSHTHumi();
        payload[i + 1] = highByte(shttemp);
        payload[i + 2] = lowByte(shttemp);
        payload[i + 3] = highByte(shthumi);
        payload[i + 4] = lowByte(shthumi);
        i += 5;
    }

    if (nodeConfig.conf.has_moisture) {
        payload[i] = (uint8_t)ID_SOIL_MOIST;
        moisture = measureAnalogLevel(nodeConfig.conf.moisturePin);
        payload[i + 1] = highByte(moisture);
        payload[i + 2] = lowByte(moisture);
        i += 3;
    }

    if ((nodeConfig.conf.has_ds18b20) && (sensor_error[ID_SOIL_TEMP] == 0)) {
        payload[i] = (uint8_t)ID_SOIL_TEMP;
        ds18b21temp = getDs18b21Temp();
        payload[i + 1] = highByte(ds18b21temp);
        payload[i + 2] = lowByte(ds18b21temp);
        i += 3;
    }

    if (nodeConfig.conf.has_mcgas) {
        gas1 = getGas(GAS_NO2);
        gas2 = getGas(GAS_C2H5OH);
        gas3 = getGas(GAS_VOC);
        gas4 = getGas(GAS_CO);
        payload[i] = ID_MC_GAS;
        payload[i + 1] = highByte(gas1);
        payload[i + 2] = lowByte(gas1);
        payload[i + 3] = highByte(gas2);
        payload[i + 4] = lowByte(gas2);
        payload[i + 5] = highByte(gas3);
        payload[i + 6] = lowByte(gas3);
        payload[i + 7] = highByte(gas4);
        payload[i + 8] = lowByte(gas4);
        i += 9;
    }
    if (nodeConfig.conf.has_uv) {
        uv = getUVIndex(nodeConfig.conf.uvPin);
        payload[i] = ID_UV;
        payload[i + 1] = highByte(uv);
        payload[i + 2] = lowByte(uv);
        i += 3;
    }
    if (nodeConfig.conf.has_turbidity) {
        turbidity = measureAnalogLevel(nodeConfig.conf.turbidityPin);
        payload[i] = ID_TURBIDITY;
        payload[i + 1] = highByte(turbidity);
        payload[i + 2] = lowByte(turbidity);
        i += 3;
    }
    if ((nodeConfig.conf.has_distance) && (sensor_error[ID_DISTANCE] == 0)) {
        distance = measureDistance();
        payload[i] = ID_DISTANCE;
        payload[i + 1] = highByte(distance);
        payload[i + 2] = lowByte(distance);
        i += 3;
    }
    if (nodeConfig.conf.has_sound) {
        sound = measureSoundLevel(nodeConfig.conf.soundPin);
        payload[i] = ID_SOUND;
        payload[i + 1] = highByte(sound);
        payload[i + 2] = lowByte(sound);
        i += 3;
    }
    payloadSize = i;

    if (SERIAL_DEBUG) printPayload();

    digitalWrite(VCC_EN_PIN, LOW);
}

void printPayload() {
    Serial.print("[ ");
    for (int i = 0; i < payloadSize; i += 2) {
        if (payload[i] < 16) {
            Serial.print("0");
        }
        Serial.print(payload[i], HEX);
        Serial.print(" ");
        if (payload[i + 1] < 16) {
            Serial.print("0");
        }
        Serial.print(payload[i + 1], HEX);
        Serial.print("  ");
    }
    Serial.println("]");
    Serial.println("Payload size = " + String(payloadSize));
}

void do_send(osjob_t* j) {
    if (SERIAL_DEBUG) Serial.println("doSend()");
    if ((LMIC.opmode & OP_TXRXPEND) == 0) {  // No TX/RX pending
        updateMeasurement();
        setNeoPixelColor(0, 0, 255);

        LMIC_setTxData2(1, payload, payloadSize, 0);  // queue TX packet
    }
}

void onEvent(ev_t ev) {
    if (ev == EV_TXCOMPLETE) {
        if (SERIAL_DEBUG) Serial.println("EV_TXCOMPLETE");
        //delay(nodeConfig.conf.txInterval * 1000 - 100);
        measurementDuration = millis() - measurementStart;
        int sleepDuration = nodeConfig.conf.txInterval * 1000 - 200 - measurementDuration;
        wakeupMillis = millis() + sleepDuration;
        if (SERIAL_DEBUG) {
            Serial.print("going to sleep for ");
            Serial.println(sleepDuration);
        }
        delay(100);
        setNeoPixelColor(0, 0, 0);
        if (SERIAL_DEBUG)
            os_setTimedCallback(&sendjob, os_getTime() + ms2osticksRound(sleepDuration), do_send);
        else {
            lowPowerDelay(sleepDuration);
            os_setTimedCallback(&sendjob, os_getTime() + ms2osticksRound(100), do_send);
        }
    }
    if (ev == EV_JOINED) {
        if (SERIAL_DEBUG) Serial.println(F("EV_JOINED"));
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        if (SERIAL_DEBUG) {
            Serial.print("netid: ");
            Serial.println(netid, DEC);
            Serial.print("devaddr: ");
            Serial.println(devaddr, HEX);
        }
        LMIC_setLinkCheckMode(0);
        LMIC_setDrTxpow(12 - nodeConfig.conf.loraSf, 14);
        LMIC_setAdrMode(nodeConfig.conf.use_adr);
    }
    if (ev == EV_JOIN_TXCOMPLETE) {
        if (SERIAL_DEBUG) Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));

    } else {
        if (SERIAL_DEBUG) Serial.printf("Event : %d\n", (unsigned)ev);
    }
}

void parseConfigFile() {
    nodeConfig.clearConfig();
    File dataFile = fatfs.open(CONFIG_FILE, FILE_READ);
    if (dataFile) {
        String line = "";
        bool isComment = false;
        bool newLine = true;
        while (dataFile.available()) {
            char c = dataFile.read();
            // skip spaces and tabs
            if (c == ' ' || c == '\t') {
                continue;
            }
            // check if the line is a comment
            if (newLine) {
                if (c == '#') {
                    isComment = true;
                }
                newLine = false;
            }
            if (!isComment) {
                line += c;
            }
            // end of the line
            if (c == '\n') {
                newLine = true;
                if (!isComment) {
                    nodeConfig.addLine(line);
                    line = "";
                }
                isComment = false;
            }
        }
        if (line != "") {
            nodeConfig.addLine(line);
            line = "";
        }
    } else {
        if (SERIAL_DEBUG) {
            Serial.print("Config File '");
            Serial.print(CONFIG_FILE);
            Serial.println("' not found!");
        }
    }
}

bool initUsbMsc() {
    usb_msc.setID("Mitwelten", "LoRaNode", "1.0");
    usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
    usb_msc.setUnitReady(false);
    usb_msc.begin();
    uint32_t block_count = flash.size();
    usb_msc.setCapacity(block_count / 512, 512);
    usb_msc.setUnitReady(true);
    return true;
}

bool setupLoRaAbp() {
    delay(100);
    os_init();
    LMIC_reset();
    LMIC_setSession(0x13, nodeConfig.conf.devAddr, nodeConfig.conf.nwkskey, nodeConfig.conf.appskey);
#if defined(CFG_eu868)
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);
#else
#error Region not supported
    return false;
#endif
    LMIC_setLinkCheckMode(0);
    LMIC.dn2Dr = DR_SF9;  // TTN RX2 window
    LMIC_setDrTxpow(12 - nodeConfig.conf.loraSf, 14);

    LMIC_setAdrMode(nodeConfig.conf.use_adr);
    return true;
}

void setupLoRaOtaa() {
    LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);
    os_init();
    LMIC_reset();
}

bool initFlash() {
    if (!flash.begin()) {
        if (SERIAL_DEBUG) Serial.println("Flash init failed!");

        return false;
    }
    if (!fatfs.begin(&flash)) {
        if (SERIAL_DEBUG) Serial.println("FATFS init failed!");
        return false;
    }
    if (SERIAL_DEBUG) Serial.println("Flash init succeeded!");

    return true;
}

void initNeoPixel() {
    neoPixel.begin();
    neoPixel.setBrightness(NEOPIXEL_BRIGHTNESS);
    setNeoPixelColor(255, 0, 255);
}

void setNeoPixelColor(uint8_t red, uint8_t green, uint8_t blue) {
    neoPixel.setPixelColor(0, neoPixel.Color(red, green, blue));
    neoPixel.show();
}

void lowPowerDelay(int ms) {
    int dms;
    do {
        dms = Watchdog.sleep(ms);
        ms -= dms;
    } while (ms > 0);
}

int32_t msc_read_cb(uint32_t lba, void* buffer, uint32_t bufsize) {
    (void)bufsize;
    return flash.readBlock(lba, (uint8_t*)buffer) ? 512 : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
int32_t msc_write_cb(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
    (void)bufsize;
    return flash.writeBlock(lba, buffer) ? 512 : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_cb(void) {
    flash.syncBlocks();
    // clear file system's cache to force refresh
    fatfs.cacheClear();
    fs_changed = true;
    last_fs_change = millis();
}