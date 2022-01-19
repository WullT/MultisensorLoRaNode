# MultisensorLoRaNode
Configurable low-power multisensor node with LoRaWAN connectivity

**[Node Configurator here](https://wullt.github.io/MultisensorLoRaNode/)**

## Hardware

Board: [Adafruit Feather M4 Express](https://www.adafruit.com/product/3857)

LoRa Module: `RFM95W` LoRa Module e.g. [Radio FeatherWing](https://www.adafruit.com/product/3231)
- Pin mapping according to https://github.com/tamberg/fhnw-iot/wiki/FeatherWing-RFM95W

To enter the config_mode: A button that connects GPIO `D4` to `GND` if pressed (or simply a jumper wire)

GPIO `12` is set to `HIGH` 2000 ms before the measurement starts and is set to `LOW` after the measurement. It can be used to turn on and off sensors. If power consumption of the sensors is more than 10 mA, you should use a transistor to switch the load.

Supported Sensors | Link
--|--
SHT31 Air Temperature and Humidity Sensor | [Grove](https://wiki.seeedstudio.com/Grove-TempAndHumi_Sensor-SHT31/), [DFRobot (Weatherproof)](https://wiki.dfrobot.com/SHT31_Temperature_Humidity_Sensor_Weatherproof_SKU_SEN0385)
Capacitive Soil Moisture Sensor | [Grove](https://wiki.seeedstudio.com/Grove-Capacitive_Moisture_Sensor-Corrosion-Resistant/), [DFRobot](https://wiki.dfrobot.com/Waterproof_Capacitive_Soil_Moisture_Sensor_SKU_SEN0308)
DS18B20 OneWire Temperature Sensor | [Grove](https://wiki.seeedstudio.com/One-Wire-Temperature-Sensor-DS18B20/)
Multichannel Gas Sensor | [Grove](https://wiki.seeedstudio.com/Grove-Multichannel-Gas-Sensor-V2/)
UV Sensor | [Grove](https://wiki.seeedstudio.com/Grove-UV_Sensor/)
Analog Turbidity Sensor | [Grove](https://wiki.seeedstudio.com/Grove-Turbidity-Sensor-Meter-for-Arduino-V1.0/)
Sound Sensor | [Grove](https://wiki.seeedstudio.com/Grove-Sound_Sensor/)
Ultrasonic Distance Sensor | [DFRobot](https://wiki.dfrobot.com/_A02YYUW_Waterproof_Ultrasonic_Sensor_SKU_SEN0311)

## Setup Without IDE (UF2 Bootloader)

- Connect the board to your PC
- Double click on the reset button and you should see a new USB Disk drive
- Copy [MultiSensorNode.ino.feather_m4.uf2](UF2/MultiSensorNode.ino.feather_m4.uf2) on the new USB Disk drive
- The new USB Disk Drive will disapear and the application on the board will start

## Configuration

### Register the device in the [TTN console](https://console.thethingsnetwork.org/)
- Create a new application
- Add end device
  - Manual registration
  - Frequency plan `Europe 863-870 MHz (SF9 for RX2)` (for use in Europe)
  - LoRaWAN version `MAC V1.0.3`
  - Select activation method and generate EUIs and Keys
- Go to the Payload formatters tab and select Javascript
  - Paste the content of [decoder.js](TTN/decoder.js) into the field and set the `node_id` to a unique value
- Go to the Location tab and set the location of the node

### Create the config file
- Go to the [Node Configurator](https://wullt.github.io/MultisensorLoRaNode/) or open [index.html](docs/index.html) locally in a webbrowser
- Select the LoRa [Spreading Factor](https://www.thethingsnetwork.org/docs/lorawan/spreading-factors/)
- Uncheck `Use ADR` if you dont want the Spreading Factor to be adapted dynamically ([Adaptive Data Rate](https://www.thethingsnetwork.org/docs/lorawan/adaptive-data-rate/))
- Select the Activation Method [ABP or OTAA](https://www.thethingsnetwork.org/docs/lorawan/end-device-activation/)
- Copy and Paste the credentials directly from the [TTN console](https://console.thethingsnetwork.org/) (**ALL IN MSB FORMAT**)
- Set the Transmit Interval (in seconds)
- Select the connected sensors and the pins / I2C addresses through which they are connected
- Click Generate and Download

### Upload the config file to the board
- Connect the board via USB to your PC
- Connect GPIO `D4` to `GND` (or keep the button pressed if you have one) and press the reset button
- A new USB Disk drive will appear on your PC
- Copy the config file (with the name `configuration.conf`) to the new USB Disk drive
- If the NeoPixel LED on the board turns green, the file was validated
- Remove the connection from GPIO `D4` to `GND` and click the reset button

## Build & Upload using Arduino IDE

Install following libraries:
* [Adafruit_TinyUSB](https://github.com/adafruit/Adafruit_TinyUSB_Arduino)
* [LMIC](https://github.com/mcci-catena/arduino-lmic)
* [Adafruit_SleepyDog](https://github.com/adafruit/Adafruit_SleepyDog)
* [Adafruit_SPIFlash](https://github.com/adafruit/Adafruit_SPIFlash)
* [Adafruit_NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel)
* [Grove_Multichannel_Gas_Sensor V2.0](https://github.com/Seeed-Studio/Seeed_Arduino_MultiGas)
* [OneWire](https://github.com/PaulStoffregen/OneWire)
* [Arduino-Temperature-Control-Library](https://github.com/milesburton/Arduino-Temperature-Control-Library)
* [Adafruit_SHT31](https://github.com/adafruit/Adafruit_SHT31)

Select Board &rarr; *Adafruit Feather M4 Express*

Select Tools &rarr; USB Stack &rarr; *TinyUSB*

- Build
- Upload directly or convert the compiled application to an UF2 file to flash multiple nodes

### Convert binary to UF2

Clone [Microsoft's USB Flashing Format (UF2) Repo](https://github.com/microsoft/uf2)
```sh
git clone https://github.com/microsoft/uf2.git
```

Find the family-id for your board in [utils/uf2families.json](https://github.com/microsoft/uf2/blob/master/utils/uf2families.json) (`0x55114460` for SAMD51)

Find the base_address of the application for your board, `0x4000` for [SAMD51](https://github.com/Microsoft/uf2-samdx1#configuration)

Convert the binary application to uf2:
```sh
cd utils
python uf2conv.py -f FAMILY_ID -b BASE_ADDRESS SOURCE_APP.bin -o OUTPUT_FILE.uf2
```

For Feather M4:
```sh
python uf2conv.py -f 0x55114460 -b 0x4000 MultiSensorNode.ino.feather_m4.bin -o MultiSensorNode.uf2
```



