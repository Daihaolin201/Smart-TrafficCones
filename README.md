# Smart-TrafficCones
 Third Year Individual Project

# Smart Traffic Cones Third Year Individual Project

## Overview
This project consists of embedded system code for a vehicle module developed on the ESP32 using the Arduino platform and a control module based on the Keil Studio platform.

## Directory Structure

- **Library**: Contains all the third-party libraries used in the project.
- **function test**: Contains independent test codes for various modules.
- **Module functional testing**: Contains codes that test all functionality modules combined.
- **Combined Code**: Contains the final version of the code that integrates all functionalities.

## Development Environment

### Vehicle Module
- **Platform**: Arduino
- **File Format**: `.ino`

### Control Module
- **Platform**: Keil Studio
- **File Format**: `.txt`

## Libraries Used

### Mbed Platform
- **LCD Driver Library** (`C12832.h`)
  - **Source**: [C12832 Class Documentation](https://os.mbed.com/teams/components/code/C12832/docs/tip/classC12832.html#_details)
  - **Installation**: Import from [Mbed Repository](http://os.mbed.com/teams/components/code/C12832/)
  - **Author**: Chris Styles

### Arduino Platform
Libraries can be installed directly via the library manager in the Arduino IDE.

- **Ultrasound Sensor Driver Library** (`SR04.h`)
- **I2C Communication Library** (`Wire.h`)
  - **License**: GNU Lesser General Public License
  - **Additional Info**: Supports various ESP8266 and ESP32 modifications for improved stability and compatibility.
- **OLED Display Driver Library** (`SSD1306`, `Wire.h`)
  - **License**: MIT License
  - **Maintainer**: ThingPulse

## Testing

### function test Folder
- `Encoder_Test.ino` - Tests the ultrasound functionality.
- `OLED_BESIC_HELLO.ino` - Tests OLED display functionality.
- `Board_motorDrive_Code.ino` - Tests motor drive board functionality.
- `ENCODER_PULSE_COUNTER.ino` - Tests the encoder counting functionality.
- `AT_NODE1.txt` and `AT_NODE2.txt` - Tests XBee 802.15.4 point-to-point communication.

### Module functional testing Folder
- `OLED_ENCODER_ULTRA.ino` - Displays encoder count and ultrasound readings on OLED.
- `PULSE_READ_TO_NUCLEO.ino` - Uploads encoder information to the STM32 control module.
- `wireless_control.ino` - Remote control code for the vehicle module using STM32.

### Combined Code Folder
- `ESP32_END.ino` - Final version for the ESP32 vehicle module.
- `STM32_END.txt` - Final version for the STM32 control module.