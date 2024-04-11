# Arduino_Alvik
Arduino Alvik library, code your Alvik

<br> 

To use this library, you need an [Arduino® Alvik](https://store.arduino.cc/pages/alvik) robot.

Once the Arduino® Nano ESP32 of the robot is attached to the computer, select `Arduino Nano ESP32` from Arduino IDE 2 to program it.

<br>

## Color Calibration

Flash `color_calibration` sketch into Arduino Nano ESP32 mounted on Alvik and follow the instructions on serial monitor.

The white and black values will be written into ESP32 EEPROM. 

<br>

## How to update firmware of Arduino Alvik Carrier

Since this [issue](https://github.com/stm32duino/Arduino_Core_STM32/issues/2292), Arduino® Alvik Carrier is not integrated yet into STM32duino's boards.

<br>

At the moment, it is possible to:
- flash `bridge_firmware_updater` example into Arduino Nano ESP32 mounted on Alvik
- use [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) to flash the new firmware.

<br>


In particular these settings are needed:
- UART communication with DTR setted to 1 in STM32CubeProgrammer
- robot must be turned on.

<br>
<br>
<br>

## Useful links
- [arduino-alvik-mpy](https://github.com/arduino/arduino-alvik-mpy): MicroPython library required to program Alvik
- [Arduino_AlvikCarrier](https://github.com/arduino-libraries/Arduino_AlvikCarrier): Arduino library required to build the firmware
- [Arduino Alvik product page](https://store.arduino.cc/pages/alvik)
