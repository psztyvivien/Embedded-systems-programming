# Embedded systems programming

This project is for demonstration purposes only! Not licensed for reuse.

The project was developed (on register level only) and tested on https://www.thinkercad.com . 

Challenge: Build an Arduino-based smart sprinkler system that automatically controls irrigation based on soil moisture levels.

Sensors used: the system uses an analogue soil moisture sensor and a digital rain sensor.

Input and display devices: User input is provided by a button and status and settings are managed via a UART-based menu.

## Hardware
- Arduino UNO
- ATmega
- LCD 16 x 2: displays the menu (UART)
    - motor: off/on
    - soil condition
- Pushbutton (3): controlls the UART menu
    - up
    - down
    - menu
- Soil Moisture Sensor
- Relay SPDT: contolls dc motor (start/stop)
- DC Motor: provides the irrigation
- 5 V, 100 mA Solar Cell: powering the whole system
- Voltage Multimeter
- 250 ohm Potentiometer
- 1 k ohm Resistor
- Red LED: turn on when the dc motor is on
- 120 ohm Resistor

  ## 
