This is the code for the PocketBeagle Plant Conditions Monitor, running on python 3.11

You will need to install and perform the following: 

Adafruit libraries sudo pip3 install --upgrade setuptools --> sudo pip3 install --upgrade Adafruit_BBIO --> 
sudo pip3 install adafruit-blinka

Python Files in this Repo:

main.py – Main application logic. Reads sensors, updates the LCD, controls LEDs and relays, handles button input, and manages system states

run.sh – Startup script used by systemd. Configures pins, waits for hardware stabilization, and launches main.py.

configure_pins.sh – Sets PocketBeagle pin modes (GPIO, I2C, LCD) required for the project hardware.

hd44780.py – Driver for the HD44780 16x2 LCD. Handles initialization, commands, and character output.

soil_moist_sensor.py – Interfaces with the soil moisture sensor over I2C and converts raw readings into usable values.

bmp280_sensor.py – Reads temperature (and pressure) data from the BMP280 sensor via I2C.

test_bme280.py – Standalone test script used during sensor debugging and validation.

button.py – Handles button input and debouncing logic.

led.py – Controls red, yellow, and green LED indicators.

gpio_test.py – Verifies GPIO pin functionality during setup.

button_test.py – Test script for validating button wiring and logic.

led_test.py – Test script for checking LED operation.

lcd_test.py – Simple test to verify LCD wiring and character display.

test_pins.py - Confirms correct pin configurations and modes.

simple_calc.py – Utility script used for early logic testing and calculations.
