#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
License:
Copyright 2025 Alexis Leyow

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

--------------------------------------------------------------------------
Plant Health Monitor - Main Driver
--------------------------------------------------------------------------

This script ties together:
    * HD44780 character LCD (4-bit parallel)
    * STEMMA I2C Soil Moisture sensor
    * BMP280 Temperature / Pressure sensor (using temp only)
    * Status LEDs (Red / Yellow / Green)
    * Pushbutton to cycle between screens
    * Simple calculation utilities (moisture status / level)

Screen Layout (cycled with button on P2_27):
--------------------------------------------
Screen 0:
    Line 1: "Soil: XX.X%"
    Line 2: "Status: XXXXX"

Screen 1:
    Line 1: "Level: N"
    Line 2: "Status: XXXXX"

Screen 2:
    Line 1: "Plant Monitor"
    Line 2: "Press Btn to next"

Screen 3:
    Line 1: "Temp: XX.XC"
    Line 2: "Soil: XX.X%"

LED meanings:
-------------
    "Soil Wet"   -> Green
    "Optimal"    -> Green + Yellow
    "Dry"        -> Yellow
    "Very Dry!"  -> Red

Hardware pins are documented below and must match your wiring.
"""

import time

from hd44780 import LCD
from soil_moist_sensor import SoilMoistureSensor
from led import LED
from button import Button
from simple_calc import moisture_status, dryness_level
from bmp280_sensor import BMP280


# ------------------------------------------------------------------------
# Hardware Configuration Constants
# ------------------------------------------------------------------------

# I2C configuration for soil moisture sensor
SOIL_I2C_BUS   = 2       # /dev/i2c-2 (P1_26, P1_28)
SOIL_I2C_ADDR  = 0x36    # Default address for STEMMA soil sensor

# LCD pins (matching your wiring)
LCD_RS   = "P2_24"
LCD_E    = "P2_22"
LCD_D4   = "P2_18"
LCD_D5   = "P2_20"
LCD_D6   = "P2_17"
LCD_D7   = "P2_10"

LCD_COLS = 16
LCD_ROWS = 2

RED_LED_PIN    = "P2_19"
YELLOW_LED_PIN = "P2_25"
GREEN_LED_PIN  = "P2_29"

BUTTON_PIN     = "P2_27"

# Loop timing / debounce
UPDATE_PERIOD_SEC   = 0.5      # main loop delay
BUTTON_DEBOUNCE_SEC = 0.15     # 150 ms debounce


# ------------------------------------------------------------------------
# Helper Functions
# ------------------------------------------------------------------------

def update_leds_for_status(status, red_led, yellow_led, green_led):
    """
    Given a status string from moisture_status(), turn the appropriate
    LEDs on/off.

    Status strings expected:
        "Soil Wet"    -> Green
        "Optimal"     -> Green + Yellow
        "Dry"         -> Yellow
        "Very Dry!"   -> Red
    """
    # Turn all LEDs off first
    red_led.off()
    yellow_led.off()
    green_led.off()

    # Then selectively turn on based on status
    if status == "Soil Wet":
        green_led.on()
    elif status == "Optimal":
        green_led.on()
        yellow_led.on()
    elif status == "Dry":
        yellow_led.on()
    else:  # "Very Dry!" or anything else
        red_led.on()


def draw_screen(lcd, screen_index, moisture_pct, status, level, temp_c):
    """
    Draw the requested screen on the LCD.

    screen_index:
        0 -> main soil moisture view
        1 -> dryness level view
        2 -> help / instructions
        3 -> temperature + soil moisture
    """
    lcd.clear()

    if screen_index == 0:
        # Screen 0: numeric moisture + status
        line1 = "Soil: {:4.1f}%".format(moisture_pct)
        line2 = "Status: " + status
        lcd.message(line1[:LCD_COLS])
        lcd.message("\n" + line2[:LCD_COLS])

    elif screen_index == 1:
        # Screen 1: dryness level (0-3) + status
        line1 = "Level: {}".format(level)
        line2 = "Status: " + status
        lcd.message(line1[:LCD_COLS])
        lcd.message("\n" + line2[:LCD_COLS])

    elif screen_index == 2:
        # Screen 2: help + instructions
        lcd.message("Plant Monitor")
        lcd.message("\nPress Btn to next")

    elif screen_index == 3:
        # Screen 3: temperature + soil moisture
        if temp_c is None:
            line1 = "Temp:   ---"
        else:
            line1 = "Temp: {:4.1f}C".format(temp_c)
        line2 = "Soil: {:4.1f}%".format(moisture_pct)
        lcd.message(line1[:LCD_COLS])
        lcd.message("\n" + line2[:LCD_COLS])


# ------------------------------------------------------------------------
# Main Application
# ------------------------------------------------------------------------

def main():
    """
    Main function to initialize devices and enter the monitoring loop.
    Button cycles through screens 0 -> 1 -> 2 -> 3 -> 0 ...
    """

    print("Plant monitor starting main loop...")
    # Initialize soil moisture sensor
    soil_sensor = SoilMoistureSensor(bus=SOIL_I2C_BUS, address=SOIL_I2C_ADDR)

    # Initialize BMP280 temperature/pressure sensor (bus 1, addr 0x76)
    try:
        bmp = BMP280(bus=1, address=0x76)
        bmp_ok = True
    except Exception as e:
        print("Warning: BMP280 init failed:", e)
        bmp = None
        bmp_ok = False

    # Initialize LCD
    lcd = LCD(LCD_RS, LCD_E,
              LCD_D4, LCD_D5, LCD_D6, LCD_D7,
              LCD_COLS, LCD_ROWS)

    # Initialize LEDs
    red_led    = LED(RED_LED_PIN)
    yellow_led = LED(YELLOW_LED_PIN)
    green_led  = LED(GREEN_LED_PIN)

    # Initialize button
    button = Button(BUTTON_PIN)

    # Startup message
    lcd.clear()
    lcd.message("Plant Monitor\nStarting...")
    time.sleep(1.5)

    # Screen cycling state
    current_screen     = 0
    last_button_state  = False   # pressed/unpressed in previous loop
    last_button_change = time.time()

    while True:
        # --------------------------------------------------------------
        # 1. Read soil moisture sensor
        # --------------------------------------------------------------
        try:
            moisture = soil_sensor.read_moisture_percent()
        except Exception as e:
            print("Soil sensor exception:", repr(e))
            lcd.clear()
            lcd.message("Soil Sensor Err")
            lcd.message("\nCheck terminal")
            time.sleep(UPDATE_PERIOD_SEC)
            continue

        # Interpret moisture
        status = moisture_status(moisture)
        level  = dryness_level(moisture)

        # --------------------------------------------------------------
        # 2. Read temperature from BMP280 (if available)
        # --------------------------------------------------------------
        temp_c = None
        if bmp_ok and bmp is not None:
            try:
                temp_c, pressure_hpa = bmp.read()
            except Exception:
                # If we get repeated errors, disable BMP to avoid spamming bus
                bmp_ok = False
                temp_c = None

        # --------------------------------------------------------------
        # 3. Update LEDs (always reflect current soil status)
        # --------------------------------------------------------------
        update_leds_for_status(status, red_led, yellow_led, green_led)

        # --------------------------------------------------------------
        # 4. Button handling (edge + debounce)
        # --------------------------------------------------------------
        now = time.time()
        pressed = button.is_pressed()  # True when physically pressed (active-low)

        # Detect *new* press (was not pressed before, now is pressed)
        if pressed and not last_button_state:
            # Debounce: only accept if enough time since last change
            if (now - last_button_change) >= BUTTON_DEBOUNCE_SEC:
                current_screen = (current_screen + 1) % 4  # 0 -> 1 -> 2 -> 3 -> 0
                last_button_change = now

        last_button_state = pressed

        # --------------------------------------------------------------
        # 5. Draw the current screen
        # --------------------------------------------------------------
        draw_screen(lcd, current_screen, moisture, status, level, temp_c)

        # --------------------------------------------------------------
        # 6. Sleep before next loop
        # --------------------------------------------------------------
        time.sleep(UPDATE_PERIOD_SEC)


# ------------------------------------------------------------------------
# Main Script Entry Point
# ------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting Plant Health Monitor.")