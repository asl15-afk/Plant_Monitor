#!/usr/bin/env python3
import time
import Adafruit_BBIO.GPIO as GPIO

"""
PocketBeagle GPIO Pin Test
--------------------------
Tests all pins used in the Plant Monitor project.

Outputs:
  - LCD pins
  - LED pins

Inputs:
  - Button pin

This script will:
  - Configure each pin
  - Toggle outputs HIGH/LOW
  - Report PASS / FAIL per pin
"""

# -------------------------------
# PIN DEFINITIONS (UPDATE IF NEEDED)
# -------------------------------

LCD_PINS = {
    "LCD_RS": "P2_24",
    "LCD_E":  "P2_22",
    "LCD_D4": "P2_18",
    "LCD_D5": "P2_20",
    "LCD_D6": "P2_17",
    "LCD_D7": "P2_10",
}

LED_PINS = {
    "RED_LED":    "P2_19",
    "YELLOW_LED": "P2_25",
    "GREEN_LED":  "P2_29",
}

BUTTON_PIN = "P2_27"   # change to P2_31 if needed

# -------------------------------
# OUTPUT PIN TEST
# -------------------------------

def test_output_pin(name, pin):
    print(f"Testing {name} ({pin}) as OUTPUT...", end=" ")
    try:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(0.2)
        GPIO.output(pin, GPIO.LOW)
        print("OK")
    except Exception as e:
        print("FAIL ->", e)

# -------------------------------
# INPUT PIN TEST
# -------------------------------

def test_input_pin(name, pin):
    print(f"Testing {name} ({pin}) as INPUT...", end=" ")
    try:
        GPIO.setup(pin, GPIO.IN)
        val = GPIO.input(pin)
        print("OK (value =", val, ")")
    except Exception as e:
        print("FAIL ->", e)

# -------------------------------
# MAIN
# -------------------------------

print("\n=== PocketBeagle GPIO Test ===\n")

print("LCD PINS:")
for name, pin in LCD_PINS.items():
    test_output_pin(name, pin)

print("\nLED PINS:")
for name, pin in LED_PINS.items():
    test_output_pin(name, pin)

print("\nBUTTON PIN:")
test_input_pin("BUTTON", BUTTON_PIN)

print("\n=== Test Complete ===\n")

