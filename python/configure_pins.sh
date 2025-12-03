#!/bin/bash
# -------------------------------------------------------------------------
# configurepin.sh
# Plant Health Monitor - PocketBeagle Pinmux Setup
# -------------------------------------------------------------------------
# This script sets the pinmux modes so the Plant Monitor hardware works:
#   - LCD (HD44780 4-bit) GPIO outputs
#   - LEDs GPIO outputs
#   - Button GPIO input
#   - I2C2 for soil sensor (P1_26/P1_28)
#   - I2C1 for BME/BMP sensor (P2_09/P2_11)
#
# Run:
#   sudo ./configurepin.sh
# -------------------------------------------------------------------------

set -e

echo "Configuring PocketBeagle pins for Plant Monitor..."

# -----------------------
# LCD (HD44780) GPIO pins
# -----------------------
config-pin P2_24 gpio   # LCD_RS
config-pin P2_22 gpio   # LCD_E
config-pin P2_18 gpio   # LCD_D4
config-pin P2_20 gpio   # LCD_D5
config-pin P2_17 gpio   # LCD_D6
config-pin P2_10 gpio   # LCD_D7

# ---------------
# LEDs GPIO outputs
# ---------------
config-pin P2_19 gpio   # RED_LED
config-pin P2_25 gpio   # YELLOW_LED
config-pin P2_29 gpio   # GREEN_LED

# ----------------
# Button GPIO input
# ----------------
config-pin P2_27 gpio   # BUTTON (active-low)

# -----------------------
# I2C buses (sensors)
# -----------------------
# Soil moisture sensor on I2C2
config-pin P1_26 i2c    # I2C2_SDA
config-pin P1_28 i2c    # I2C2_SCL

# BME/BMP280 sensor on I2C1
config-pin P2_09 i2c    # I2C1_SDA
config-pin P2_11 i2c    # I2C1_SCL

echo "Done."
echo ""
echo "Quick check (modes):"
config-pin -q P2_24
config-pin -q P2_22
config-pin -q P2_18
config-pin -q P2_20
config-pin -q P2_17
config-pin -q P2_10
config-pin -q P2_19
config-pin -q P2_25
config-pin -q P2_29
config-pin -q P2_27
config-pin -q P1_26
config-pin -q P1_28
config-pin -q P2_09
config-pin -q P2_11

