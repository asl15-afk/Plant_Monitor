#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
--------------------------------------------------------------------------
Adafruit STEMMA I2C Capacitive Soil Moisture Sensor Library
--------------------------------------------------------------------------

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

Software API
------------

SoilMoistureSensor(bus=2, address=0x36)
    - Create a new soil moisture sensor object on the given I2C bus
      and address.
    - Default bus is 2 (PocketBeagle I2C2).
    - Default address is 0x36 (Adafruit STEMMA Soil Sensor).

read_moisture_raw()
    - Returns the raw capacitive moisture value as a 16-bit integer.

read_moisture_percent(dry=300, wet=1200)
    - Returns a 0–100% moisture estimate based on raw capacitance.
    - `dry` and `wet` are calibration points you can tune for your soil.

--------------------------------------------------------------------------

Hardware / Wiring Information
-----------------------------

Sensor: Adafruit STEMMA Soil Sensor - I2C Capacitive Moisture Sensor

Pinout:
    GND      - power and logic ground
    VIN      - 3.0–5.0 V (must match I2C logic level)
    I2C SDA  - I2C data, 10k pull-up to VIN
    I2C SCL  - I2C clock, 10k pull-up to VIN

PocketBeagle Wiring (using I2C2 on this board):
    VIN  -> P1_14 (3.3 V)
    GND  -> P1_15 (GND)
    SDA  -> P1_26 (I2C2_SDA)
    SCL  -> P1_28 (I2C2_SCL)

Check device is present:
    $ sudo i2cdetect -y -r 2
    (should show 0x36 in the 30: row)

--------------------------------------------------------------------------

Background / Seesaw Protocol Notes
----------------------------------

The STEMMA Soil Sensor uses an on-board microcontroller running
Adafruit's "Seesaw" firmware.

To read the capacitive moisture measurement on channel 0 the docs say:

    Base register (module)   = 0x0F
    Function register        = 0x10

Sequence:
    1. I2C write: [0x0F, 0x10]   (select TOUCH base + channel 0)
    2. Wait ~5 ms
    3. I2C read: 2 bytes         (16-bit moisture value)

This driver implements exactly that using smbus2's i2c_rdwr support.

"""

import time
from smbus2 import SMBus, i2c_msg


# ------------------------------------------------------------------------
# Constants
# ------------------------------------------------------------------------

# Default I2C address for Adafruit STEMMA Soil Sensor
SOIL_I2C_ADDR = 0x36

# Seesaw moisture registers (from Adafruit FAQ)
SEESAW_TOUCH_BASE          = 0x0F  # module / base
SEESAW_TOUCH_CHANNEL_0     = 0x10  # function / channel 0 moisture


# ------------------------------------------------------------------------
# Classes
# ------------------------------------------------------------------------

class SoilMoistureSensor:
    """Class to interface with Adafruit STEMMA I2C Soil Moisture Sensor."""

    def __init__(self, bus=2, address=SOIL_I2C_ADDR):
        """
        Initialize the soil moisture sensor.

        Parameters:
            bus (int)     : I2C bus number (PocketBeagle I2C2 is bus 2)
            address (int) : I2C address of the sensor (default 0x36)
        """
        self._bus_num = bus
        self._address = address
        self._bus = SMBus(bus)

    # ----------------- Low-level Seesaw helpers ------------------------

    def _read_u16_from_seesaw(self, base, function, delay_s=0.005):
        """
        Read a 16-bit unsigned value from a Seesaw module/function.

        Protocol:
            1) Write [base, function]
            2) Wait for the device to perform the measurement
            3) Read 2 bytes (big-endian)

        Returns:
            int : 16-bit unsigned value (0–65535)
        """
        # 1) Write base + function
        write_msg = i2c_msg.write(self._address, [base, function])
        self._bus.i2c_rdwr(write_msg)

        # 2) Wait a little for conversion / internal processing
        time.sleep(delay_s)

        # 3) Read 2 bytes back
        read_msg = i2c_msg.read(self._address, 2)
        self._bus.i2c_rdwr(read_msg)
        data = list(read_msg)

        # Big-endian 16-bit value
        value = (data[0] << 8) | data[1]
        return value

    # ----------------- Public API methods ------------------------------

    def read_moisture_raw(self):
        """
        Read the raw capacitive moisture value.

        Returns:
            int : raw moisture reading (typ. ~300–500 in soil, higher = wetter)
        """
        value = self._read_u16_from_seesaw(
            SEESAW_TOUCH_BASE,
            SEESAW_TOUCH_CHANNEL_0,
            delay_s=0.005
        )
        return value

    def read_moisture_percent(self, dry=300, wet=1200):
        """
        Convert raw moisture reading into an approximate 0–100% value.

        Parameters:
            dry (int) : raw reading for "0%" moisture (very dry soil)
            wet (int) : raw reading for "100%" moisture (very wet soil)

        Returns:
            float : moisture percentage, clamped between 0.0 and 100.0
        """
        raw = self.read_moisture_raw()

        # Clamp and map linearly between dry and wet
        if raw <= dry:
            return 0.0
        if raw >= wet:
            return 100.0

        span = float(wet - dry)
        pct = (raw - dry) * 100.0 / span
        return pct


# ------------------------------------------------------------------------
# Main script (self-test)
# ------------------------------------------------------------------------

if __name__ == "__main__":
    print("Adafruit STEMMA Soil Moisture Sensor - Seesaw Self Test")
    print("Using I2C bus 1, address 0x36\n")

    sensor = SoilMoistureSensor(bus=1, address=SOIL_I2C_ADDR)

    try:
        while True:
            raw = sensor.read_moisture_raw()
            pct = sensor.read_moisture_percent()

            print("Raw moisture : {:5d}".format(raw))
            print("Moisture     : {:5.1f} %".format(pct))
            print("-" * 30)

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nSelf-test ended by user.")
