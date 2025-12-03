#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
--------------------------------------------------------------------------
BME280 I2C Temperature, Humidity, and Pressure Sensor Library
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

BME280(bus=1, address=0x76)
    - Create a new BME280 object on the given I2C bus and address.
    - Default bus is 1 (PocketBeagle I2C1 → P2_11 / P2_09).
    - Default address is 0x76 (many breakouts use 0x76; some use 0x77).

read_raw()
    - Read raw uncompensated temperature, pressure, and humidity values.
      Returns (adc_T, adc_P, adc_H).

read_temperature()
    - Return temperature in degrees Celsius (float).

read_pressure()
    - Return pressure in hPa (hectopascals, same as mbar) as float.

read_humidity()
    - Return relative humidity in %RH (float).

read()
    - Return a tuple (temperature_C, pressure_hPa, humidity_RH).

--------------------------------------------------------------------------

Hardware / Wiring Information
-----------------------------

Sensor: BME280 (Bosch) breakout — I2C/SPI, 3.3V, soldered connectors.
Tested with GY-BME280-style modules.

Typical breakout pinout:
    VIN / VCC   - 3.3 V supply
    GND         - ground
    SDA         - I2C data
    SCL         - I2C clock
    CS / CSB    - chip select (SPI); must be HIGH for I2C
    SDO         - I2C address select / SPI MISO

PocketBeagle Wiring (using I2C1 → /dev/i2c-1):
    VIN  -> P1_14 (3.3 V)
    GND  -> P1_15 (GND)
    SDA  -> P2_11 (I2C1_SDA)
    SCL  -> P2_09 (I2C1_SCL)
    CSB  -> P1_14 (3.3 V)   # hold HIGH to enable I2C mode
    SDO  -> P1_15 (GND)     # address 0x76 (tie to 3.3V for 0x77)

Check device is present:
    $ sudo i2cdetect -y -r 1
    # Look for 0x76 or 0x77

--------------------------------------------------------------------------

Background / Compensation Formulas
----------------------------------

The BME280 returns raw 20-bit temperature and pressure readings, and a
16-bit humidity reading. Calibration coefficients stored in the device
must be used to convert these into human-readable values.

This driver implements the compensation formulas from the BME280
datasheet (float math) to compute:

    * Temperature (°C)
    * Pressure (Pa, then converted to hPa)
    * Relative humidity (%RH)

t_fine is used as an internal intermediate value for all conversions.

"""

import time
import smbus


# ------------------------------------------------------------------------
# Constants
# ------------------------------------------------------------------------

# Default I2C addresses (use the one that matches your wiring)
BME280_I2C_ADDR_76 = 0x76
BME280_I2C_ADDR_77 = 0x77

# Register addresses
REG_ID        = 0xD0
REG_RESET     = 0xE0
REG_CTRL_HUM  = 0xF2
REG_STATUS    = 0xF3
REG_CTRL_MEAS = 0xF4
REG_CONFIG    = 0xF5
REG_PRESS_MSB = 0xF7  # start of pressure/temperature/humidity block

REG_CALIB00   = 0x88  # temp/press calibration start
REG_CALIB26   = 0xE1  # humidity calibration start

# Reset value
BME280_RESET_VALUE = 0xB6


# ------------------------------------------------------------------------
# Classes
# ------------------------------------------------------------------------

class BME280:
    """Class to interface with BME280 temperature, humidity, and pressure sensor."""

    def __init__(self, bus=1, address=BME280_I2C_ADDR_76):
        """
        Initialize BME280 object and configure the sensor.

        Parameters:
            bus (int)     : I2C bus number (PocketBeagle I2C1 is bus 1)
            address (int) : I2C address (0x76 or 0x77)
        """
        self._bus_num = bus
        self._address = address
        self._bus = smbus.SMBus(bus)

        # Calibration data (filled in by _read_calibration)
        self.dig_T1 = 0
        self.dig_T2 = 0
        self.dig_T3 = 0
        self.dig_P1 = 0
        self.dig_P2 = 0
        self.dig_P3 = 0
        self.dig_P4 = 0
        self.dig_P5 = 0
        self.dig_P6 = 0
        self.dig_P7 = 0
        self.dig_P8 = 0
        self.dig_P9 = 0
        self.dig_H1 = 0
        self.dig_H2 = 0
        self.dig_H3 = 0
        self.dig_H4 = 0
        self.dig_H5 = 0
        self.dig_H6 = 0

        self.t_fine = 0.0

        # Soft reset, read calibration, configure sensor
        self._reset()
        time.sleep(0.1)
        self._read_calibration()
        self._configure()

    # ----------------- Low-level helpers -------------------------------

    def _reset(self):
        """Soft reset the sensor."""
        try:
            self._bus.write_byte_data(self._address, REG_RESET, BME280_RESET_VALUE)
        except OSError:
            # If this fails, it's usually an address/wiring problem.
            # Caller will see failures on next operations.
            pass

    @staticmethod
    def _to_s16(msb, lsb):
        """Convert two bytes to signed 16-bit."""
        value = (msb << 8) | lsb
        if value & 0x8000:
            value -= 0x10000
        return value

    @staticmethod
    def _to_u16(msb, lsb):
        """Convert two bytes to unsigned 16-bit."""
        return (msb << 8) | lsb

    def _read_calibration(self):
        """Read calibration data from BME280."""
        # Read 24 bytes from 0x88 to 0xA1 (temp/pressure)
        calib = self._bus.read_i2c_block_data(self._address, REG_CALIB00, 24)

        self.dig_T1 = self._to_u16(calib[1], calib[0])
        self.dig_T2 = self._to_s16(calib[3], calib[2])
        self.dig_T3 = self._to_s16(calib[5], calib[4])

        self.dig_P1 = self._to_u16(calib[7], calib[6])
        self.dig_P2 = self._to_s16(calib[9],  calib[8])
        self.dig_P3 = self._to_s16(calib[11], calib[10])
        self.dig_P4 = self._to_s16(calib[13], calib[12])
        self.dig_P5 = self._to_s16(calib[15], calib[14])
        self.dig_P6 = self._to_s16(calib[17], calib[16])
        self.dig_P7 = self._to_s16(calib[19], calib[18])
        self.dig_P8 = self._to_s16(calib[21], calib[20])
        self.dig_P9 = self._to_s16(calib[23], calib[22])

        # Read H1 from 0xA1 (one byte)
        self.dig_H1 = self._bus.read_byte_data(self._address, 0xA1)

        # Read humidity calibration from 0xE1..0xE7 (7 bytes)
        calib_h = self._bus.read_i2c_block_data(self._address, REG_CALIB26, 7)

        self.dig_H2 = self._to_s16(calib_h[1], calib_h[0])
        self.dig_H3 = calib_h[2]

        # H4 and H5 are stored across E4, E5, E6:
        # H4 = (E4 << 4) | (E5 & 0x0F)
        # H5 = (E6 << 4) | (E5 >> 4)
        e4 = calib_h[3]
        e5 = calib_h[4]
        e6 = calib_h[5]

        h4 = (e4 << 4) | (e5 & 0x0F)
        h5 = (e6 << 4) | (e5 >> 4)

        # Convert to signed 12-bit, then to Python int
        if h4 & 0x800:
            h4 -= 0x1000
        if h5 & 0x800:
            h5 -= 0x1000

        self.dig_H4 = h4
        self.dig_H5 = h5

        # H6 is signed 8-bit
        h6 = calib_h[6]
        if h6 & 0x80:
            h6 -= 0x100
        self.dig_H6 = h6

    def _configure(self):
        """Configure sensor oversampling and mode."""
        # Humidity oversampling x1
        self._bus.write_byte_data(self._address, REG_CTRL_HUM, 0x01)

        # Temperature and pressure oversampling x1, normal mode
        self._bus.write_byte_data(self._address, REG_CTRL_MEAS, 0x27)

        # Standby 1000 ms, filter off
        self._bus.write_byte_data(self._address, REG_CONFIG, 0xA0)

    # ----------------- Raw data reading --------------------------------

    def read_raw(self):
        """
        Read raw uncompensated temperature, pressure, and humidity.

        Returns:
            (adc_T, adc_P, adc_H) as integers.
        """
        # Data block: pressure (3 bytes), temperature (3 bytes), humidity (2 bytes)
        data = self._bus.read_i2c_block_data(self._address, REG_PRESS_MSB, 8)

        # Pressure raw 20-bit
        adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        # Temperature raw 20-bit
        adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        # Humidity raw 16-bit
        adc_H = (data[6] << 8) | data[7]

        return adc_T, adc_P, adc_H

    # ----------------- Compensation calculations -----------------------

    def _compensate_temperature(self, adc_T):
        """
        Temperature compensation. Updates t_fine and returns degC.
        """
        var1 = (adc_T / 16384.0 - self.dig_T1 / 1024.0) * self.dig_T2
        var2 = ((adc_T / 131072.0 - self.dig_T1 / 8192.0) *
                (adc_T / 131072.0 - self.dig_T1 / 8192.0)) * self.dig_T3

        self.t_fine = var1 + var2
        T = self.t_fine / 5120.0
        return T

    def _compensate_pressure(self, adc_P):
        """
        Pressure compensation. Returns pressure in Pa.
        """
        var1 = self.t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * self.dig_P6 / 32768.0
        var2 = var2 + var1 * self.dig_P5 * 2.0
        var2 = var2 / 4.0 + self.dig_P4 * 65536.0
        var1 = (self.dig_P3 * var1 * var1 / 524288.0 +
                self.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1

        if var1 == 0:
            return 0.0  # avoid division by zero

        p = 1048576.0 - adc_P
        p = (p - var2 / 4096.0) * 6250.0 / var1
        var1 = self.dig_P9 * p * p / 2147483648.0
        var2 = p * self.dig_P8 / 32768.0
        p = p + (var1 + var2 + self.dig_P7) / 16.0

        return p  # Pa

    def _compensate_humidity(self, adc_H):
        """
        Humidity compensation. Returns %RH.
        """
        var_h = self.t_fine - 76800.0
        if var_h == 0:
            return 0.0

        var_h = (adc_H - (self.dig_H4 * 64.0 +
                          self.dig_H5 / 16384.0 * var_h)) * \
                (self.dig_H2 / 65536.0 *
                 (1.0 + self.dig_H6 / 67108864.0 * var_h *
                  (1.0 + self.dig_H3 / 67108864.0 * var_h)))

        var_h = var_h * (1.0 - self.dig_H1 * var_h / 524288.0)

        if var_h > 100.0:
            var_h = 100.0
        elif var_h < 0.0:
            var_h = 0.0

        return var_h

    # ----------------- Public API --------------------------------------

    def read_temperature(self):
        """Return temperature in degrees Celsius."""
        adc_T, _, _ = self.read_raw()
        return self._compensate_temperature(adc_T)

    def read_pressure(self):
        """Return pressure in hPa (hectopascals)."""
        adc_T, adc_P, _ = self.read_raw()
        # Need valid t_fine, so ensure temperature compensation is run
        self._compensate_temperature(adc_T)
        p_pa = self._compensate_pressure(adc_P)
        return p_pa / 100.0  # Pa → hPa

    def read_humidity(self):
        """Return relative humidity in %RH."""
        adc_T, _, adc_H = self.read_raw()
        self._compensate_temperature(adc_T)
        return self._compensate_humidity(adc_H)

    def read(self):
        """
        Read all values at once.

        Returns:
            (temperature_C, pressure_hPa, humidity_RH)
        """
        adc_T, adc_P, adc_H = self.read_raw()
        temp = self._compensate_temperature(adc_T)
        press_hpa = self._compensate_pressure(adc_P) / 100.0
        hum = self._compensate_humidity(adc_H)
        return temp, press_hpa, hum


# ------------------------------------------------------------------------
# Main script (self-test)
# ------------------------------------------------------------------------

if __name__ == "__main__":
    print("BME280 Temperature / Humidity / Pressure Sensor - Self Test")
    print("Using I2C bus 1, address 0x76\n")

    try:
        sensor = BME280(bus=1, address=BME280_I2C_ADDR_76)
    except OSError as e:
        print("Error initializing BME280:", e)
        print("Check wiring, I2C bus, and address (0x76 vs 0x77).")
        raise SystemExit(1)

    try:
        while True:
            temp_c, press_hpa, hum = sensor.read()

            print("Temperature : {:6.2f} C".format(temp_c))
            print("Pressure    : {:7.2f} hPa".format(press_hpa))
            print("Humidity    : {:6.2f} %RH".format(hum))
            print("-" * 32)

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nSelf-test ended by user.")
