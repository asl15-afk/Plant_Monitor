#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
--------------------------------------------------------------------------
BMP280 I2C Temperature and Pressure Sensor Library
--------------------------------------------------------------------------

This driver supports the Bosch BMP280 sensor over I2C.
The BMP280 provides:
    - Temperature (°C)
    - Pressure (hPa)

NOTE: It does NOT provide humidity. If your chip ID is 0x58, you
are using a BMP280, not a BME280.

Chip IDs:
    0x58 -> BMP280
    0x60 -> BME280

Usage:
    from bmp280_sensor import BMP280

    sensor = BMP280(bus=1, address=0x76)
    temp_c = sensor.read_temperature()
    pressure_hpa = sensor.read_pressure()
    temp_c, pressure_hpa = sensor.read()

--------------------------------------------------------------------------"""

import time
import smbus

BMP280_DEFAULT_ADDR = 0x76
BMP280_CHIP_ID      = 0x58

# Registers
REG_ID        = 0xD0
REG_RESET     = 0xE0
REG_STATUS    = 0xF3
REG_CTRL_MEAS = 0xF4
REG_CONFIG    = 0xF5
REG_PRESS_MSB = 0xF7
REG_CALIB00   = 0x88

# Reset value
BMP280_RESET_VALUE = 0xB6


class BMP280:
    """Class to interface with the BMP280 temperature/pressure sensor."""

    def __init__(self, bus=1, address=BMP280_DEFAULT_ADDR):
        """
        Initialize BMP280 on given I2C bus and address.

        bus    : I2C bus number (PocketBeagle I2C1 is bus 1)
        address: I2C address (usually 0x76 or 0x77)
        """
        self._bus_num = bus
        self._address = address
        self._bus = smbus.SMBus(bus)

        # Calibration coefficients
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

        self.t_fine = 0.0

        # Check chip ID
        chip_id = self._bus.read_byte_data(self._address, REG_ID)
        if chip_id != BMP280_CHIP_ID:
            print("Warning: Expected BMP280 (ID 0x58), got 0x{:02X}".format(chip_id))

        # Reset and configure
        self._reset()
        time.sleep(0.1)
        self._read_calibration()
        self._configure()

    # ----------------- Low-level helpers -------------------------------

    def _reset(self):
        """Soft reset the sensor."""
        try:
            self._bus.write_byte_data(self._address, REG_RESET, BMP280_RESET_VALUE)
        except IOError:
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
        """Read temperature and pressure calibration coefficients."""
        calib = self._bus.read_i2c_block_data(self._address, REG_CALIB00, 24)

        self.dig_T1 = self._to_u16(calib[1], calib[0])
        self.dig_T2 = self._to_s16(calib[3], calib[2])
        self.dig_T3 = self._to_s16(calib[5], calib[4])

        self.dig_P1 = self._to_u16(calib[7],  calib[6])
        self.dig_P2 = self._to_s16(calib[9],  calib[8])
        self.dig_P3 = self._to_s16(calib[11], calib[10])
        self.dig_P4 = self._to_s16(calib[13], calib[12])
        self.dig_P5 = self._to_s16(calib[15], calib[14])
        self.dig_P6 = self._to_s16(calib[17], calib[16])
        self.dig_P7 = self._to_s16(calib[19], calib[18])
        self.dig_P8 = self._to_s16(calib[21], calib[20])
        self.dig_P9 = self._to_s16(calib[23], calib[22])

    def _configure(self):
        """Configure oversampling and normal mode."""
        # Temperature oversampling x1, pressure oversampling x1, normal mode
        self._bus.write_byte_data(self._address, REG_CTRL_MEAS, 0x27)
        # Config: standby 1000 ms, filter off
        self._bus.write_byte_data(self._address, REG_CONFIG, 0xA0)

    # ----------------- Raw reading -------------------------------------

    def _read_raw(self):
        """
        Read raw uncompensated temperature and pressure.

        Returns:
            (adc_T, adc_P)
        """
        data = self._bus.read_i2c_block_data(self._address, REG_PRESS_MSB, 6)

        # Pressure raw 20-bit
        adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        # Temperature raw 20-bit
        adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)

        return adc_T, adc_P

    # ----------------- Compensation formulas ---------------------------

    def _compensate_temperature(self, adc_T):
        """Return temperature in °C."""
        var1 = (adc_T / 16384.0 - self.dig_T1 / 1024.0) * self.dig_T2
        var2 = ((adc_T / 131072.0 - self.dig_T1 / 8192.0) *
                (adc_T / 131072.0 - self.dig_T1 / 8192.0)) * self.dig_T3

        self.t_fine = var1 + var2
        T = self.t_fine / 5120.0
        return T

    def _compensate_pressure(self, adc_P):
        """Return pressure in Pa."""
        var1 = self.t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * self.dig_P6 / 32768.0
        var2 = var2 + var1 * self.dig_P5 * 2.0
        var2 = var2 / 4.0 + self.dig_P4 * 65536.0
        var1 = (self.dig_P3 * var1 * var1 / 524288.0 +
                self.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1

        if var1 == 0:
            return 0

        p = 1048576.0 - adc_P
        p = (p - var2 / 4096.0) * 6250.0 / var1
        var1 = self.dig_P9 * p * p / 2147483648.0
        var2 = p * self.dig_P8 / 32768.0
        p = p + (var1 + var2 + self.dig_P7) / 16.0
        return p  # Pa

    # ----------------- Public API --------------------------------------

    def read_temperature(self):
        """Return temperature in °C."""
        adc_T, _ = self._read_raw()
        return self._compensate_temperature(adc_T)

    def read_pressure(self):
        """Return pressure in hPa."""
        adc_T, adc_P = self._read_raw()
        self._compensate_temperature(adc_T)
        p_pa = self._compensate_pressure(adc_P)
        return p_pa / 100.0  # hPa

    def read(self):
        """
        Read both temperature and pressure.

        Returns:
            (temperature_C, pressure_hPa)
        """
        adc_T, adc_P = self._read_raw()
        temp_c = self._compensate_temperature(adc_T)
        p_hpa = self._compensate_pressure(adc_P) / 100.0
        return temp_c, p_hpa


# ------------------------------------------------------------------------
# Self-test
# ------------------------------------------------------------------------

if __name__ == "__main__":
    print("BMP280 Temperature / Pressure Sensor Self-Test")
    print("Using I2C bus 1, address 0x76\n")

    sensor = BMP280(bus=1, address=BMP280_DEFAULT_ADDR)

    try:
        while True:
            t, p = sensor.read()
            print("Temperature : {:6.2f} C".format(t))
            print("Pressure    : {:7.2f} hPa".format(p))
            print("-" * 32)
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nSelf-test ended by user.")

