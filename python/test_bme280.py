#!/usr/bin/env python3
from bme280_sensor import BME280, BME280_I2C_ADDR_76

print("Testing BMP280 on I2C bus 1, addr 0x76...")

sensor = BME280(bus=1, address=BME280_I2C_ADDR_76)

while True:
    temp, press = sensor.read()

    print("Temperature : {:.2f} C".format(temp))
    print("Pressure    : {:.2f} hPa".format(press))
    print("-" * 40)

    import time
    time.sleep(1)

