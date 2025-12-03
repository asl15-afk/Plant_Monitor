#!/usr/bin/env python3
import time
from led import LED

red = LED("P2_19")
yellow = LED("P2_25")
green = LED("P2_29")

for led in (red, yellow, green):
    led.on()
    time.sleep(1)
    led.off()

