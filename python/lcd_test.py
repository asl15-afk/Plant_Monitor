#!/usr/bin/env python3
import time
from hd44780 import LCD

lcd = LCD(
    "P2_24", "P2_22",
    "P2_18", "P2_20", "P2_17", "P2_10",
    16, 2
)

lcd.clear()
lcd.message("LCD TEST OK\nHello PB")
time.sleep(3)
lcd.clear()

