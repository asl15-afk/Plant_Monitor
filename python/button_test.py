#!/usr/bin/env python3
import time
from button import Button

btn = Button("P2_27")

print("Press Ctrl+C to stop")
while True:
    print("Pressed:", btn.is_pressed())
    time.sleep(0.2)

