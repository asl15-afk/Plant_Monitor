import time
import Adafruit_BBIO.GPIO as GPIO

print("\n=== PocketBeagle GPIO Test ===\n")

# -------------------------------------------------------------------
# Pin Definitions (MATCH YOUR PROJECT)
# -------------------------------------------------------------------

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

BUTTON_PIN = "P2_27"

# -------------------------------------------------------------------
# LCD GPIO OUTPUT TEST
# -------------------------------------------------------------------

print("LCD PINS:")
for name, pin in LCD_PINS.items():
    try:
        print(f"Testing {name} ({pin}) as OUTPUT...", end=" ")
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(0.05)
        GPIO.output(pin, GPIO.LOW)
        print("OK")
    except Exception as e:
        print("FAIL")
        print(" ", e)

print()

# -------------------------------------------------------------------
# LED OUTPUT TEST
# -------------------------------------------------------------------

print("LED PINS:")
for name, pin in LED_PINS.items():
    try:
        print(f"Testing {name} ({pin}) as OUTPUT...", end=" ")
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(0.3)
        GPIO.output(pin, GPIO.LOW)
        print("OK")
    except Exception as e:
        print("FAIL")
        print(" ", e)

print()

# -------------------------------------------------------------------
# BUTTON INPUT TEST
# -------------------------------------------------------------------

print("BUTTON PIN:")
try:
    GPIO.setup(BUTTON_PIN, GPIO.IN)
    val = GPIO.input(BUTTON_PIN)
    print(f"Testing BUTTON ({BUTTON_PIN}) as INPUT... OK (value = {val})")
except Exception as e:
    print("FAIL")
    print(" ", e)

print("\n=== Test Complete ===\n")