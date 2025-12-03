# -*- coding: utf-8 -*-
"""
--------------------------------------------------------------------------
HD44780 4-bit Parallel LCD Library for PocketBeagle
--------------------------------------------------------------------------

License:   
Copyright 2025 Alexis Leyow

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors 
may be used to endorse or promote products derived from this software without 
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------

This module drives a standard HD44780-compatible character LCD in 4-bit
mode using six GPIO pins on the PocketBeagle.

Default wiring (matches setup):

    LCD Pin   Function    PocketBeagle Pin
    -------   --------    ----------------
      1       GND         P1_15 (GND)
      2       VDD         P1_14 (3.3V)
      3       VO          Potentiometer wiper (contrast)
      4       RS          P2_24
      5       RW          GND
      6       E           P2_22
     11       D4          P2_18
     12       D5          P2_20
     13       D6          P2_4
     14       D7          P2_6
     15       BLA         3.3V (backlight +)
     16       BLK         GND  (backlight -)

Usage example:

    from HD44780 import LCD

    lcd = LCD()  # uses default pins and 16x2 size
    lcd.clear()
    lcd.message("Hello,\nPocketBeagle")
--------------------------------------------------------------------------"""

import time
import Adafruit_BBIO.GPIO as GPIO

# ------------------------------------------------------------------------
# LCD Command Constants
# ------------------------------------------------------------------------

LCD_CLEARDISPLAY   = 0x01
LCD_RETURNHOME     = 0x02
LCD_ENTRYMODESET   = 0x04
LCD_DISPLAYCONTROL = 0x08
LCD_CURSORSHIFT    = 0x10
LCD_FUNCTIONSET    = 0x20
LCD_SETCGRAMADDR   = 0x40
LCD_SETDDRAMADDR   = 0x80

# Entry mode flags
LCD_ENTRYRIGHT          = 0x00
LCD_ENTRYLEFT           = 0x02
LCD_ENTRYSHIFTINCREMENT = 0x01
LCD_ENTRYSHIFTDECREMENT = 0x00

# Display control flags
LCD_DISPLAYON  = 0x04
LCD_DISPLAYOFF = 0x00
LCD_CURSORON   = 0x02
LCD_CURSOROFF  = 0x00
LCD_BLINKON    = 0x01
LCD_BLINKOFF   = 0x00

# Cursor/display shift flags
LCD_DISPLAYMOVE = 0x08
LCD_CURSORMOVE  = 0x00
LCD_MOVERIGHT   = 0x04
LCD_MOVELEFT    = 0x00

# Function set flags
LCD_8BITMODE = 0x10
LCD_4BITMODE = 0x00
LCD_2LINE    = 0x08
LCD_1LINE    = 0x00
LCD_5x10DOTS = 0x04
LCD_5x8DOTS  = 0x00

# Row offsets for 16x2 (DDRAM addresses)
LCD_ROW_OFFSETS = (0x00, 0x40)

# Default PocketBeagle pins for this project
DEFAULT_RS = "P2_24"
DEFAULT_E  = "P2_22"
DEFAULT_D4 = "P2_18"
DEFAULT_D5 = "P2_20"
DEFAULT_D6 = "P2_17"
DEFAULT_D7 = "P2_10"

# ------------------------------------------------------------------------
# LCD Class
# ------------------------------------------------------------------------

class LCD:
    """
    Class to control an HD44780-compatible LCD in 4-bit mode.

    Pins default to the wiring listed above, but you can override them:

        lcd = LCD(rs="P2_24", enable="P2_22",
                  d4="P2_18", d5="P2_20", d6="P2_4", d7="P2_6",
                  cols=16, rows=2)
    """

    def __init__(self,
                 rs=DEFAULT_RS,
                 enable=DEFAULT_E,
                 d4=DEFAULT_D4,
                 d5=DEFAULT_D5,
                 d6=DEFAULT_D6,
                 d7=DEFAULT_D7,
                 cols=16,
                 rows=2):
        self._cols   = cols
        self._rows   = rows
        self._rs     = rs
        self._enable = enable
        self._d4     = d4
        self._d5     = d5
        self._d6     = d6
        self._d7     = d7

        # Last known cursor position (col, row)
        self.cursor_position = (0, 0)

        self._setup_pins()
        self._init_lcd()

    # ----------------- Setup & Initialization ---------------------------

    def _setup_pins(self):
        """Configure the GPIO pins as outputs."""
        for pin in (self._rs, self._enable, self._d4, self._d5, self._d6, self._d7):
            GPIO.setup(pin, GPIO.OUT)

    def _init_lcd(self):
        """Run the HD44780 4-bit initialization sequence."""
        time.sleep(0.05)  # Wait for LCD to power up

        # Put into 4-bit mode (standard magic sequence)
        self._write4bits(0x03 << 4)
        time.sleep(0.005)
        self._write4bits(0x03 << 4)
        time.sleep(0.005)
        self._write4bits(0x03 << 4)
        time.sleep(0.001)
        self._write4bits(0x02 << 4)  # 4-bit mode

        # Set function (4-bit, lines, font)
        self.displayfunction = LCD_4BITMODE | LCD_5x8DOTS
        if self._rows > 1:
            self.displayfunction |= LCD_2LINE
        else:
            self.displayfunction |= LCD_1LINE

        # Display on, cursor & blink off by default
        self.displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF

        # Text entry mode: left to right, no shift
        self.displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT

        # Send configuration
        self.command(LCD_FUNCTIONSET | self.displayfunction)
        self.command(LCD_DISPLAYCONTROL | self.displaycontrol)
        self.command(LCD_ENTRYMODESET | self.displaymode)

        self.clear()

    # ----------------- High-level API ----------------------------------

    def clear(self):
        """Clear display and reset cursor to (0,0)."""
        self.command(LCD_CLEARDISPLAY)
        self._delay_microseconds(3000)
        self.cursor_position = (0, 0)

    def home(self):
        """Return cursor to home position (0,0)."""
        self.command(LCD_RETURNHOME)
        self._delay_microseconds(3000)
        self.cursor_position = (0, 0)

    def set_cursor(self, col, row):
        """Move the cursor to a specific column and row."""
        # Clamp to valid range
        if row < 0:
            row = 0
        if row >= self._rows:
            row = self._rows - 1

        if col < 0:
            col = 0
        if col >= self._cols:
            col = self._cols - 1

        addr = col + LCD_ROW_OFFSETS[row]
        self.command(LCD_SETDDRAMADDR | addr)
        self.cursor_position = (col, row)

    def get_cursor(self):
        """Return last known cursor position as (col, row)."""
        return self.cursor_position

    def show_cursor(self, show=True):
        """Show or hide the underline cursor."""
        if show:
            self.displaycontrol |= LCD_CURSORON
        else:
            self.displaycontrol &= ~LCD_CURSORON
        self.command(LCD_DISPLAYCONTROL | self.displaycontrol)

    def blink_cursor(self, blink=True):
        """Enable or disable blinking cursor."""
        if blink:
            self.displaycontrol |= LCD_BLINKON
        else:
            self.displaycontrol &= ~LCD_BLINKON
        self.command(LCD_DISPLAYCONTROL | self.displaycontrol)

    def enable_display(self, enable=True):
        """Turn the display on or off (contents in memory are kept)."""
        if enable:
            self.displaycontrol |= LCD_DISPLAYON
        else:
            self.displaycontrol &= ~LCD_DISPLAYON
        self.command(LCD_DISPLAYCONTROL | self.displaycontrol)

    def move_display_left(self):
        """Scroll the entire display content left."""
        self.command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT)

    def move_display_right(self):
        """Scroll the entire display content right."""
        self.command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT)

    def message(self, text):
        """
        Write a string to the LCD.

        '\n' moves to the next row. Extra characters beyond the row/column
        limits are simply ignored (they won't wrap around).
        """
        row = 0
        col = 0
        self.set_cursor(col, row)

        for ch in text:
            if ch == '\n':
                row += 1
                col = 0
                if row >= self._rows:
                    break
                self.set_cursor(col, row)
            else:
                self.write_char(ch)
                col += 1
                if col >= self._cols:
                    continue


    def write_char(self, ch):
        """Write a single character at the current cursor position."""
        self.write8(ord(ch), char_mode=True)

    def cleanup(self):
        """Release GPIO pins (optional but nice)."""
        GPIO.cleanup()

    # ----------------- Low-level helpers -------------------------------

    def command(self, value):
        """Send a command byte (RS=0)."""
        self.write8(value, char_mode=False)

    def write8(self, value, char_mode=False):
        """
        Write an 8-bit value to the LCD in 4-bit mode.

        char_mode = False → command (RS=0)
        char_mode = True  → data/character (RS=1)
        """
        self._delay_microseconds(50)

        # RS: LOW for command, HIGH for data
        GPIO.output(self._rs, GPIO.HIGH if char_mode else GPIO.LOW)

        # Send high nibble first, then low nibble
        self._write4bits(value & 0xF0)
        self._write4bits((value << 4) & 0xF0)

    def _write4bits(self, data):
        """Output the upper 4 bits of 'data' on D4..D7 then pulse E."""
        # data is expected to have the nibble in bits 7..4:
        GPIO.output(self._d4, bool(data & 0x10))
        GPIO.output(self._d5, bool(data & 0x20))
        GPIO.output(self._d6, bool(data & 0x40))
        GPIO.output(self._d7, bool(data & 0x80))
        self._pulse_enable()

    def _pulse_enable(self):
        """Toggle the Enable pin to latch data into the LCD."""
        GPIO.output(self._enable, False)
        self._delay_microseconds(1)
        GPIO.output(self._enable, True)
        self._delay_microseconds(1)
        GPIO.output(self._enable, False)
        self._delay_microseconds(50)

    @staticmethod
    def _delay_microseconds(us):
        """Busy-wait for approximately 'us' microseconds."""
        end = time.time() + us / 1_000_000.0
        while time.time() < end:
            pass

# ------------------------------------------------------------------------
# Self-test (run this file directly to test wiring)
# ------------------------------------------------------------------------

if __name__ == "__main__":
    print("HD44780 LCD self-test starting...")
    lcd = LCD()  # uses default pins and 16x2

    lcd.clear()
    lcd.show_cursor(True)
    lcd.message("Hello,\nPocketBeagle")
    time.sleep(3)

    lcd.show_cursor(False)
    lcd.clear()
    lcd.message("Test finished.")
    time.sleep(2)

    lcd.clear()
    lcd.cleanup()
    print("Self-test complete.")
