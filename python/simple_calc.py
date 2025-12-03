#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
--------------------------------------------------------------------------
Simple Calculation Utilities for Plant Health Monitor
--------------------------------------------------------------------------

This module contains small helper functions used by the main
Plant Health Monitor application.

Currently implemented:

    clamp(value, low, high)
        - Clamp a value into a given [low, high] range.

    moisture_status(moisture_pct)
        - Convert a soil moisture percentage into a human-readable
          status string (used by the main application for LCD text
          and LED logic).

    dryness_level(moisture_pct)
        - Map moisture percentage into a numeric "dryness" level
          that could be used for additional logic, logging, or
          thresholds:

              0 -> Very Dry
              1 -> Dry
              2 -> Optimal
              3 -> Wet

These functions are kept separate from the hardware drivers so that
they can be easily tested and reused.
"""

# ------------------------------------------------------------------------
# Helper Functions
# ------------------------------------------------------------------------

def clamp(value, low, high):
    """
    Clamp 'value' into the inclusive range [low, high].

    Parameters:
        value (float or int): input value to be clamped
        low   (float or int): minimum allowed value
        high  (float or int): maximum allowed value

    Returns:
        float or int: clamped value
    """
    return max(low, min(high, value))


# ------------------------------------------------------------------------
# Soil Moisture Interpretation
# ------------------------------------------------------------------------

def moisture_status(moisture_pct):
    """
    Convert soil moisture percentage into a human-readable status string.

    This function is used by main.py to decide what to display on the LCD
    and which LEDs to turn on.

    Moisture thresholds (in %):
        >= 70%        -> "Soil Wet"
        40% - 69.9%   -> "Optimal"
        25% - 39.9%   -> "Dry"
        <  25%        -> "Very Dry!"

    Parameters:
        moisture_pct (float): soil moisture percentage (0–100)

    Returns:
        str: one of "Soil Wet", "Optimal", "Dry", or "Very Dry!"
    """
    # Make sure we are in a sensible 0–100% range
    m = clamp(moisture_pct, 0.0, 100.0)

    if m >= 70.0:
        return "Soil Wet"
    elif m >= 40.0:
        return "Optimal"
    elif m >= 25.0:
        return "Dry"
    else:
        return "Very Dry!"


def dryness_level(moisture_pct):
    """
    Map moisture percentage into a numeric dryness level.

        0 -> Very Dry    (0–24.9%)
        1 -> Dry         (25–39.9%)
        2 -> Optimal     (40–69.9%)
        3 -> Wet         (70–100%)

    This is not currently required by main.py, but it can be useful
    for logging, debugging, or future features (like hysteresis or
    averaging).

    Parameters:
        moisture_pct (float): soil moisture percentage (0–100)

    Returns:
        int: dryness level (0, 1, 2, or 3)
    """
    status = moisture_status(moisture_pct)

    if status == "Very Dry!":
        return 0
    elif status == "Dry":
        return 1
    elif status == "Optimal":
        return 2
    else:  # "Soil Wet"
        return 3


# ------------------------------------------------------------------------
# Self-test
# ------------------------------------------------------------------------

if __name__ == "__main__":
    print("SimpleCalc Moisture Status Self-Test")
    print("------------------------------------")

    example_values = [5, 20, 30, 45, 75, 100]

    for val in example_values:
        status = moisture_status(val)
        level  = dryness_level(val)
        print("Moisture: {:5.1f}% -> {:10s} (level {})".format(val, status, level))


