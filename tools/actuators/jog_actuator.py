#!/usr/bin/env python3
"""
Actuonix L16-100-150-6-R test on Jetson Orin Nano (Super)
Signal: pin 15 (hardware PWM)   Ground: any Jetson GND pin, shared with the 6V supply

Before running:
  1. sudo /opt/nvidia/jetson-io/jetson-io.py
     -> Configure Jetson 40pin Header -> Configure header pins manually
     -> enable pwm1 (pin 15) -> Save and reboot
  2. Jetson.GPIO is already installed system-wide.

Run (no sudo -- this account is in the 'gpio' group, and sudo would strip
the JETSON_MODEL_NAME set below unless you pass -E):
  python3 tools/actuators/jog_actuator.py
"""

import os
import time

# This devkit's device tree reports "nvidia,p3768-0000+p3767-0005-super", and
# Jetson.GPIO 2.1.7's model table only lists the string without the "-super"
# suffix. Without this override the import below dies with "Could not determine
# Jetson model". Must be set before the import.
os.environ.setdefault("JETSON_MODEL_NAME", "JETSON_ORIN_NANO")

import Jetson.GPIO as GPIO  # noqa: E402

PIN = 15            # board pin number
FREQ_HZ = 50        # RC servo standard: 20 ms period
PERIOD_MS = 1000.0 / FREQ_HZ

MIN_PULSE_MS = 1.0  # fully retracted (0 mm)
MAX_PULSE_MS = 2.0  # fully extended (100 mm)
STROKE_MM = 100.0

# 150:1 gear ratio is slow (~8 mm/s at 6V), so a full stroke takes ~13 s
FULL_STROKE_WAIT_S = 14


def pulse_to_duty(pulse_ms):
    return pulse_ms / PERIOD_MS * 100.0


def mm_to_duty(mm):
    mm = max(0.0, min(STROKE_MM, mm))
    pulse = MIN_PULSE_MS + (mm / STROKE_MM) * (MAX_PULSE_MS - MIN_PULSE_MS)
    return pulse_to_duty(pulse)


def move_to(pwm, mm, wait_s):
    print(f"-> {mm:.0f} mm  (duty {mm_to_duty(mm):.2f}%)")
    pwm.ChangeDutyCycle(mm_to_duty(mm))
    time.sleep(wait_s)


def main():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PIN, GPIO.OUT, initial=GPIO.LOW)
    pwm = GPIO.PWM(PIN, FREQ_HZ)
    pwm.start(mm_to_duty(0))

    try:
        print("Retracting to 0 mm...")
        time.sleep(FULL_STROKE_WAIT_S)

        print("\n--- Automatic sweep ---")
        move_to(pwm, 50, FULL_STROKE_WAIT_S / 2)
        move_to(pwm, 100, FULL_STROKE_WAIT_S / 2)
        move_to(pwm, 25, FULL_STROKE_WAIT_S)
        move_to(pwm, 0, FULL_STROKE_WAIT_S / 2)

        print("\n--- Manual mode ---")
        print("Type a position in mm (0-100), or 'q' to quit.")
        while True:
            cmd = input("position mm> ").strip().lower()
            if cmd in ("q", "quit", "exit"):
                break
            try:
                mm = float(cmd)
            except ValueError:
                print("Enter a number 0-100 or 'q'.")
                continue
            pwm.ChangeDutyCycle(mm_to_duty(mm))
            print(f"Set to {max(0, min(100, mm)):.0f} mm")

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        pwm.stop()
        GPIO.cleanup()
        print("PWM stopped, GPIO cleaned up.")


if __name__ == "__main__":
    main()