#!/usr/bin/env python3
"""
Legacy low-level motor test script.

This script drives two pins per motor and was used for previous wiring.
For SN754410 (PWM + IN1 + IN2), prefer `sn754410_test.py`.
"""

import lgpio
import time

# --- Pin assignments (BCM numbering) ---
A_IN1 = 16  # Motor A IN1 (PWM1)
A_IN2 = 13  # Motor A IN2

B_IN1 = 15  # Motor B IN1 (PWM0)
B_IN2 = 18  # Motor B IN2

PWM_HZ = 500
TEST_DUTY = 50.0  # 50%
RUN_SECS = 2.0


def coast(h, in1, in2):
    """Coast: both inputs LOW, outputs float."""
    lgpio.tx_pwm(h, in1, PWM_HZ, 0.0, 0, 0)
    lgpio.tx_pwm(h, in2, PWM_HZ, 0.0, 0, 0)


def forward(h, in1, in2, duty=TEST_DUTY):
    """Forward/coast: IN1=PWM, IN2=0."""
    lgpio.gpio_write(h, in2, 0)
    lgpio.tx_pwm(h, in1, PWM_HZ, duty, 0, 0)


def reverse(h, in1, in2, duty=TEST_DUTY):
    """Reverse/coast: IN1=0, IN2=PWM."""
    lgpio.gpio_write(h, in1, 0)
    lgpio.tx_pwm(h, in2, PWM_HZ, duty, 0, 0)


def coast_all(h):
    for pin in [A_IN1, A_IN2, B_IN1, B_IN2]:
        lgpio.tx_pwm(h, pin, PWM_HZ, 0.0, 0, 0)


# --- Open GPIO chip (try RPi 5 first, fall back to RPi 4) ---
h = None
for chip in [4, 0]:
    try:
        h = lgpio.gpiochip_open(chip)
        print(f"Opened gpiochip{chip} (handle={h})")
        break
    except lgpio.error as e:
        print(f"gpiochip{chip} failed: {e}")

if h is None:
    print("ERROR: Could not open any gpiochip — check /dev/gpiochip* permissions")
    exit(1)

# --- Claim all four pins as outputs, initially LOW ---
for label, pin in [
    ("A_IN1", A_IN1),
    ("A_IN2", A_IN2),
    ("B_IN1", B_IN1),
    ("B_IN2", B_IN2),
]:
    rc = lgpio.gpio_claim_output(h, pin, 0)
    print(f"  ClaimOutput {label} (BCM{pin}): rc={rc}")

print(f"\nPWM: {PWM_HZ} Hz  Duty: {TEST_DUTY}%  Duration: {RUN_SECS}s\n")

try:
    print("=== Motor A — FORWARD/coast (IN1=PWM, IN2=0) ===")
    forward(h, A_IN1, A_IN2)
    time.sleep(RUN_SECS)
    coast_all(h)
    time.sleep(0.5)

    print("=== Motor A — REVERSE/coast (IN1=0, IN2=PWM) ===")
    reverse(h, A_IN1, A_IN2)
    time.sleep(RUN_SECS)
    coast_all(h)
    time.sleep(0.5)

    print("=== Motor B — FORWARD/coast (IN1=PWM, IN2=0) ===")
    forward(h, B_IN1, B_IN2)
    time.sleep(RUN_SECS)
    coast_all(h)
    time.sleep(0.5)

    print("=== Motor B — REVERSE/coast (IN1=0, IN2=PWM) ===")
    reverse(h, B_IN1, B_IN2)
    time.sleep(RUN_SECS)
    coast_all(h)
    time.sleep(0.5)

    print("\nAll tests complete.")

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    coast_all(h)
    lgpio.gpiochip_close(h)
    print("GPIO closed.")
