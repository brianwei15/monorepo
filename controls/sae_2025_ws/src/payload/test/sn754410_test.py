import lgpio
import time

# ----------------------
# Pin Definitions (BCM)
# ----------------------

A_PWM = 12
A_IN1 = 5
A_IN2 = 6

B_PWM = 13
B_IN1 = 20
B_IN2 = 21

PWM_FREQ = 1000  # Hz
PWM_DUTY = 50    # Percent (0–100)

# ----------------------
# Setup
# ----------------------

h = lgpio.gpiochip_open(0)

# Claim direction pins as outputs
for pin in [A_IN1, A_IN2, B_IN1, B_IN2]:
    lgpio.gpio_claim_output(h, pin)

# Claim PWM pins as outputs (needed before PWM)
for pin in [A_PWM, B_PWM]:
    lgpio.gpio_claim_output(h, pin)

# ----------------------
# Motor Control Functions
# ----------------------

def motor_forward(pwm, in1, in2):
    lgpio.gpio_write(h, in1, 1)
    lgpio.gpio_write(h, in2, 0)
    lgpio.tx_pwm(h, pwm, PWM_FREQ, PWM_DUTY)

def motor_reverse(pwm, in1, in2):
    lgpio.gpio_write(h, in1, 0)
    lgpio.gpio_write(h, in2, 1)
    lgpio.tx_pwm(h, pwm, PWM_FREQ, PWM_DUTY)

def motor_stop(pwm, in1, in2):
    lgpio.tx_pwm(h, pwm, PWM_FREQ, 0)  # stop PWM
    lgpio.gpio_write(h, in1, 0)
    lgpio.gpio_write(h, in2, 0)        # coast

# ----------------------
# Test Sequence
# ----------------------

try:
    print("Motor A forward")
    motor_forward(A_PWM, A_IN1, A_IN2)
    time.sleep(2)

    print("Motor A reverse")
    motor_reverse(A_PWM, A_IN1, A_IN2)
    time.sleep(2)

    print("Motor A stop")
    motor_stop(A_PWM, A_IN1, A_IN2)
    time.sleep(1)

    print("Motor B forward")
    motor_forward(B_PWM, B_IN1, B_IN2)
    time.sleep(2)

    print("Motor B reverse")
    motor_reverse(B_PWM, B_IN1, B_IN2)
    time.sleep(2)

    print("Motor B stop")
    motor_stop(B_PWM, B_IN1, B_IN2)

finally:
    # Cleanup
    lgpio.tx_pwm(h, A_PWM, 0, 0)
    lgpio.tx_pwm(h, B_PWM, 0, 0)
    lgpio.gpiochip_close(h)
    print("Done.")