#include "payload/gpio.hpp"
#include <lgpio.h>

GPIO::GPIO(int handle, int pin, Direction direction) {
    handle_ = handle;
    pin_ = pin;
    direction_ = direction;
    switch(direction_){
        case Direction::Input:
            int rc = lgGpioClaimInput(handle_, 0, pin_);
            break;
        case Direction::Output:
            int rc = lgGpioClaimOutput(handle_, 0, pin_, 0);
            break;
    }
}

void GPIO::write_high() {
    if (direction_ != Direction::Output) {
        RCLCPP_WARN(logger(), "Illegal Binary HIGH write on pin %d", pin_);
        return;
    }
    int rc = lgGpioWrite(handle_, pin_, 1);
    if (rc < 0) RCLCPP_WARN(logger(), "lgGpioWrite HIGH failed on pin %d: rc=%d", pin_, rc);
}

void GPIO::write_low() {
    if (direction_ != Direction::Output) {
        RCLCPP_WARN(logger(), "Illegal Binary LOW write on pin %d", pin_);
        return;
    }
    int rc = lgGpioWrite(handle_, pin_, 0);
    if (rc < 0) RCLCPP_WARN(logger(), "lgGpioWrite LOW failed on pin %d: rc=%d", pin_, rc);
}

void GPIO::write_pwm(float frequency, float duty_cycle, int offset, int cycles) {
    if (direction_ != Direction::Output) {
        RCLCPP_WARN(logger(), "Illegal PWM write on pin %d", pin_);
        return;
    }
    //for continuous and immediate, set offset and cycles to 0
    int rc = lgTxPwm(handle_, pin_, frequency, duty_cycle, offset, cycles);
    if (rc < 0) RCLCPP_WARN(logger(), "lgTxPwm failed on pin %d: rc=%d", pin_, rc);
}

void GPIO::write_servo(int pulse_width, int frequency, int offset, int cycles) {
    if (direction_ != Direction::Output) {
        RCLCPP_WARN(logger(), "Illegal servo write on pin %d", pin_);
        return;
    }
    int rc = lgTxServo(handle_, pin_, pulse_width, frequency, offset, cycles);
    if (rc < 0) RCLCPP_WARN(logger(), "lgTxServo failed on pin %d: rc=%d", pin_, rc);
}

rclcpp::Logger GPIO::logger() {
    return rclcpp::get_logger("gpio");
}