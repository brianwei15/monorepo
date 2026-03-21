#include "payload/servo.hpp"

Servo::Servo(int handle, int pin, int frequency) : 
    handle_(handle), 
    frequency_(frequency), 
    servo_pin_(handle, pin, Direction::Output) { }

void Servo::degree_setpoint(float degree) {
    int pulse = angle_to_pulse(degree);
    RCLCPP_DEBUG(logger(), "Outputting pulse: %d", pulse);
    servo_pin_.write_servo(pulse, frequency_, 0, 0); 
}


int Servo::angle_to_pulse(float degree) {
    float clamped = std::clamp(degree, 0.0f, 180.0f);
    return static_cast<int>(1000.0f + (clamped * 1000.0f) / 180.0f);
}


rclcpp::Logger Servo::logger() {
    return rclcpp::get_logger("servo");
}