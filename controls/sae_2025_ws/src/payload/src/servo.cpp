#include "payload/servo.hpp"

Servo::Servo(int handle, int pin, int frequency) : 
    handle_(handle), 
    frequency_(frequency), 
    servo_pin_(handle, pin, Direction::Output) { }

void Servo::degree_setpoint(float degree) {
    int clamped_degree = static_cast<int>(std::clamp<float>(degree, 0, 180));
    int pulse = angle_to_pulse(degree);
    servo_pin_.write_servo(pulse, frequency_, 0, 0); 
}


int Servo::angle_to_pulse(float degree) {
    int clamped_degree = static_cast<int>(std::clamp<float>(degree, 0, 180));
    return 1000 + (clamped_degree * 1000) / 180;
}