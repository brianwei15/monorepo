#include "payload/motor.hpp"

Motor::Motor(int handle, int in1, int in2, int frequency, MotorType motor_type) 
: handle_(handle),
  frequency_(frequency),
  in1_(handle, in1, Direction::Output),
  in2_(handle, in2, Direction::Output),
  motor_type_(motor_type) { }

void Motor::set_speed(float speed) {
    float norm_speed = std::clamp<float>(speed, -1.0f, 1.0f);
    if (norm_speed != speed) {
        RCLCPP_WARN(rclcpp::get_logger("motor"), "Clamped speed from %f to %f", speed, norm_speed);
    }

    float pwm_duty_cycle = abs(norm_speed * 100.0f); //percentage
    if (norm_speed > 0) { //forward, set phase to 0
        forward(pwm_duty_cycle);
    } else if (norm_speed < 0){ //reverse
        reverse(pwm_duty_cycle);
    } else { //coast
        coast();
    }
}

void Motor::forward(float duty) {
    switch(motor_type_){
        case MotorType::LEFT:
            in1_.write_low();
            in2_.write_pwm(frequency_, duty, 0, 0);
            break;
        case MotorType::RIGHT:
            in1_.write_pwm(frequency_, duty, 0, 0);
            in2_.write_low();
            break;
    }
}

void Motor::reverse(float duty) {
    switch(motor_type_){
        case MotorType::LEFT:
            in1_.write_pwm(frequency_, duty, 0, 0);
            in2_.write_low();
            break;
        case MotorType::RIGHT:
            in1_.write_low();
            in2_.write_pwm(frequency_, duty, 0, 0);
            break;
    }
}

void Motor::coast() {
    in1_.write_pwm(frequency_, 0, 0, 0);
    in2_.write_pwm(frequency_, 0, 0, 0);
}

void Motor::hard_brake() {
    in1_.write_high();
    in2_.write_high();
}
    