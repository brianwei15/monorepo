#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <memory>
#include "payload/gpio.hpp"
#include <algorithm>


enum class MotorType {
    LEFT,
    RIGHT
};

// PWM Controls based on DRV8835 driver with MODE=0
class Motor {
    public:
        Motor(int handle, int in1, int in2, float frequency, MotorType motor_type);
        void set_speed(float speed);// Normalized speed, -1.0 for max reverse, 1.0 for max forward
        void forward(float duty); // forward/coast mode
        void reverse(float duty); // reverse/coast mode
        void coast();
        void hard_brake();
    

    private:
        int handle_{};
        float frequency_{};
        std::unique_ptr<GPIO> in1_;   //binary phase_pin, 
        std::unique_ptr<GPIO> in2_;  //pwm only enable_pin
        MotorType motor_type_;


};


#endif