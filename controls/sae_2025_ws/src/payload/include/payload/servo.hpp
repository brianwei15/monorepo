#ifndef SERVO_HPP
#define SERVO_HPP

#include "payload/gpio.hpp"

class Servo {
    public:
        Servo(int handle, int pin, int frequency);
        void degree_setpoint(float degree);
    
    private:
        int handle_{};
        int frequency_{};
        GPIO servo_pin_;

        int angle_to_pulse(float degree);
};


#endif