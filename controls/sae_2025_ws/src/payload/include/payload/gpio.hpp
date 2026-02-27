#ifndef GPIO_HPP
#define GPIO_HPP

#include <rclcpp/rclcpp.hpp>


enum class Direction {
    Input,
    Output
};

class GPIO {
    public:
        GPIO(int handle, int pin, Direction direction);
        void write_high();
        void write_low();
        void write_pwm(float frequency, float duty_cycle, int offset, int cycles);
        void write_servo(int pulse_width, int frequency, int offset, int cycles);


    private:
        int handle_{}; //handle for gpiochip
        int pin_{};
        Direction direction_;
        
        static rclcpp::Logger logger();
        bool allowed();

};

#endif