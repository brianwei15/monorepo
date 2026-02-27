#include <rclcpp/rclcpp.hpp>
#include <lgpio.h>

#include <chrono>
#include <cstdio>
#include <thread>

#include "payload/motor.hpp"
#include "payload/servo.hpp"

// ---- Pins from payload_params.yaml ----
static constexpr int AIN1  = 16;
static constexpr int AIN2 = 13;
static constexpr int BIN1  = 18;
static constexpr int BIN2 = 15;
constexpr int SERVO = 14;
constexpr float FREQ = 200.0; //200 Hz

static void pause(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// ---- Test sequences ----

// ---- Main ----
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Open gpiochip — try RPi 5 (chip4) then RPi 4 (chip0)
    int h = -1;
    for (int chip : {4, 0}) {
        h = lgGpiochipOpen(chip);
        if (h >= 0) { printf("Opened gpiochip%d (handle=%d)\n", chip, h); break; }
        printf("gpiochip%d failed: %d\n", chip, h);
    }
    if (h < 0) {
        printf("ERROR: Could not open any gpiochip\n");
        rclcpp::shutdown();
        return 1;
    }

    // // Claim output pins before handing to Motor
    // for (auto [label, pin] : {std::pair{"AIN1",  AIN1},
    //                            std::pair{"AIN2", AIN2},
    //                            std::pair{"BIN1",  BIN1},
    //                            std::pair{"BIN2", BIN2}}) {
    //     int rc = lgGpioClaimOutput(h, 0, pin, 0);
    //     printf("  ClaimOutput %-8s (BCM%2d): rc=%d\n", label, pin, rc);
    // }

    Motor motor_a(h, AIN1, AIN2, FREQ, MotorType::RIGHT);
    // Motor motor_b(h, BPHASE, BENABLE);

    std::cout << "MOTOR FORWARD 70%" << std::endl;
    motor_a.forward(70.0f);
    pause(2000);

    std::cout << "MOTOR FORWARD 20%" << std::endl;
    motor_a.forward(20.0f);
    pause(2000);


    std::cout << "MOTOR REVERSE 100%" << std::endl;
    motor_a.reverse(100.0f);
    pause(2000);

    std::cout << "MOTOR REVERSE 50%" << std::endl;
    motor_a.reverse(50.0f);
    pause(2000);
    // motor_a.set_speed(-0.2f);
    // pause(2000);

    Motor motor_b(h, BIN1, BIN2, FREQ, MotorType::LEFT);

    std::cout << "MOTOR B FORWARD 70%" << std::endl;
    motor_b.forward(70.0f);
    pause(2000);

    std::cout << "MOTOR B FORWARD 20%" << std::endl;
    motor_b.forward(20.0f);
    pause(2000);


    std::cout << "MOTOR B REVERSE 100%" << std::endl;
    motor_b.reverse(100.0f);
    pause(2000);

    std::cout << "MOTOR B REVERSE 50%" << std::endl;
    motor_b.reverse(50.0f);
    pause(2000);



    motor_a.set_speed(0.0f);
    motor_b.set_speed(0.0f);

    Servo servo(h, SERVO, 200);
    servo.degree_setpoint(35.0f);
    pause(2000);
    servo.degree_setpoint(0.0f);

    printf("\nDone.\n");

    pause(500);
    lgGpiochipClose(h);
    rclcpp::shutdown();
    return 0;
}
