#include <rclcpp/rclcpp.hpp>
#include <lgpio.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "payload/motor.hpp"
#include "payload/servo.hpp"

// ---- Pins from payload_params.yaml ----
// DRV8835 pins
static constexpr int DRV_AIN1 = 16;
static constexpr int DRV_AIN2 = 13;
static constexpr int DRV_BIN1 = 18;
static constexpr int DRV_BIN2 = 15;
// SN754410 pins
static constexpr int SN_A_PWM = 13;
static constexpr int SN_AIN1  = 16;
static constexpr int SN_AIN2  = 20;
static constexpr int SN_B_PWM = 18;
static constexpr int SN_BIN1  = 15;
static constexpr int SN_BIN2  = 14;

constexpr int SERVO = 14;
constexpr float FREQ = 200.0; //200 Hz

static void pause(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// ---- Signal handling ----
static int       g_handle  = -1;
static Motor*    g_motor_a = nullptr;
static Motor*    g_motor_b = nullptr;

static void on_sigint(int)
{
    printf("\nCaught SIGINT — coasting motors and cleaning up\n");
    if (g_motor_a) g_motor_a->coast();
    if (g_motor_b) g_motor_b->coast();
    if (g_handle >= 0) lgGpiochipClose(g_handle);
    rclcpp::shutdown();
    std::exit(0);
}

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
    g_handle = h;

    if (argc < 2) {
        printf("Usage: %s <0|1>  (0=DRVMotor, 1=SNMotor)\n", argv[0]);
        lgGpiochipClose(h);
        rclcpp::shutdown();
        return 1;
    }
    int test_mode = std::atoi(argv[1]);

    std::unique_ptr<Motor> motor_a;
    std::unique_ptr<Motor> motor_b;
    if (test_mode == 0) {
        printf("Running DRVMotor test\n");
        motor_a = std::make_unique<DRVMotor>(h, DRV_AIN1, DRV_AIN2, FREQ, MotorType::RIGHT);
        motor_b = std::make_unique<DRVMotor>(h, DRV_BIN1, DRV_BIN2, FREQ, MotorType::LEFT);
    } else if (test_mode == 1) {
        printf("Running SNMotor test\n");
        motor_a = std::make_unique<SNMotor>(h, SN_A_PWM, SN_AIN1, SN_AIN2, FREQ, MotorType::RIGHT);
        motor_b = std::make_unique<SNMotor>(h, SN_B_PWM, SN_BIN1, SN_BIN2, FREQ, MotorType::LEFT);
    } else {
        printf("ERROR: invalid argument '%s', expected 0 or 1\n", argv[1]);
        lgGpiochipClose(h);
        rclcpp::shutdown();
        return 1;
    }
    g_motor_a = motor_a.get();
    g_motor_b = motor_b.get();
    std::signal(SIGINT, on_sigint);

    std::cout << "MOTOR A FORWARD 70%" << std::endl;
    motor_a->forward(70.0f);
    pause(1000);

    std::cout << "MOTOR A FORWARD 20%" << std::endl;
    motor_a->forward(20.0f);
    pause(1000);


    std::cout << "MOTOR A REVERSE 100%" << std::endl;
    motor_a->reverse(100.0f);
    pause(1000);

    std::cout << "MOTOR A REVERSE 50%" << std::endl;
    motor_a->reverse(50.0f);
    pause(1000);

    std::cout << "MOTOR A STOPPING:" << std::endl;
    motor_a->coast();
    pause(1000);

    std::cout << "MOTOR B FORWARD 70%" << std::endl;
    motor_b->forward(70.0f);
    pause(1000);

    std::cout << "MOTOR B FORWARD 20%" << std::endl;
    motor_b->forward(20.0f);
    pause(1000);


    std::cout << "MOTOR B REVERSE 100%" << std::endl;
    motor_b->reverse(100.0f);
    pause(1000);

    std::cout << "MOTOR B REVERSE 50%" << std::endl;
    motor_b->reverse(50.0f);
    pause(1000);

    std::cout << "Stopping motors" << std::endl;
    motor_a->coast();
    motor_b->coast();
    pause(1000);

    std::cout << "BOTH FORWARD" << std::endl;
    motor_a->forward(100.0f);
    motor_b->forward(100.0f);
    pause(1000);


    std::cout << "BOTH FORWARD" << std::endl;
    motor_a->reverse(100.0f);
    motor_b->reverse(100.0f);
    pause(1000);

    printf("\nDone.\n");

    pause(500);
    lgGpiochipClose(h);
    rclcpp::shutdown();
    return 0;
}
