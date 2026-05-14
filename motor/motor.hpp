#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <memory>
#include <string>
#include "motor_driver.hpp"

// Selects the underlying motor driver implementation.
enum class MotorDriverType {
    PWM,   // sysfs PWM (legacy, 4-channel H-bridge)
    UART,  // ESP32-C3 UART controller
};

// High-level Motor API.
// Interface is identical to aka-sg2002/motor.hpp so tennis.cpp compiles unchanged.
//
// Usage:
//   Motor motor(MotorDriverType::UART, "/dev/ttyS3");
//   motor.forward(50);
class Motor {
public:
    // PWM constructor
    explicit Motor(MotorDriverType type = MotorDriverType::UART,
                   const std::string& device = "/dev/ttyS3");
    ~Motor();

    void forward(int speed);
    void backward(int speed);
    void left(int speed);
    void right(int speed);
    void brake();
    void standby();

    // Author: Zhihang Shao <dio_ro@outlook.com>
    // Source: aka0-ref commits 755a885, 9c69f3f
    // 差速驱动：正值前进，负值后退，范围 [-100, 100]
    void drive(int left_speed, int right_speed);

private:
    std::unique_ptr<MotorDriver> driver_;
};

#endif // MOTOR_HPP
