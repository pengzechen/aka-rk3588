#include "motor.hpp"
#include "pwm_motor_driver.hpp"
#include "uart_motor_driver.hpp"

Motor::Motor(MotorDriverType type, const std::string& device)
{
    switch (type) {
        case MotorDriverType::PWM:
            driver_ = std::make_unique<PwmMotorDriver>(device);
            break;
        case MotorDriverType::UART:
        default:
            driver_ = std::make_unique<UartMotorDriver>(device);
            break;
    }
}

Motor::~Motor() = default;

void Motor::forward(int speed)
{
    driver_->drive(speed, speed);
}

void Motor::backward(int speed)
{
    driver_->drive(-speed, -speed);
}

void Motor::left(int speed)
{
    // Spin left: right wheel forward, left wheel backward
    driver_->drive(-speed, speed);
}

void Motor::right(int speed)
{
    // Spin right: left wheel forward, right wheel backward
    driver_->drive(speed, -speed);
}

void Motor::brake()
{
    driver_->brake();
}

void Motor::standby()
{
    driver_->standby();
}

// Author: Zhihang Shao <dio_ro@outlook.com>
// Source: aka0-ref commits 755a885, 9c69f3f
void Motor::drive(int left_speed, int right_speed)
{
    driver_->drive(left_speed, right_speed);
}
