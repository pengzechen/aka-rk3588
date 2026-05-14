// Author: Zhihang Shao <dio_ro@outlook.com>
// Source: aka0-ref (aka-sg2002/motor.cpp) - ported to rk3588 PWM abstraction layer
#include "pwm_motor_driver.hpp"
#include <fstream>
#include <string>
#include <unistd.h>

PwmMotorDriver::PwmMotorDriver(const std::string& pwm_path)
    : PWM_PATH(pwm_path)
{
    init_pwm(LEFT_WHEEL_BACKWARD);
    init_pwm(LEFT_WHEEL_FORWARD);
    init_pwm(RIGHT_WHEEL_BACKWARD);
    init_pwm(RIGHT_WHEEL_FORWARD);
}

PwmMotorDriver::~PwmMotorDriver()
{
    standby();
}

void PwmMotorDriver::init_pwm(int pwm_id)
{
    std::ofstream ofs_export(PWM_PATH + "export");
    if (ofs_export.is_open()) {
        ofs_export << pwm_id;
        ofs_export.close();
    }

    std::string pwm_channel_path = PWM_PATH + "pwm" + std::to_string(pwm_id);
    std::ofstream ofs_period(pwm_channel_path + "/period");
    if (ofs_period.is_open()) {
        ofs_period << PERIOD;
        ofs_period.close();
    }
}

void PwmMotorDriver::set_pwm_duty_cycle(int pwm_id, int duty_cycle)
{
    std::string path = PWM_PATH + "pwm" + std::to_string(pwm_id) + "/duty_cycle";
    std::ofstream ofs(path);
    if (ofs.is_open()) {
        ofs << duty_cycle;
        ofs.close();
    }
}

void PwmMotorDriver::set_pwm_enable(int pwm_id, bool enable)
{
    std::string path = PWM_PATH + "pwm" + std::to_string(pwm_id) + "/enable";
    std::ofstream ofs(path);
    if (ofs.is_open()) {
        ofs << (enable ? "1" : "0");
        ofs.close();
    }
}

void PwmMotorDriver::set_speed(int pwm_id, int speed)
{
    if (speed < 0)   speed = 0;
    if (speed > 100) speed = 100;
    int duty_cycle = PERIOD - (speed / 100.0) * PERIOD;
    set_pwm_duty_cycle(pwm_id, duty_cycle);
}

// Author: Zhihang Shao <dio_ro@outlook.com>
// Source: aka0-ref commits 755a885, 9c69f3f
// 差速驱动实现，commit 9c69f3f: 右轮速度补偿+2修正偏差
void PwmMotorDriver::drive(int left_speed, int right_speed)
{
    // 左轮
    if (left_speed > 0) {
        set_speed(LEFT_WHEEL_FORWARD, left_speed);
        set_pwm_enable(LEFT_WHEEL_FORWARD, true);
        set_pwm_enable(LEFT_WHEEL_BACKWARD, false);
    } else if (left_speed < 0) {
        set_speed(LEFT_WHEEL_BACKWARD, -left_speed);
        set_pwm_enable(LEFT_WHEEL_BACKWARD, true);
        set_pwm_enable(LEFT_WHEEL_FORWARD, false);
    } else {
        set_pwm_enable(LEFT_WHEEL_FORWARD, false);
        set_pwm_enable(LEFT_WHEEL_BACKWARD, false);
    }
    // 右轮（带+2速度补偿修正偏差）
    if (right_speed > 0) {
        set_speed(RIGHT_WHEEL_FORWARD, right_speed + 2);
        set_pwm_enable(RIGHT_WHEEL_FORWARD, true);
        set_pwm_enable(RIGHT_WHEEL_BACKWARD, false);
    } else if (right_speed < 0) {
        set_speed(RIGHT_WHEEL_BACKWARD, -right_speed + 2);
        set_pwm_enable(RIGHT_WHEEL_BACKWARD, true);
        set_pwm_enable(RIGHT_WHEEL_FORWARD, false);
    } else {
        set_pwm_enable(RIGHT_WHEEL_FORWARD, false);
        set_pwm_enable(RIGHT_WHEEL_BACKWARD, false);
    }
}

void PwmMotorDriver::brake()
{
    set_pwm_enable(LEFT_WHEEL_FORWARD,   true);
    set_pwm_enable(LEFT_WHEEL_BACKWARD,  true);
    set_pwm_enable(RIGHT_WHEEL_FORWARD,  true);
    set_pwm_enable(RIGHT_WHEEL_BACKWARD, true);
}

void PwmMotorDriver::standby()
{
    set_pwm_enable(LEFT_WHEEL_FORWARD,   false);
    set_pwm_enable(LEFT_WHEEL_BACKWARD,  false);
    set_pwm_enable(RIGHT_WHEEL_FORWARD,  false);
    set_pwm_enable(RIGHT_WHEEL_BACKWARD, false);
}
