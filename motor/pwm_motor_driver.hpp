#ifndef PWM_MOTOR_DRIVER_HPP
#define PWM_MOTOR_DRIVER_HPP

#include "motor_driver.hpp"
#include <string>

// PWM-based motor driver (ported from aka-sg2002).
// Controls four PWM channels: left/right × forward/backward.
// PWM sysfs path must be set to match the target board's pwmchip.
class PwmMotorDriver : public MotorDriver {
public:
    // pwm_path: e.g. "/sys/class/pwm/pwmchip0/"
    explicit PwmMotorDriver(const std::string& pwm_path = "/sys/class/pwm/pwmchip0/");
    ~PwmMotorDriver() override;

    void drive(int left_speed, int right_speed) override;
    void brake() override;
    void standby() override;

private:
    void init_pwm(int pwm_id);
    void set_pwm_duty_cycle(int pwm_id, int duty_cycle);
    void set_pwm_enable(int pwm_id, bool enable);
    void set_speed(int pwm_id, int speed);

    std::string PWM_PATH;
    const int PERIOD = 10000; // 10kHz

    // PWM channel IDs
    const int LEFT_WHEEL_BACKWARD  = 0;
    const int LEFT_WHEEL_FORWARD   = 1;
    const int RIGHT_WHEEL_BACKWARD = 2;
    const int RIGHT_WHEEL_FORWARD  = 3;
};

#endif // PWM_MOTOR_DRIVER_HPP
