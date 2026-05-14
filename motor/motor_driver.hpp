#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

// Abstract base class for motor driver backends.
// Concrete implementations: PwmMotorDriver, UartMotorDriver
class MotorDriver {
public:
    virtual ~MotorDriver() = default;

    // Differential drive: positive = forward, negative = backward, range [-100, 100]
    virtual void drive(int left_speed, int right_speed) = 0;

    // Both wheels locked (active brake)
    virtual void brake() = 0;

    // All outputs disabled (coast)
    virtual void standby() = 0;
};

#endif // MOTOR_DRIVER_HPP
