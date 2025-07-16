// MotorController.h
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

class MotorController {
public:
    virtual ~MotorController() = default;

    // Set motor command (e.g., PWM, speed, etc.)
    virtual void setCommand(double cmd) = 0;

    // Get feedback value for diagnostics or control (e.g., actual speed, encoder reading)
    virtual double getFeedbackValue() const = 0;
};

#endif // MOTOR_CONTROLLER_H

