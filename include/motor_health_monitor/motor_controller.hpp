#pragma once

namespace motor_health_monitor
{

class MotorController {
public:
    virtual ~MotorController() = default;

    // Set motor command (e.g., PWM, speed, etc.)
    virtual void setCommand(double cmd) = 0;

    // Get feedback value for diagnostics or control (e.g., actual speed, encoder reading)
    virtual double getFeedbackValue() const = 0;
};

} // namespace motor_health_monitor

