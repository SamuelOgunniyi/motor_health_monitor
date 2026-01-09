#pragma once

#include "motor_controller.hpp"

namespace motor_health_monitor
{

class RobotDriveMotorController  : public MotorController {
public:
    explicit RobotDriveMotorController () : value_(0.0) {}

    void setCommand(double value) override {
        value_ = value;
    }

    double getFeedbackValue() const override {
        return value_;
    }

private:
    double value_;
};

}