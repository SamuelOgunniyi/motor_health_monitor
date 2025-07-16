#ifndef ROBOT_DRIVE_MOTOR_CONTROLLER_H
#define ROBOT_DRIVE_MOTOR_CONTROLLER_H

#include "MotorController.h"

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

#endif // ROBOT_DRIVE_MOTOR_CONTROLLER_H
