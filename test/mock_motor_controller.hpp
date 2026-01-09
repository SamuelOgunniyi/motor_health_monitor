#pragma once

#include "motor_health_monitor/motor_controller.hpp"

namespace motor_health_monitor
{
namespace test
{

/// Mock MotorController for testing MotorFaultDetector
class MockMotorController : public MotorController {
public:
    MockMotorController() : feedback_value_(0.0) {}

    void setCommand(double cmd) override {
        (void)cmd;
    }

    double getFeedbackValue() const override {
        return feedback_value_;
    }

    void setFeedbackValue(double value) {
        feedback_value_ = value;
    }

private:
    double feedback_value_;
};

}  // namespace test
}  // namespace motor_health_monitor
