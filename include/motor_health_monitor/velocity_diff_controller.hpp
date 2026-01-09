#pragma once

#include "motor_controller.hpp"
#include <cmath>

namespace motor_health_monitor
{

class VelocityDiffController : public MotorController {
public:
    VelocityDiffController() noexcept : cmd_velocity_(0.0), odom_velocity_(0.0) {}

    VelocityDiffController(const VelocityDiffController&) = default;
    VelocityDiffController& operator=(const VelocityDiffController&) = default;
    VelocityDiffController(VelocityDiffController&&) noexcept = default;
    VelocityDiffController& operator=(VelocityDiffController&&) noexcept = default;
    ~VelocityDiffController() override = default;

    void setCommand(double cmd) override {
        cmd_velocity_ = cmd;
    }

    double getFeedbackValue() const override {
        return std::abs(cmd_velocity_ - odom_velocity_);
    }

    void setOdomVelocity(double odom) noexcept {
        odom_velocity_ = odom;
    }

private:
    double cmd_velocity_;
    double odom_velocity_;
};

}  // namespace motor_health_monitor
