// Copyright 2025 Samuel Ogunniyi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Example: How to create and register a custom MotorController
//
// This file demonstrates how users can create their own MotorController
// implementations and register them to be used via YAML configuration.

#pragma once

#include <cmath>

#include "motor_health_monitor/motor_controller.hpp"
#include "motor_health_monitor/motor_controller_factory.hpp"

namespace my_robot
{

/// Example custom motor controller implementation
/// This could read from a custom ROS topic, hardware interface, etc.
class MyCustomMotorController : public motor_health_monitor::MotorController
{
public:
    MyCustomMotorController() : pwm_value_(0.0), max_pwm_(1.0) {}

    void setCommand(double cmd) override
    {
        // Store PWM command (could also subscribe to a ROS topic here)
        pwm_value_ = cmd;

        // Example: Normalize PWM if needed
        if (std::abs(pwm_value_) > max_pwm_) {
            pwm_value_ = (pwm_value_ > 0) ? max_pwm_ : -max_pwm_;
        }
    }

    double getFeedbackValue() const override
    {
        // Return absolute PWM value for fault detection
        // This is what MotorFaultDetector will use
        return std::abs(pwm_value_);
    }

    // Optional: Add custom methods for your specific needs
    void setMaxPWM(double max) { max_pwm_ = max; }  // NOLINT

private:
    double pwm_value_;
    double max_pwm_;
};

// Register your custom controller with the factory
// This makes it available via YAML: motor_controller_type: "my_custom"
REGISTER_MOTOR_CONTROLLER("my_custom", MyCustomMotorController)

}  // namespace my_robot

// Usage in YAML:
// motor_health_monitor:
//   ros__parameters:
//     motor_controller_type: "my_custom"  # Uses your custom controller
//     # ... other parameters
