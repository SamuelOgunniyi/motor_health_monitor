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

#pragma once

#include <rclcpp/subscription.hpp>

#include "motor_health_monitor/motor_controller.hpp"

namespace motor_health_monitor
{
namespace test
{

class MockMotorController : public MotorController {
public:
    MockMotorController() : feedback_value_(0.0) {}

    bool configure(
        rclcpp_lifecycle::LifecycleNode* node,
        const std::string& topic_name,
        const std::string& message_type_override = "",
        const std::string& field_path_override = "",
        int queue_depth = 1) override {
        (void)node;
        (void)topic_name;
        (void)message_type_override;
        (void)field_path_override;
        (void)queue_depth;
        return true;
    }

    void cleanup() override {}

    double getFeedbackValue() const override {
        return feedback_value_;
    }

    rclcpp::SubscriptionBase::SharedPtr getSubscription() const override {
        return nullptr;
    }

    void setFeedbackValue(double value) {
        feedback_value_ = value;
    }

private:
    double feedback_value_;
};

}  // namespace test
}  // namespace motor_health_monitor
