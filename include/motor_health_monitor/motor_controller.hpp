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

#include <memory>
#include <string>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/subscription.hpp>

namespace motor_health_monitor
{

class MotorController {
public:
    virtual ~MotorController() = default;

    virtual bool configure(
        rclcpp_lifecycle::LifecycleNode* node,
        const std::string& topic_name,
        const std::string& message_type_override = "",
        const std::string& field_path_override = "") = 0;

    virtual void cleanup() = 0;

    virtual double getFeedbackValue() const = 0;

    virtual rclcpp::SubscriptionBase::SharedPtr getSubscription() const = 0;
};

}  // namespace motor_health_monitor