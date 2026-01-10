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

#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialization.hpp>
#include <std_msgs/msg/float64.hpp>

#include "motor_controller.hpp"
#include "motor_controller_factory.hpp"

namespace motor_health_monitor
{

class RobotDriveMotorController : public MotorController {
public:
    RobotDriveMotorController() : node_(nullptr), pwm_value_(0.0) {}

    RobotDriveMotorController(const RobotDriveMotorController&) = delete;
    RobotDriveMotorController& operator=(const RobotDriveMotorController&) = delete;
    RobotDriveMotorController(RobotDriveMotorController&&) = delete;
    RobotDriveMotorController& operator=(RobotDriveMotorController&&) = delete;
    ~RobotDriveMotorController() override = default;

    bool configure(
        rclcpp_lifecycle::LifecycleNode* node,
        const std::string& topic_name,
        const std::string& message_type_override = "",
        const std::string& field_path_override = "") override {
        if (!node) {
            return false;
        }

        node_ = node;
        topic_name_ = topic_name;
        field_path_ = field_path_override.empty() ? "data" : field_path_override;

        std::string msg_type = message_type_override;

        if (msg_type.empty()) {
            msg_type = introspectTopicType(topic_name);
            if (msg_type.empty()) {
                msg_type = "std_msgs/msg/Float64";
                if (node_) {
                    RCLCPP_WARN(node_->get_logger(),
                               "Could not introspect topic '%s', using default: %s",
                               topic_name.c_str(), msg_type.c_str());
                }
            } else {
                if (node_) {
                    RCLCPP_INFO(node_->get_logger(),
                               "Auto-detected message type for '%s': %s",
                               topic_name.c_str(), msg_type.c_str());
                }
            }
        }

        if (!node_) {
            return false;
        }

        subscription_ = node_->create_generic_subscription(
            topic_name,
            msg_type,
            rclcpp::QoS(10),
            [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                this->messageCallback(msg);
            }
        );

        if (node_) {
            RCLCPP_INFO(node_->get_logger(),
                       "Subscribed to '%s' (type: %s, field: %s)",
                       topic_name.c_str(), msg_type.c_str(), field_path_.c_str());
        }

        return true;
    }

    void cleanup() override {
        subscription_.reset();
        node_ = nullptr;
    }

    double getFeedbackValue() const override {
        return pwm_value_;
    }

    rclcpp::SubscriptionBase::SharedPtr getSubscription() const override {
        return subscription_;
    }

private:
    void messageCallback(std::shared_ptr<rclcpp::SerializedMessage> msg) {
        if (!msg) {
            return;
        }

        if (field_path_ == "data") {
            std_msgs::msg::Float64 pwm_msg;
            rclcpp::Serialization<std_msgs::msg::Float64> serializer;
            serializer.deserialize_message(msg.get(), &pwm_msg);
            pwm_value_ = pwm_msg.data;
        }
    }

    std::string introspectTopicType(const std::string& topic_name) {
        if (!node_) {
            return "";
        }

        auto topic_names_and_types = node_->get_topic_names_and_types();
        for (const auto& [name, types] : topic_names_and_types) {
            if (name == topic_name && !types.empty()) {
                return types[0];
            }
        }
        return "";
    }

    rclcpp_lifecycle::LifecycleNode* node_;
    rclcpp::SubscriptionBase::SharedPtr subscription_;
    std::string topic_name_;
    std::string field_path_;
    double pwm_value_;
};

REGISTER_MOTOR_CONTROLLER("robot_drive", RobotDriveMotorController)

}  // namespace motor_health_monitor