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

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "motor_health_monitor/cmd_odom_sync.hpp"
#include "motor_health_monitor/motor_controller.hpp"
#include "motor_health_monitor/motor_controller_factory.hpp"
#include "motor_health_monitor/motor_fault_detector.hpp"

namespace motor_health_monitor
{

class MotorHealthNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MotorHealthNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  MotorHealthNode(const MotorHealthNode&) = delete;
  MotorHealthNode& operator=(const MotorHealthNode&) = delete;
  MotorHealthNode(MotorHealthNode&&) = delete;
  MotorHealthNode& operator=(MotorHealthNode&&) = delete;

  ~MotorHealthNode() = default;

  // Lifecycle callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &);

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void update();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg_;
  nav_msgs::msg::Odometry::SharedPtr odom_msg_;

  std::unique_ptr<MotorController> motor_controller_;
  CmdOdomSync sync_;
  std::unique_ptr<MotorFaultDetector> fault_detector_;
};

}  // namespace motor_health_monitor
