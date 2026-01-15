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

#include <chrono>
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

protected:
  void update();

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  std::pair<bool, bool> checkStaleData(
    std::chrono::steady_clock::time_point current_time) const;
  std::tuple<bool, std::string, double> updateFaultDetection(
    std::chrono::steady_clock::time_point current_time);
  std::pair<int8_t, std::string> determineDiagnosticLevel(
    bool fault, const std::string& fault_reason, bool cmd_stale, bool odom_stale) const;
  diagnostic_msgs::msg::DiagnosticStatus buildDiagnosticStatus(
    bool fault, double feedback_value, bool cmd_stale, bool odom_stale,
    std::chrono::steady_clock::time_point current_time) const;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg_;
  nav_msgs::msg::Odometry::SharedPtr odom_msg_;
  std::chrono::steady_clock::time_point cmd_vel_timestamp_;
  std::chrono::steady_clock::time_point odom_timestamp_;
  double stale_data_timeout_;

  std::unique_ptr<MotorController> motor_controller_;
  CmdOdomSync sync_;
  std::unique_ptr<MotorFaultDetector> fault_detector_;
};

}  // namespace motor_health_monitor
