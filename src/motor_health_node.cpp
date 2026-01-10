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

#include "motor_health_monitor/motor_health_node.hpp"

#include <chrono>

#include <lifecycle_msgs/msg/state.hpp>

#include "motor_health_monitor/robot_drive_motor_controller.hpp"

using std::placeholders::_1;

namespace motor_health_monitor
{

MotorHealthNode::MotorHealthNode(const rclcpp::NodeOptions & options)
: LifecycleNode("motor_health_monitor", options)
{
  declare_parameter("sync_tolerance", 0.05);
  declare_parameter("update_rate_ms", 100);
  declare_parameter("fault_threshold", 0.95);
  declare_parameter("fault_duration", 0.2);
  declare_parameter("fault_threshold_type", "absolute_value");
  declare_parameter("pwm_topic", "motor_pwm");
  declare_parameter("motor_controller_type", "robot_drive");
  declare_parameter("motor_message_type", "");
  declare_parameter("motor_pwm_field", "");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotorHealthNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring Motor Health Monitor");

  std::string controller_type = get_parameter("motor_controller_type").as_string();
  std::string pwm_topic = get_parameter("pwm_topic").as_string();
  std::string msg_type_override = get_parameter("motor_message_type").as_string();
  std::string field_path_override = get_parameter("motor_pwm_field").as_string();

  auto& factory = MotorControllerFactory::getInstance();
  motor_controller_ = factory.create(controller_type);

  if (!motor_controller_) {
    auto registered_types = factory.getRegisteredTypes();
    std::string types_list;
    for (size_t i = 0; i < registered_types.size(); ++i) {
      if (i > 0) types_list += ", ";
      types_list += "'" + registered_types[i] + "'";
    }

    RCLCPP_ERROR(get_logger(),
                 "Unknown motor controller type: '%s'. Registered types: %s",
                 controller_type.c_str(), types_list.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Using motor controller type: '%s'", controller_type.c_str());

  if (!motor_controller_->configure(this, pwm_topic, msg_type_override, field_path_override)) {
    RCLCPP_ERROR(get_logger(), "Failed to configure motor controller");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  FaultDetectionConfig fault_config;
  fault_config.threshold = get_parameter("fault_threshold").as_double();
  fault_config.duration = get_parameter("fault_duration").as_double();

  std::string threshold_type_str = get_parameter("fault_threshold_type").as_string();
  if (threshold_type_str == "upper_limit") {
    fault_config.type = ThresholdType::UPPER_LIMIT;
  } else if (threshold_type_str == "lower_limit") {
    fault_config.type = ThresholdType::LOWER_LIMIT;
  } else {
    fault_config.type = ThresholdType::ABSOLUTE_VALUE;
  }

  fault_detector_ = std::make_unique<MotorFaultDetector>(fault_config, *motor_controller_);

  sync_ = CmdOdomSync(get_parameter("sync_tolerance").as_double());

  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&MotorHealthNode::cmdCallback, this, _1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&MotorHealthNode::odomCallback, this, _1));

  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(get_parameter("update_rate_ms").as_int()),
    std::bind(&MotorHealthNode::update, this));
  timer_->cancel();

  RCLCPP_INFO(get_logger(), "Motor Health Monitor configured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotorHealthNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating Motor Health Monitor");

  diag_pub_->on_activate();
  timer_->reset();

  RCLCPP_INFO(get_logger(), "Motor Health Monitor activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotorHealthNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating Motor Health Monitor");

  timer_->cancel();
  diag_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Motor Health Monitor deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotorHealthNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up Motor Health Monitor");

  cmd_sub_.reset();
  odom_sub_.reset();
  if (motor_controller_) {
    motor_controller_->cleanup();
  }
  diag_pub_.reset();
  timer_.reset();
  motor_controller_.reset();
  fault_detector_.reset();
  
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cmd_vel_msg_.reset();
    odom_msg_.reset();
  }

  RCLCPP_INFO(get_logger(), "Motor Health Monitor cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotorHealthNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting down Motor Health Monitor (state: %s)",
      state.label().c_str());

  cmd_sub_.reset();
  odom_sub_.reset();
  if (motor_controller_) {
    motor_controller_->cleanup();
  }
  diag_pub_.reset();
  timer_.reset();
  motor_controller_.reset();
  fault_detector_.reset();
  
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cmd_vel_msg_.reset();
    odom_msg_.reset();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void MotorHealthNode::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  cmd_vel_msg_ = msg;
}

void MotorHealthNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  odom_msg_ = msg;
}

void MotorHealthNode::update()
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  geometry_msgs::msg::Twist::SharedPtr cmd_msg;
  nav_msgs::msg::Odometry::SharedPtr odom_msg;
  bool fault;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cmd_msg = cmd_vel_msg_;
    odom_msg = odom_msg_;

    auto current_time = now().seconds();
    if (fault_detector_) {
      fault_detector_->update(current_time);
      fault = fault_detector_->getState() == MotorFaultState::FAULT;
    } else {
      fault = false;
    }
  }

  double cmd_vel = cmd_msg ? cmd_msg->linear.x : 0.0;
  double odom_vel = odom_msg ? odom_msg->twist.twist.linear.x : 0.0;
  sync_.update(cmd_vel, odom_vel);

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "drive_motor";
  status.hardware_id = "drive_motor_0";
  status.level = fault ?
    diagnostic_msgs::msg::DiagnosticStatus::ERROR :
    diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = fault ? "Motor fault detected" : "Motor healthy";

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(status);

  if (diag_pub_->is_activated()) {
    diag_pub_->publish(array);
  }
}

}  // namespace motor_health_monitor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(motor_health_monitor::MotorHealthNode)
