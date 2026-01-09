#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mutex>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "motor_health_monitor/cmd_odom_sync.hpp"
#include "motor_health_monitor/motor_fault_detector.hpp"
#include "motor_health_monitor/velocity_diff_controller.hpp"

namespace motor_health_monitor
{

class MotorHealthNode : public rclcpp::Node
{
public:
  explicit MotorHealthNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  
  MotorHealthNode(const MotorHealthNode&) = delete;
  MotorHealthNode& operator=(const MotorHealthNode&) = delete;
  MotorHealthNode(MotorHealthNode&&) = delete;
  MotorHealthNode& operator=(MotorHealthNode&&) = delete;

  ~MotorHealthNode() = default;

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void update();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex data_mutex_;
  geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg_;
  nav_msgs::msg::Odometry::SharedPtr odom_msg_;

  VelocityDiffController motor_controller_;
  CmdOdomSync sync_;
  MotorFaultDetector fault_detector_;
};

}  // namespace motor_health_monitor
