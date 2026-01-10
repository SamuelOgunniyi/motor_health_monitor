#!/usr/bin/env python3

# Copyright 2025 Samuel Ogunniyi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Test script for motor_health_monitor node.

This script:
1. Publishes test cmd_vel and odom messages
2. Subscribes to diagnostics topic and prints messages
3. Simulates various scenarios (normal operation, sync issues, faults)
"""

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node


class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.publish_messages)

        self.cmd_vel = 0.0
        self.odom_vel = 0.0
        self.scenario = 0
        self.time_elapsed = 0.0

    def publish_messages(self):
        # Publish cmd_vel
        cmd_msg = Twist()
        cmd_msg.linear.x = self.cmd_vel
        cmd_msg.linear.y = 0.0
        cmd_msg.linear.z = 0.0
        cmd_msg.angular.x = 0.0
        cmd_msg.angular.y = 0.0
        cmd_msg.angular.z = 0.0
        self.cmd_pub.publish(cmd_msg)

        # Publish odom
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.twist.twist.linear.x = self.odom_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        self.odom_pub.publish(odom_msg)

        self.time_elapsed += 0.1

        # Run different scenarios
        self.run_scenario()

    def run_scenario(self):
        """Run different test scenarios."""
        if self.scenario == 0:
            # Scenario 1: Normal operation - velocities match
            if self.time_elapsed < 3.0:
                self.cmd_vel = 1.0
                self.odom_vel = 1.0
            elif self.time_elapsed < 6.0:
                self.cmd_vel = 0.5
                self.odom_vel = 0.5
            else:
                self.scenario = 1
                self.time_elapsed = 0.0
                self.get_logger().info('=== Scenario 2: Sync issue ===')

        elif self.scenario == 1:
            # Scenario 2: Sync issue - odometry lags behind command
            if self.time_elapsed < 3.0:
                self.cmd_vel = 1.0
                self.odom_vel = 0.3  # Large difference
            else:
                self.scenario = 2
                self.time_elapsed = 0.0
                self.get_logger().info('=== Scenario 3: Fault condition ===')

        elif self.scenario == 2:
            # Scenario 3: Fault condition - sustained large error
            if self.time_elapsed < 5.0:
                self.cmd_vel = 1.0
                self.odom_vel = 0.0  # Motor not responding
            else:
                self.scenario = 3
                self.time_elapsed = 0.0
                self.get_logger().info('=== Scenario 4: Recovery ===')

        elif self.scenario == 3:
            # Scenario 4: Recovery - back to normal
            if self.time_elapsed < 3.0:
                self.cmd_vel = 0.5
                self.odom_vel = 0.5
            else:
                self.get_logger().info('=== Test complete ===')
                self.scenario = 4


class DiagnosticsSubscriber(Node):
    def __init__(self):
        super().__init__('diagnostics_subscriber')
        self.subscription = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10
        )
        self.last_status = None

    def diagnostics_callback(self, msg):
        if not msg.status:
            return

        for status in msg.status:
            if status.name == 'drive_motor':
                # Only print if status changed
                if self.last_status != status.level:
                    level_str = self.level_to_string(status.level)
                    self.get_logger().info(
                        f"\n{'='*60}\n"
                        f'Diagnostic Status Update:\n'
                        f'  Name: {status.name}\n'
                        f'  Hardware ID: {status.hardware_id}\n'
                        f'  Level: {level_str} ({status.level})\n'
                        f'  Message: {status.message}\n'
                        f'  Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}\n'
                        f"{'='*60}\n"
                    )
                    self.last_status = status.level
                else:
                    # Print periodic updates even if status hasn't changed
                    level_str = self.level_to_string(status.level)
                    self.get_logger().info(
                        f'[{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}] '
                        f'{level_str}: {status.message}'
                    )

    @staticmethod
    def level_to_string(level):
        if level == 0:
            return 'OK'
        elif level == 1:
            return 'WARN'
        elif level == 2:
            return 'ERROR'
        elif level == 3:
            return 'STALE'
        else:
            return f'UNKNOWN({level})'


def main(args=None):
    rclpy.init(args=args)

    print('\n' + '='*60)
    print('Motor Health Monitor Test Script')
    print('='*60)
    print('\nThis script will:')
    print('  1. Publish test cmd_vel and odom messages')
    print('  2. Monitor and display diagnostic messages')
    print('  3. Run through several test scenarios')
    print('\nScenarios:')
    print('  - Normal operation (matching velocities)')
    print('  - Sync issue (odometry lagging)')
    print('  - Fault condition (motor not responding)')
    print('  - Recovery (back to normal)')
    print('\nPress Ctrl+C to stop\n')
    print('='*60 + '\n')

    try:
        publisher = TestPublisher()
        subscriber = DiagnosticsSubscriber()

        # Run both nodes in the same executor
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(publisher)
        executor.add_node(subscriber)

        print('Starting test...\n')
        print('=== Scenario 1: Normal operation ===\n')

        executor.spin()

    except KeyboardInterrupt:
        print('\n\nTest interrupted by user')
    except Exception as e:
        print(f'\nError: {e}')
        import traceback
        traceback.print_exc()
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
