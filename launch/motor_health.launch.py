from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('motor_health_monitor')
    config_file = os.path.join(package_dir, 'config', 'motor_health.yaml')
    
    return LaunchDescription([
        ComposableNodeContainer(
            name='motor_health_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='motor_health_monitor',
                    plugin='motor_health_monitor::MotorHealthNode',
                    name='motor_health_monitor',
                    parameters=[config_file] if os.path.exists(config_file) else []
                )
            ],
            output='screen'
        )
    ])
