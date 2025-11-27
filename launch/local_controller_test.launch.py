from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lamapf_and_gazebo',
            executable='test_local_controller',
            name='test_local_controller',
            output='log'     # 关键语句：确保 cout 和 RCLCPP_INFO 都打印
        )
    ])
