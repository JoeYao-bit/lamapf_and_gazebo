from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lamapf_and_gazebo',
            executable='test_center_controller_in_read_world',
            name='test_center_controller_in_read_world',
            output='screen'     # 关键语句：确保 cout 和 RCLCPP_INFO 都打印
        )
    ])
