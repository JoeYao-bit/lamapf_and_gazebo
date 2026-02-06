from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_ns')
    agent_id = LaunchConfiguration('agent_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ns',
            default_value='robot7'
        ),

        DeclareLaunchArgument(
            'agent_id',
            default_value='7'   # 👈 数字参数
        ),

        Node(
            package='lamapf_and_gazebo',
            executable='test_center_controller_in_read_world',
            name='test_center_controller_in_read_world',
            namespace=robot_ns,
            output='screen',
            arguments=[agent_id]   # ⭐ 关键
        )
    ])
