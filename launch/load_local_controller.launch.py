from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    agent_id = LaunchConfiguration('agent_id')

    return LaunchDescription([

        DeclareLaunchArgument(
            'agent_id',
            default_value='0'   # 👈 数字参数
        ),

        Node(
            package='lamapf_and_gazebo',
            executable='test_raise_local_controller',
            name='test_raise_local_controller',
            #namespace=robot_ns,
            output='screen',
            arguments=[LaunchConfiguration('agent_id')]
        )
    ])
