from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_dir = os.path.join(
        os.getenv('HOME'), 'code', 'ros2_ws', 'src', 'lamapf_and_gazebo', 'config'
    )

    carto_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'turtlebot2.lua'
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/odom', '/odom')
        ]
    )

    grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0'
        ]
    )

    return LaunchDescription([
        carto_node,
        grid_node
    ])
