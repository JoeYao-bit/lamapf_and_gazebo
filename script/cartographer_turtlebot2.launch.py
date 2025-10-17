from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 配置文件路径
    configuration_directory = os.path.join(
        '/opt/ros/jazzy/share/cartographer_ros/configuration_files'
    )

    configuration_basename = 'backpack_2d.lua'  # 2D 建图配置

    # 1️⃣ 主 SLAM 节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/odom', '/odom')
        ]
    )

    # 2️⃣ 地图栅格发布节点
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0'
        ]
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node
    ])
