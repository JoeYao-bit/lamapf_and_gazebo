#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory  # ✅ 必须加


def generate_launch_description():

    # LIDAR
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_a2m8_launch.py'
            )
        ),
        launch_arguments={
            'serial_port': '/dev/rplidar',
            'serial_baudrate': '115200'
        }.items()
    )

    # Kobuki
    kobuki_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('kobuki_node'),
                'launch',
                'kobuki_node-launch.py'
            )
        ),
        launch_arguments={
            'serial_port': '/dev/kobuki',
            'serial_baudrate': '115200',
        }.items()
    )

    # static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','1','0','base_footprint','laser']
    )

    # AMCL
    amcl_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lamapf_and_gazebo'),
                'launch',
                'turtlebot2_amcl_localization.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false'
        }.items()
    )

    # RVIZ
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    # 顺序控制：延迟 2 秒启动 AMCL，延迟 3 秒启动 RVIZ
    delayed_amcl = TimerAction(period=2.0, actions=[amcl_node])
    delayed_rviz = TimerAction(period=3.0, actions=[rviz])

    return LaunchDescription([
        lidar_node,
        kobuki_node,
        static_tf,
        delayed_amcl,
        delayed_rviz,
    ])
