#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    TimerAction, IncludeLaunchDescription, GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_ns = 'robot0'

    # RVIZ
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        remappings=[
            ('/initialpose', f'/{robot_ns}/initialpose'),
            ('/goal_pose', f'/{robot_ns}/goal_pose'),
        ],
    )

    delayed_rviz = TimerAction(period=3.0, actions=[rviz])

    # ⭐⭐ 关键部分：把所有节点放入 GroupAction + PushRosNamespace
    ns_group = GroupAction([
        PushRosNamespace(robot_ns),
        delayed_rviz,
    ])

    return LaunchDescription([
        ns_group
    ])
