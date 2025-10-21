import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    # 参数文件路径
    default_param_path = '/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/config/turtlebot2_localization_params.yaml'
    slam_params_file = LaunchConfiguration('slam_params_file')

    # 声明参数
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_param_path,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    # 定位节点
    start_localization = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',  # 必填
        output='screen',
        parameters=[slam_params_file],  # ✅ 必须这样传
        emulate_tty=True
    )

    ld = LaunchDescription()
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_localization)

    return ld
