from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    # 包的路径
    from ament_index_python.packages import get_package_share_directory
    
    # rplidar package
    rplidar_pkg = get_package_share_directory('rplidar_ros')

    # kobuki_node package
    kobuki_pkg = get_package_share_directory('kobuki_node')

    # your AMCL launch file
    lamapf_pkg = get_package_share_directory('lamapf_and_gazebo')

    # ------------------------------
    # 1) RPLIDAR
    # ------------------------------
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_pkg, 'launch', 'rplidar_a2m8_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/rplidar',
            'serial_baudrate': '115200'
        }.items()
    )

    # ------------------------------
    # 2) Kobuki base driver
    # ------------------------------
    kobuki = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kobuki_pkg, 'launch', 'kobuki_node-launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/kobuki',
            'serial_baudrate': '115200'
        }.items()
    )

    # ------------------------------
    # 3) AMCL（你的自定义 localization）
    # ------------------------------
    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lamapf_pkg, 'launch', 'turtlebot2_amcl_localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false'
        }.items()
    )

    # ------------------------------
    # 4) 静态 TF: base_footprint → laser
    # ------------------------------
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            "0", "0", "0",    # x y z
            "0", "0", "1", "0",   # qx qy qz qw
            "base_footprint", "laser"
        ]
    )

    # ------------------------------
    # 5) RViz2
    # ------------------------------
    rviz = ExecuteProcess(
        cmd=['rviz2'],
        output='screen'
    )

    return LaunchDescription([
        rplidar,
        kobuki,
        static_tf,
        rviz,
                amcl,

    ])
