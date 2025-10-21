from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml

def load_origin_from_yaml(yaml_file):
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    x, y, theta = data['origin']
    # 转换 theta 到四元数
    from tf_transformations import quaternion_from_euler
    qx, qy, qz, qw = quaternion_from_euler(0, 0, theta)
    return x, y, qz, qw


def generate_launch_description():
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value='/home/yaozhuo/my_map.yaml',
        description='Full path to map file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value='/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/config/amcl_localization.yaml',
        description='Full path to AMCL config file')

    # 地图服务器节点
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file,
                     'use_sim_time': use_sim_time}]
    )

    # AMCL 节点
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    # 生命周期管理器节点（自动激活 map_server 和 amcl）
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'node_names': ['map_server', 'amcl']}]
    )

    # # 在 launch 中
    # x, y, qz, qw = load_origin_from_yaml('/home/yaozhuo/my_map.yaml')
    # initial_pose_node = Node(
    #     package='rclpy',
    #     executable='topic_publisher',  # 你需要写一个简单节点发布 /initialpose
    #     parameters=[{'pose': {'x': x, 'y': y, 'z': 0.0, 'qx': 0, 'qy': 0, 'qz': qz, 'qw': qw}}]
    # )

    return LaunchDescription([
        declare_map_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        #initial_pose_node
    ])
