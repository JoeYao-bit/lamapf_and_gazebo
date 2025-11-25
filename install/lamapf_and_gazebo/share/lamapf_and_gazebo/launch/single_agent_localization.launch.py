from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取参数文件路径
    param_file = os.path.join(
        get_package_share_directory('lamapf_and_gazebo'),
        'config',
        'amcl.yaml'
    )

    return LaunchDescription([
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[param_file],
            # remappings=[
            #     ('/scan', '/your_lidar_topic')
            # ]
        )
    ])
