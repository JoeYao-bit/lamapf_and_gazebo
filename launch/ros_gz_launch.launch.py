# # 导入描述launch文件的库，这是必须的
# from launch import LaunchDescription
# # 我们需要告诉现在的launch文件，其他要打开的launch文件，在文件系统的哪个位置
# # 然后，去打开
# # 于是就需要导入OS库和FindPackageShare库
# # FindPackageShare库：找到指定名字的包，得到包的路径,例如/home/m/fishbot/src/fishbot_bringup
# # 包下就是launch文件夹和xxx.launch.py文件，即launch/xxx.launch.py
# # OS库：将上面两个路径合成一个/home/m/fishbot/src/fishbot_bringup/launch/xxx.launch.py,
# # 即xxx.launch.py文件的路径
# import os
# from launch_ros.substitutions import FindPackageShare
# # 导入启动另一个launch文件所需要的库
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
 
# def generate_launch_description():
#     """launch内容描述函数，由ros2 launch 扫描调用"""
#     # 构造被包含launch文件的路径，
#     # 比如：/home/m/fishbot/src/fishbot_bringup/launch/xxx.launch.py（见上面的分析）
#     # 找到指定名字的包，得到包的路径,例如/home/m/fishbot/src/fishbot_bringup
#     gazebo_pkg_share = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
#     # 将两个路径合成一个/home/m/fishbot/src/fishbot_bringup/launch/xxx.launch.py,
#     # 即xxx.launch.py文件的路径        
#     gazebo_ros_launch_file_path = os.path.join(gazebo_pkg_share, 'launch', 'gazebo.launch.py')    
    
 
#     # 启动另一个launch的必要格式
#     gazebo_ros_launch_file = IncludeLaunchDescription(
#         # yolov5_ros2_launch_file_path为yolov5_ros2包下launch文件的路径
#         PythonLaunchDescriptionSource(gazebo_ros_launch_file_path),
#         # 可以在这里传递参数，例如：launch_arguments={'arg_name': 'arg_value'}.items(),
#     )
    
#     # 创建LaunchDescription对象launch_description,用于描述launch文件
#     ld = LaunchDescription()
#     ld.add_action(gazebo_ros_launch_file)
 
#     # 返回让ROS2根据launch描述执行节点
#     return ld


"""Demo for using ros2 launch command to start a simulation world file.

Launches Gazebo with a specified world file.

"""

import os

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import ThisLaunchFileDir

from launch_ros.actions import Node

from launch.actions import ExecuteProcess

from ros_gz_bridge.actions import RosGzBridge

def generate_launch_description():

    # empty  （ok）
    # industrial-warehouse （ok）
    # tugbot_warehouse （ok）
    # tugbot_depot (not ok）
    # living_room （not ok）
    # office_env_large （not ok）
    # simple_world (ok)

    world = '/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/world/tugbot_warehouse.sdf'

    print("world path = ", world)

    gazebo = ExecuteProcess(cmd=['gz', 'sim', world], output='screen')

    spawn_entity_srv = ExecuteProcess(cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/world/default/create@ros_gz_interfaces/srv/SpawnEntity'], output='screen')

    set_entity_pose_srv = ExecuteProcess(cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose'], output='screen')

    delete_entity_srv = ExecuteProcess(cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/world/default/remove@ros_gz_interfaces/srv/DeleteEntity'], output='screen')

  #   laser_bridge_bridge1 = ExecuteProcess(cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
  # '/world/default/model/robot_with_lidar/link/base_link/sensor/lidar_sensor/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'], output='screen')
    
    # ROS–Gazebo bridge for lidar
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Lidar bridge
            "/world/default/model/robot_with_lidar/link/base_link/sensor/lidar_sensor/scan"
            "@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        output="screen"
    )
    
    return LaunchDescription([
        gazebo, 
        spawn_entity_srv,
        set_entity_pose_srv,
        delete_entity_srv,
        bridge_node,
    ])