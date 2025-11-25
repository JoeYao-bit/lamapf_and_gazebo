from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='large_agent_mapf',
            executable='lamapf_planner_node',
            output='screen',
            name='lamapf_planner_node1'
        )
    ])
