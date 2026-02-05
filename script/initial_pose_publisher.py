#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler
import yaml
import os
import time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # 发布器
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # 获取 map yaml 路径（可通过参数传入）
        self.declare_parameter('map_yaml_file', '/home/wangweilab/ros2_ws/src/lamapf_and_gazebo/map/my_map_2602052038.yaml')
        map_yaml_file = self.get_parameter('map_yaml_file').value

        if not os.path.exists(map_yaml_file):
            self.get_logger().error(f"Map YAML file not found: {map_yaml_file}")
            return

        # 读取 map origin
        with open(map_yaml_file, 'r') as f:
            data = yaml.safe_load(f)
        x, y, theta = data['origin']
        qx, qy, qz, qw = quaternion_from_euler(0, 0, theta)

        # 创建消息
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = [
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 0.0, 0, 0, 0,
            0, 0, 0, 0.0, 0, 0,
            0, 0, 0, 0, 0.0, 0,
            0, 0, 0, 0, 0, 0.06853891945200942
        ]

        # 延时保证 AMCL 已经 active
        time.sleep(1.0)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published initial pose: x={x}, y={y}, theta={theta}")

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
