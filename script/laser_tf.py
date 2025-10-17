#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class LaserTF(Node):
    def __init__(self):
        super().__init__('laser_tf')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.broadcast_tf)

    def broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'laser'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = LaserTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
