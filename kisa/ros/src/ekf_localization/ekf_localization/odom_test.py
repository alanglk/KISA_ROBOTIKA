#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

from time import sleep

class odom_test(Node):
    def __init__(self):
        super().__init__('odom_test_node')
        self.sub_odom = self.create_subscription(Odometry, 
                                                'odometry/filtered', 
                                                self.odometryCallback, 10)
    def odometryCallback(self, msg):
        self.get_logger().info('Odometry callback')

def main(args=None):
    rclpy.init(args=args)
    odom_node = odom_test()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()            