#!/usr/bin/env python

import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import numpy as np


class ColorFollow(Node):
    def __init__(self):
        super().__init__('color_follow_node')
        self.declare_parameter('vel_topic', 'cmd_vel')
        self.declare_parameter('color_pose_topic', 'blob_segment/color_pose')

        self.get_logger().info("ColorFollow Node created")

        self._vel_topic          = self.get_parameter('vel_topic').value
        self._color_pose_topic   = self.get_parameter('color_pose_topic').value

        # ROS Subscribers
        self._pose_sub = self.create_subscription(Point, self._color_pose_topic, self.received_position_callback, 10)

        # ROS Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, self._vel_topic, 10)

        self.last_dir = -1.0

    def received_position_callback(self, msg: Point):
        point = msg # point.x, point.y, point.z
        # x: rotation target
        # y: bounding box area (normalized)
        # z: 1.0 if found centroid else 0.0
        # Only go forward if there is a target centroid in the image

        bbox_area = point.y
        is_target = point.z
        
        target_speed = 0.2 * is_target
        target_rotation = -1.0 * point.x


        if is_target == 1.0:
            self.last_dir = 1.0 if point.x > 1.0 else -1.0
        else:
            target_rotation = -1.0 * self.last_dir

        if bbox_area > 0.4:
            target_speed = 0.0


        speed   = target_speed
        turn    = target_rotation

        # Publish actions
        cmd_vel_msg_ = Twist()
        cmd_vel_msg_.linear.x  = speed
        cmd_vel_msg_.linear.y  = 0.0
        cmd_vel_msg_.angular.z = turn
        self._cmd_vel_pub.publish( cmd_vel_msg_ ) 
    


    def stop(self):
        cmd_vel_msg_ = Twist()      
        cmd_vel_msg_.linear.x  = 0.0
        cmd_vel_msg_.linear.y  = 0.0
        cmd_vel_msg_.angular.z = 0.0
        for i in range(0, 10):
            self._cmd_vel_pub.publish( cmd_vel_msg_ )

def main(args=None):
    rclpy.init(args=args)
    node = ColorFollow()
    print()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by keyboard (CTRL+C)")
        node.stop()
    finally:
        node.stop()
        node.destroy_node()  
        rclpy.shutdown()
    
if __name__=="__main__":
    main()