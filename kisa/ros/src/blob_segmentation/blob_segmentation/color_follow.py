#!/usr/bin/env python

import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
import numpy as np


class ColorFollow(Node):
    def __init__(self):
        super().__init__('color_follow_node')
        self.declare_parameter('vel_topic', 'cmd_vel')
        self.declare_parameter('color_pose_topic', 'blob_segment/color_pose')
        self.declare_parameter('robot_frame_id', 'base_footprint')

        self.get_logger().info("ColorFollow Node created")

        self._vel_topic          = self.get_parameter('vel_topic').value
        self._color_pose_topic   = self.get_parameter('color_pose_topic').value
        self._robot_frame_id   = self.get_parameter('robot_frame_id').value

        # ROS Subscribers
        self._pose_sub = self.create_subscription(PointStamped, self._color_pose_topic, self.received_position_callback, 10)

        # ROS Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, self._vel_topic, 10)
    
    def received_position_callback(self, msg: PointStamped):

        frame_id = msg.header.frame_id
        point = msg.point # point.x, point.y, point.z

        # Transform from camera coord system to robot coord system

        # Compute angle and speed of robot
        
        speed   = 0.0
        turn    = 0.50 * point.x

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