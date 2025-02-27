#!/usr/bin/env python

import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import numpy as np
from typing import Tuple


class ColorFollow(Node):
    def __init__(self):
        super().__init__('color_follow_node')
        self.declare_parameter('vel_topic', 'cmd_vel')
        self.declare_parameter('color_pose_topic', 'blob_segment/color_pose')
        self.declare_parameter('use_depth_map', False)
        self.get_logger().info("ColorFollow Node created")

        self._vel_topic          = self.get_parameter('vel_topic').value
        self._color_pose_topic   = self.get_parameter('color_pose_topic').value
        self.use_depth_map      =  self.get_parameter('use_depth_map').value

        # ROS Subscribers
        self._pose_sub = self.create_subscription(Point, self._color_pose_topic, self.received_position_callback, 10)

        # ROS Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, self._vel_topic, 10)

        self.last_dir = -1.0
    
    def compute_actions(self, rotation:float, is_target:float, area:float) -> Tuple[float, float]:
        """Returns: [target_speed, target_rotation]
        """
        # Default values
        target_speed    = 0.0
        target_rotation = -2.0 * self.last_dir

        # There isnt a target centroid
        if not is_target:
            target_speed    = 0.0
            target_rotation = -2.0 * self.last_dir
            return target_speed, target_rotation
        self.last_dir = 1.0 if rotation > 1.0 else -1.0

        # If there is a target and the area is very big
        if area > 0.2:
            target_speed = 0.25
            target_rotation = -2.0 * self.last_dir
            return target_speed, target_rotation
        
        # Normal behaviour
        area = area if area > 0.0 else 0.001
        target_speed = (1.0 - area ) / 1.5
        target_rotation = -1.0 * rotation * 2.0
        return target_speed, target_rotation

    def compute_actions_with_depth(self, rotation:float, is_target:float, depth:float) -> Tuple[float, float]:
        """Returns: [target_speed, target_rotation]
        """
        # Default values
        target_speed    = 0.0
        target_rotation = -1.0 * self.last_dir

        # There isnt a target centroid
        if not is_target:
            target_speed    = 0.0
            target_rotation = -1.0 * self.last_dir
            return target_speed, target_rotation
        self.last_dir = 1.0 if rotation > 1.0 else -1.0

        # If there is not a computed depth for the target
        if depth < 0.0:
            target_speed    = 0.2
            target_rotation = -1.0 * rotation * 2.0
            return target_speed, target_rotation

        # If there is a target and there is also a depth calculation
        if depth < 1.0:
            target_speed    = 0.1
            target_rotation = -1.0 * rotation * 2.0
            return target_speed, target_rotation

        # If there is a target and depth
        target_speed    = 0.2 * depth
        target_rotation = -1.0 * rotation * 2.0
        return target_speed, target_rotation
    
    def received_position_callback(self, msg: Point):
        point = msg # point.x, point.y, point.z
        # x: rotation target
        # y: area / mean of compute depth to target point
        # z: 1.0 if found centroid else 0.0
        # Only go forward if there is a target centroid in the image
        is_target = point.z
        rotation = point.x

        target_speed, target_rotation = 0.0, 0.0
        if self.use_depth_map:
            depth = point.y
            target_speed, target_rotation = self.compute_actions_with_depth(rotation, is_target, depth)
        else:
            area = point.y
            target_speed, target_rotation = self.compute_actions(rotation, is_target, area)

        # Publish actions
        speed   = target_speed
        turn    = target_rotation
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