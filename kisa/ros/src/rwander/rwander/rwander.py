#!/usr/bin/env python

import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class Robot(Node):
    def __init__(self):
        super().__init__('rwander_node')
        self.declare_parameter('vel_topic', 'cmd_vel')
        self.declare_parameter('scan_topic', 'scan')
        vel_topic_ = self.get_parameter('vel_topic').value
        scan_topic_ = self.get_parameter('scan_topic').value

        # ROS Subscribers
        self._laser_sub = self.create_subscription(LaserScan, scan_topic_, self.obstacle_detect, 10)
        # ROS Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, vel_topic_, 10)       
        self._scan_count = 0
        self._max_range = 100.0
        self._uninitialized = 1
        self._bearings = []
        self._scan = []

        self._x = []
        self._y = []
        self._theta = []

        self._rplidar_type = "turtle" # Options: a1, s2, turtle

        
    def obstacle_detect(self, scan_msg):
        
        self._scan = scan_msg.ranges

        if self._uninitialized:
            self._uninitialized = 0
            self._scan_count = len(scan_msg.ranges)
            self._scan_mid = self._scan_count // 2
            self._max_range = scan_msg.range_max
            # Bearings stores the angle of each range measurement (radians)
            for i in range(0, self._scan_count):
                self._bearings.append(scan_msg.angle_min + scan_msg.angle_increment * i)
            self.get_logger().info("# Scan count %d"%(self._scan_count))  ## 3240
            self.get_logger().info("# Laser angle min: %.2f"%(np.rad2deg(scan_msg.angle_min)))
            self.get_logger().info("# Laser angle max: %.2f"%(np.rad2deg(scan_msg.angle_max)))
            self.get_logger().info("# Laser angle increment:  %.4f rad (%.2f deg)"%(scan_msg.angle_increment, np.rad2deg(scan_msg.angle_increment)))
            self.get_logger().info("# Time between mesurements [seconds]:  %.2f"%(scan_msg.time_increment))
            self.get_logger().info("# Time between scans [seconds]:  %.2f"%(scan_msg.scan_time))
            self.get_logger().info("# Minimum range value:  %.2f"%(scan_msg.range_min))
            self.get_logger().info("# Maximum range value:  %.2f "%(scan_msg.range_max))
            resolution = (scan_msg.angle_max - scan_msg.angle_min)/len(scan_msg.ranges)
            self.get_logger().info("# Resolution:  %.2f"%(np.rad2deg(resolution))) 

        # Replace infinity values with max_range 
        self._scan = [x if x < self._max_range else self._max_range for x in self._scan]
        

        # Reorganize scan indexes to make it easier to work with. 
        # 0 index corresponds to the back side of the robot for both, scan and bearings.
        
        if self._rplidar_type == "s2":
            self._scan = [self._scan[i - self._scan_mid] for i in range(self._scan_count)]
        if self._rplidar_type == "a1":
            self._scan = [self._scan[self._scan_mid-i] for i in range(self._scan_count)]
        self.get_logger().info("SCAN --> scan 0: %.2f scan Q1: %.2f scan Q2: %.2f scan Q3: %.2f scan Last: %.2f"% (self._scan[0], self._scan[self._scan_mid//2], self._scan[self._scan_mid], self._scan[self._scan_mid + self._scan_mid//2], self._scan[self._scan_count - 1]))
        self.get_logger().info("bearings 0: %.2f bearings Q1: %.2f bearings Q2: %.2f bearings Q3: %.2f bearings Last: %.2f"% (self._bearings[0], self._bearings[self._scan_mid//2], self._bearings[self._scan_mid], self._bearings[self._scan_mid + self._scan_mid//2], self._bearings[self._scan_count - 1])) 
        
        # TODO: START >>>>>>>>>>>>>>>>>>>>>>>>>>>>
        
        speed = 0.0
        turn = 0.3
        
        # TODO: END <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        
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
    rwander_node = Robot()
    try:
        rclpy.spin(rwander_node)
    except KeyboardInterrupt:
        rwander_node.get_logger().info("Node interrupted by keyboard (CTRL+C)")
        rwander_node.stop()
    finally:
        rwander_node.stop()
        rwander_node.destroy_node()  
        rclpy.shutdown()
    
if __name__=="__main__":
    main()