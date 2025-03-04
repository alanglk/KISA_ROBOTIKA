#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import math as m

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
import numpy as np
import math as m
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from time import sleep

color0 = (0, 0, 255)
color1 = (0, 0, 0)
DEBUG=False

class Map:   
    # Map class: size, scale and bitmap
    def __init__(self, rows=500, cols=500, scale=20):
        ''' constructor function  '''
        # Map properties
        self.maxXsize = cols
        self.maxYsize = rows
        self.scale = scale
        self.ox = cols/2
        self.oy = rows/2
        
        self.map = 255*np.ones((rows, cols, 3), np.uint8)

      
    def setPoint(self, xpos, ypos, color):
        i = int(self.ox + xpos * self.scale)
        j = int(self.oy + ypos * self.scale)
        # print("i, j: ", i, j)
        if i < 0 or i > self.maxXsize:
            print("Error: X size exceeded")
            # rclpy.logger("Map::setPoint").logging.info("Error: X size exceeded")
            return -1
        if j < 0 or j > self.maxYsize:
            print("Error: Y size exceeded")
            # rclpy.logger("Map::setPoint").logging.info("Error: Y size exceeded")
            return -1
        self.map[i,j] = color
        return 1

class Robot(Node):
    def __init__(self):
        super().__init__('raw_mapping_node')
        self.declare_parameter('laser_topic', 'scan')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('laser_frame', 'laser')
        self.laser_topic_ = self.get_parameter('laser_topic').value
        self.odom_topic_ = self.get_parameter('odom_topic').value
        self.laser_frame_ = self.get_parameter("laser_frame").value
        self.get_logger().info('Subscribing to %s laser topic'%self.laser_topic_)
        self.laser_sub_ = self.create_subscription(LaserScan,
                                                   self.laser_topic_,
                                                   self.processLaser, 1)
        self.get_logger().info('Subscribing to %s odometry topic'%self.odom_topic_)
        self.odom_sub_ = self.create_subscription(Odometry,
                                                  self.odom_topic_,
                                                  self.processOdometry, 1)
        self.declare_parameter("rplidar_type", "turtle") #options: s2, turtle, a1
        self.rplidar_type = self.get_parameter("rplidar_type").value
        # Robot pose information
        self.rx = 0
        self.ry = 0
        self.ryaw = 0
        self.old_rx = 0
        self.old_ry = 0
        self.old_ryaw = 0
        self.scan = []
        self.scan_count = 0
        self.max_range = 4.0
        self.min_range = 0.0
        self.bearings = []
        self.rmap = Map(rows=1000, cols=1000, scale=20)
        self.odom_uninitialized = True
        self.laser_uninitialized = True
        ## We need to listen to odom to laser_link transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.drawMap)
        self.laserx = 0
        self.lasery = 0
        self.laseryaw = 0

        
    def processOdometry(self, odom_msg):
        
        self.rx = odom_msg.pose.pose.position.x
        self.ry = odom_msg.pose.pose.position.y
        quat = odom_msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.ryaw = yaw
        if DEBUG:
            self.get_logger().info("ODOM x: %.2f y: %.2f yaw: %.2f"%(self.rx, self.ry, np.rad2deg(self.ryaw)))
        self.rmap.setPoint(self.rx, self.ry, color0)
        if self.odom_uninitialized:
            self.odom_uninitialized = False
            self.old_rx = self.rx
            self.old_ry = self.ry
            self.old_ryaw = self.ryaw
            self.laseryaw = self.ryaw

        
    def processLaser(self, scan_msg):
        self.scan = scan_msg.ranges
        if self.laser_uninitialized:
            self.scan_count = len(scan_msg.ranges)
            self.max_range = scan_msg.range_max
            self.min_range = scan_msg.range_min
            self.scan_mid = self.scan_count // 2
            self.Q1 = self.scan_mid//2
            self.Q2 = self.scan_mid
            self.Q3 = self.scan_mid//2 + self.scan_mid

            self.get_logger().info("Laser info: max_range: %.2f scan_count: %d anglemin: %.2f angleinc: %.5f"%(scan_msg.range_max, self.scan_count, scan_msg.angle_min, scan_msg.angle_increment))
            self.bearings = []
            
            for i in range(0, self.scan_count):
                # Calculate bearings
                angle = scan_msg.angle_min + scan_msg.angle_increment*i
                self.bearings.append(angle)

                
            self.laser_uninitialized = False
            # self.get_logger().info(f"BEARINGS: {self.bearings}")
        if self.odom_uninitialized:
            return
        #Filter bad readings
        self.scan = [x if x < self.max_range else self.max_range for x in self.scan]
        
        # Reorganize scan indexes to make it easier to work with. 
        # After reordering, 0 index corresponds to the back side of the robot for both, scan and bearings.        
        if self.rplidar_type == "s2":
            self.scan = [self.scan[i - self.scan_mid] for i in range(self.scan_count)]
       
        if DEBUG:
            self.get_logger().info("SCAN --> scan 0: %.2f scan Q1: %.2f scan Q2: %.2f scan Q3: %.2f scan Last: %.2f"% (
                self.scan[0], self.scan[self.Q1], self.scan[self.Q2], self.scan[self.Q3], self.scan[self.scan_count - 1]))
            self.get_logger().info("bearings 0: %.2f bearings Q1: %.2f bearings Q2: %.2f bearings Q3: %.2f bearings Last: %.2f"% (
                self.bearings[0], self.bearings[self.Q1], self.bearings[self.Q2], self.bearings[self.Q3], self.bearings[self.scan_count - 1])) 
        
    def drawMap(self): 
        if DEBUG:
            self.get_logger().info(f"Laser info:  l_Q1 = {self.scan[self.Q1]} {self.bearings[self.Q1]}")
            self.get_logger().info(f"Laser info:  l_Q2 = {self.scan[self.Q2]} {self.bearings[self.Q2]}")
            self.get_logger().info(f"Laser info:  l_Q3 = {self.scan[self.Q3]} {self.bearings[self.Q3]}")        

        for i in range(self.Q1, self.Q3, 20):
            if self.scan[i] > self.min_range and self.scan[i] < (self.max_range -2) and not m.isinf(self.scan[i]):
                # TODO
                ## Consider the laser is located 2cm backwards from the robot center
                lx = self.rx + np.cos( self.ryaw + self.bearings[i] ) * self.scan[i]
                ly = self.ry + np.sin( self.ryaw + self.bearings[i] ) * self.scan[i]
                # End TODO

                self.rmap.setPoint(lx, ly, color1)
    
        cv2.imshow("Mapa", self.rmap.map)
        cv2.waitKey(1)

    

def main(args=None):
    rclpy.init(args=args)
    mapping_node = Robot()
    rclpy.spin(mapping_node)
    mapping_node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()
    
if __name__=="__main__":
    main()