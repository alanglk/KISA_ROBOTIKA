#!/usr/bin/env python
### ROS2 
import rclpy
from rclpy.node import Node
### import service format
from depth_interface.srv import GetDepth

import numpy as np
import traceback
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point

from cv_bridge import CvBridge, CvBridgeError
import pcl

from depth_interface.srv import GetDepth

import ctypes
import struct
import math

class depthServer(Node):
    def __init__(self):
        super().__init__('depth_server_node')
        self.declare_parameter('depth_pcl_topic', '/depth/color/points')
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height',240)

        self.cols = self.get_parameter('image_width').value
        self.rows = self.get_parameter('image_height').value
        self.depth_topic_ = self.get_parameter('depth_pcl_topic').value
        self.get_logger().info('Subscribing to %s depth image topic'%(self.depth_topic_))
        self.get_logger().info('Image size: %d x %d'%(self.cols, self.rows))
        
        self.imsub = self.create_subscription(PointCloud2, self.depth_topic_, self.grab_depth_image, 10)
        
        self.srv = self.create_service(GetDepth,"depth_server", self.depth_callback)
        self.points = None
    def grab_depth_image(self, msg):
        # Extract points from PointCloud2 message
        try:
            # Read points from the PointCloud2 message
            self.points = list(pc2.read_points(
                msg, field_names=('x', 'y', 'z','rgb'), skip_nans=False
            ))
            
            if not self.points:
                self.get_logger().warning("No valid points in the PointCloud2 message.")
                return
            # HOWTO INFO:  get RGB values from pcl_data
            # for data in self.points:
            #     # data contents: 0:x, 1:y, 2:z, 3: rgb
            #     s = struct.pack('>f', data[3])
            #     i = struct.unpack('>l', s)[0]
            #     pack = ctypes.c_uint32(i).value

            #     r = (pack & 0x00FF0000) >> 16
            #     g = (pack & 0x0000FF00) >> 8
            #     b = (pack & 0x000000FF)
            #     self.get_logger().info("RGB values: %d, %d, %d"%( r, g, b))
            #     i = i+1
            #     if i>5: break       
        except Exception as e:
            self.get_logger().error(f"Error processing PointCloud2 message: {e}")

        
        
    def depth_callback(self, request, response):
           
        if request.x0 < 0 or request.x0 >= self.cols or request.y1 < 0 or request.y1 >= self.rows:
            self.get_logger().error('rectangle (%d, %d) (%d, %d) out of image bounds'%(request.x0, request.y0, request.x1, request.y1))
            response.depth = -1.0
            response.stdev = -1.0
            return response
        depth = -1.0
        std_dev = 0.0
        # TODO
        # 1.- extract the depth values from the poincloud (self.points)
        # 2.- filter the depth values within the rectangle and calculate mean and standard deviation
        
        valid_points = []
        rx0, ry0, rx1, ry1 = request.x0, request.y0, request.x1, request.y1
        width = self.cols
        
        for i, p in enumerate(self.points):
            px, py = i % width, i // width
            
            if (rx0 <= px) or (px <= rx1) or (ry0 <= py) or (py <= ry1):
                
                if math.isnan(p[0]) or math.isnan(p[1]) or math.isnan(p[2]):
                    continue
                
                valid_points.append(p)
        
        depths = []
        for p in valid_points:
            # d = np.sqrt( p[0] * p[0] + p[1] * p[1] + p[2] * p[2] )
            d = p[2]
            depths.append(d)
        depths  = np.array(depths)
        
        depth   = np.mean(depths)
        std_dev = np.std(depths)

        

        # end TODO
        self.get_logger().info('Blob mean depth = %.5f (+- %.5f)'%(depth, std_dev))
        response.depth = float(depth)
        response.stdev = float(std_dev)
        return response
    
    


def main(args=None):
    rclpy.init(args=args)
    depthsrv = depthServer()
    try:
        rclpy.spin(depthsrv)
    except KeyboardInterrupt:
        depthsrv.get_logger().info("Node interrupted by keyboard (CTRL+C)")
    finally:
        depthsrv.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()

