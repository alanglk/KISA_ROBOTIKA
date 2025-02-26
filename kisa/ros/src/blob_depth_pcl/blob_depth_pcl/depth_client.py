#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError


import numpy as np
import traceback
import cv2

from depth_interface.srv import GetDepth

class depthClient(Node):

    def __init__(self):
        super().__init__("depth_client_node")
        self.client = self.create_client(GetDepth, 'depth_server')
        while not self.client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('Waiting for service to be available...')
        
        self.declare_parameter('image_topic', '/color/image_raw')
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height', 240)

        self.cols = self.get_parameter('image_width').value
        self.rows = self.get_parameter('image_height').value
        self.image_topic_ = self.get_parameter('image_topic').value

        self.get_logger().info('Subscribing to %s RGB image topic'%(self.image_topic_))
        self.get_logger().info('Image size: %d x %d'%(self.cols, self.rows))

        # define the container for the service request
        self.request = GetDepth.Request()
        
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.image_topic_, self.image_callback, 10)

        self.orig_img = None
        self.params = [0,0,0,0]
        self.trackbarChanged = False
        self.window_initialized = False

        self.client_futures = []

    def image_callback(self, msg):
        try:
            # self.get_logger().info("image_callback")
            if not self.window_initialized:
                createControlWindow(self.cols, self.rows)
                self.get_logger().info("image_callback: control window created")
                self.window_initialized = True

            # Convert ROS image to OpenCv image
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            self.handleTrackbarChanges()

            cv2.rectangle(cv_img, (self.params[0], self.params[1]),(self.params[2], self.params[3]) , (0,0,255), 2, 8, 0 )
            # self.get_logger().info("Showing color image")
            cv2.imshow("Color panel", cv_img)
            cv2.waitKey(1)
            
            if self.trackbarChanged:
                self.trackbarChanged = False
                x0 = self.params[0]
                y0 = self.params[1]
                x1 = self.params[2]
                y1 = self.params[3]

                # Call service server
                self.get_logger().info("Calling depth service for request")
                response = self.send_depth_request(x0, y0, x1, y1)
                # self.get_logger().info("Depth service result: mean depth: %.2f stdev: %.2f"%(response.depth, response.stdev))

        except CvBridgeError as exc:
            print(traceback,format.exc())
                
    def send_depth_request(self, a, b, c, d):

        self.request.x0 = a
        self.request.y0 = b
        self.request.x1 = c
        self.request.y1 = d
        # self.get_logger().info(">>> CALL")
        # self.future = self.client.call_async(self.request)
        # self.get_logger().info(">>> SPIN")
        # rclpy.spin_until_future_complete(self, self.future)
        # self.get_logger().info(">>> RESPONSE")
        # return self.future.result()
        self.client_futures.append(self.client.call_async(self.request))
            
    def getTrackbarParams(self):
        return [cv2.getTrackbarPos("X0", 'Color panel'),
                cv2.getTrackbarPos("Y0", 'Color panel'),
                cv2.getTrackbarPos("X1", 'Color panel'),
                cv2.getTrackbarPos("Y1", 'Color panel')]
            
    def handleTrackbarChanges(self):
        oldparams = self.params
        self.params = self.getTrackbarParams()
        # self.get_logger().info("handleTrackbarChanges-> oldparams:" + str(oldparams) + "self.params:" + str(self.params))
        if oldparams != self.params and not self.trackbarChanged:
            # self.get_logger().info("Hemen sartu naiz")
            self.trackbarChanged = True
    
    def spin(self):
        while rclpy.ok():
            # self.get_logger().info("HEMEN NAGO")
            rclpy.spin_once(self)
            incomplete_futures = []
            for f in self.client_futures:
                if f.done():
                    response = f.result()
                    self.get_logger().info("Depth service result: mean depth: %.2f stdev: %.2f"%(response.depth, response.stdev))
                    # print("received service result: {}".format(res))
                else:
                    incomplete_futures.append(f)
            self.client_futures = incomplete_futures

def nothing(x):
    pass

def createControlWindow(width, height):
    cv2.namedWindow('Color panel')

    cv2.createTrackbar("X0", 'Color panel',0,  width, nothing)
    cv2.createTrackbar("Y0", 'Color panel', 0,  height, nothing)

    cv2.createTrackbar("X1", 'Color panel', 0, width, nothing)
    cv2.createTrackbar("Y1", 'Color panel', 0, height, nothing)
             
def main():
    rclpy.init()

    depthcli = depthClient()
    depthcli.spin()
    # rclpy.spin(depthcli)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
