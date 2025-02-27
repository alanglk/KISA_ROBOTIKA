#!/usr/bin/env python
import rclpy
from rclpy.node import Node

import numpy as np
import traceback
import cv2
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Import PointStamped msg to communicate with color_follow node
from geometry_msgs.msg import Point
from depth_interface.srv import GetDepth

class blobDetection(Node):
    def __init__(self):
        super().__init__('blob_segmenter_node')
        self.declare_parameter('color_pose_topic', 'blob_segment/color_pose')
        self.declare_parameter('image_topic', 'color/image_raw')
        self.declare_parameter('use_depth_map', False)
        self.image_topic        = self.get_parameter('image_topic').value
        self.color_pose_topic   = self.get_parameter('color_pose_topic').value
        self.use_depth_map      =  self.get_parameter('use_depth_map').value
        
        self.get_logger().info('Subscribing to %s image topic'%self.image_topic)

        # Create CvBridge
        self.bridge = CvBridge()
        
        # ROS SUBSCRIBERS
        self.sub = self.create_subscription( Image, self.image_topic, self.blob_callback, 10)
        
        # ROS PUBLISHERS
        self.pose_publisher = self.create_publisher(Point, self.color_pose_topic, 10)

        # ROS SERVICE CLIENTS
        self.client = self.create_client(GetDepth, 'depth_server')
        while not self.client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('Waiting for service to be available...')
        self.request = GetDepth.Request()
        self.client_futures = []

        # Define Color Follow Parameter
        self.sel_params = [0, 0, 124, 60, 60, 255] # LowB, LowG, LowR, HighB, HighG, HighR
        self.params = None

        print("Done control window")
        self.window_initialized = False
    
    def send_depth_request(self, a, b, c, d):
        self.request.x0 = a
        self.request.y0 = b
        self.request.x1 = c
        self.request.y1 = d
        self.client_futures.append(self.client.call_async(self.request))

    def blob_callback(self, msg):
        #print("inside callback")
        if not self.window_initialized:
            createControlWindow()
            self.window_initialized = True
            self.setTrackbarParams()

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.handleTrackbarChanges()
            
            cresx, cresy, img, numc, bbox_area, x0, y0, x1, y1 = findCentroid(cv_img.copy(), self.params)
            is_centroid = 1.0 if cresx != -1 else 0.0
            
            # Normalize area of bbox
            height, width = msg.width, msg.height
            norm_area = bbox_area / (width * height)

            
            # Normalize centroid
            px = cresx / width  * 2.0 -1.0
            py = cresy / height * 2.0 -1.0

            # ---- Compute depth ---- 
            # Send depth request
            if is_centroid == 1.0:
                self.send_depth_request(x0, y0, x1, y1)
            
            # Check received responses
            depth   = -1.0
            std_dev = 0.0
            index = 0
            while index < len(self.client_futures):
                f = self.client_futures[index]
                if f.done():
                    r = f.result()
                    depth   = r.depth
                    std_dev = r.stdev
                    self.client_futures.pop(index)
                index += 1            
            
            # x: rotation target
            # y: bounding box area (normalized) or depth
            # z: 1.0 if found centroid else 0.0
            x = px
            y = depth if self.use_depth_map else norm_area
            z = is_centroid
            p = Point(x=float(x), y=float(y), z=float(z))
            self.pose_publisher.publish(p)

            if numc < 1:
                cv2.imshow("Blob segmentation", cv_img)
                cv2.waitKey(1)
            else:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
                drawCross( img, cresx, cresy, (0,0,255), 5)
                cv2.imshow("Blob segmentation", img)
                cv2.waitKey(1)

        except CvBridgeError as exc:
            print(traceback.format.exc())



    def setTrackbarParams(self):
        cv2.setTrackbarPos('LowB', 'Blob segmentation', self.sel_params[0])
        cv2.setTrackbarPos('LowG', 'Blob segmentation', self.sel_params[1])
        cv2.setTrackbarPos('LowR', 'Blob segmentation', self.sel_params[2])
        cv2.setTrackbarPos('HighB', 'Blob segmentation', self.sel_params[3])
        cv2.setTrackbarPos('HighG', 'Blob segmentation', self.sel_params[4])
        cv2.setTrackbarPos('HighR', 'Blob segmentation', self.sel_params[5])

    def getTrackbarParams(self):
        return [cv2.getTrackbarPos('LowB', 'Blob segmentation'),
                cv2.getTrackbarPos('LowG', 'Blob segmentation'),
                cv2.getTrackbarPos('LowR', 'Blob segmentation'),
                cv2.getTrackbarPos('HighB', 'Blob segmentation'),
                cv2.getTrackbarPos('HighG', 'Blob segmentation'),
                cv2.getTrackbarPos('HighR', 'Blob segmentation')]
            
    def handleTrackbarChanges(self):
        self.params = self.getTrackbarParams()
        

def applyErodeAndDilate(im):
    kernel = np.ones((2,2), np.uint8)
    
    eroded = cv2.erode(im,  kernel, iterations = 1)
    dilated = cv2.dilate(eroded, kernel, iterations = 1)
    return dilated


def drawCross(img, x, y, color, d):
    cv2.line(img, (x+d, y-d), (x-d, y+d), color, 2)
    cv2.line(img, (x-d, y-d), (x+d, y+d), color, 2)


def nothing(x):
    pass


def findCentroid(src, params):
    lowRGB = np.array(params[0:3])
    highRGB = np.array(params[3:])

    img_segmented = cv2.inRange(src, lowRGB, highRGB)
    # # Apply erode and dilate
    img_segmented = applyErodeAndDilate(img_segmented);

    # Find contours
    # Depending upon cv2 version!!
    # contours, hierarchy = cv2.findContours(img_segmented, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # im, contours, _ = cv2.findContours(img_segmented, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours, hierarchy = cv2.findContours(img_segmented, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # draw contours on the original image for `CHAIN_APPROX_SIMPLE` 
    # image_copy1 = image.copy()
    # cv2.drawContours(image_copy1, contours1, -1, (0, 255, 0), 2, cv2.LINE_AA)
    resx = -1
    resy = -1
    numcontours = len(contours)
    if numcontours < 1:
      return resx, resy, src, numcontours, 0.0, -1, -1, -1, -1
  
    color = (255,255,255)
    # cv2.drawContours(src, contours, -1, color, 3)
    cv2.drawContours(src, contours, -1, color, 3, cv2.LINE_AA)
    #Approximate contours to polygons + get bounding rects and circles
    contours_poly = []
    center = []
    radius = []
    biggestContour = -1
    maxsize = 0
    boundRect = []
    x = [-1 for i in range(numcontours)]
    y = [-1 for i in range(numcontours)]
    w = [-1 for i in range(numcontours)]
    h = [-1 for i in range(numcontours)]
    for i in range(0, numcontours):
        contours_poly.append(cv2.approxPolyDP(contours[i], 3, True ))
        x[i], y[i], w[i], h[i] = cv2.boundingRect(contours_poly[i])
        c, r = cv2.minEnclosingCircle(contours_poly[i])
        center.append(c)
        radius.append(r)
    i = 0
    maxindex = -1
    maxsize = 0
    color2 = (255, 0, 0)
    for contour in contours:
        if contour.size > 10:
  	        if contour.size > maxsize:
                    maxsize = contour.size
                    maxindex = i
                    biggestContour = contour
                    cv2.rectangle(img_segmented, (x[i], y[i]), (x[i]+w[i], y[i]+h[i]),  color2, 2, 8, 0 )
        i = i+1
    
    
    # Calc max area value
    bbox_area = w[maxindex] * h[maxindex]
    x0, y0, x1, y1 = -1, -1, -1, -1

    #Centroid estimate
    if maxsize >= 10:
        M = cv2.moments( biggestContour, False )
        resx = int(M['m10']/M['m00'])
        resy = int(M['m01']/M['m00'])
        x0, y0, x1, y1 = x[maxindex], y[maxindex], x[maxindex]+w[maxindex], y[maxindex]+h[maxindex]
    
    # print("%d contours found. Biggest size: %d centroid(%d,%d)"%(len(contours), maxsize, resx, resy))
    return resx,resy, img_segmented, numcontours, bbox_area, x0, y0, x1, y1


def createControlWindow():
    print("creating control window")
    cv2.namedWindow('Blob segmentation')
    
    cv2.createTrackbar("LowR", 'Blob segmentation', 0, 255, nothing)
    cv2.createTrackbar("HighR", 'Blob segmentation', 255, 255, nothing)
    
    cv2.createTrackbar("LowG", 'Blob segmentation', 0, 255, nothing)
    cv2.createTrackbar("HighG", 'Blob segmentation', 255, 255, nothing)
    
    cv2.createTrackbar("LowB", 'Blob segmentation', 0, 255, nothing)
    cv2.createTrackbar("HighB", 'Blob segmentation', 255, 255, nothing)

def main(args=None):
    rclpy.init(args=args)
    blob_node = blobDetection()
    print("Node created")
    rclpy.spin(blob_node)
    blob_node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()

