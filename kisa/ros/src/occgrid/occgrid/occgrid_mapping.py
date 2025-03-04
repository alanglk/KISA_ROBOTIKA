#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math as m
import cv2
from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion
from rclpy.task import Future
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion
import bresenham
import tf2_geometry_msgs  # This is the package to handle PoseStamped transforms in ROS 2''' Some auxiliary functions '''
# Inverse of log of odds-ratio

DEBUG_MODE = False
def l2p(l):
    # m = np.float128(l)
    m = np.float64(l)

    return 1 - (1/(1+np.exp(m)))

# log of odds-ratio
def p2l(p):
    return np.log(p/(1-p))

# get quaternion from euler angle
def getQuaternionFromEuler(roll, pitch, yaw):
    qx = np.sin(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) - np.cos(roll/2)* np.sin(pitch/2)*np.sin(yaw/2)
    qy = np.cos(roll/2)*np.sin(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)* np.cos(pitch/2)*np.sin(yaw/2)
    qz = np.cos(roll/2)*np.cos(pitch/2)*np.sin(yaw/2) - np.sin(roll/2)* np.sin(pitch/2)*np.cos(yaw/2)
    qw = np.cos(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)* np.sin(pitch/2)*np.sin(yaw/2)
    return qx, qy, qz, qw

#-------------------------------------------------------------------------------------------------
#------------------ Definition of a robot provided with a laser scan and odometry ----------------
#-------------------------------------------------------------------------------------------------
class Robot():
    def __init__(self):
        # Robot pose information
        self._rx = 0
        self._ry = 0
        self._ryaw = 0
        self.lodomx = 0
        self.lodomy = 0
        self.lodomyaw = 0
        self._scan = []
        self.prev_pose = [0,0,0]    
        self._pinitialized = False
        # laser information
        self._angle_min = 0
        self._angle_max = 0
        self._angle_increment = 0
        self._scan_count = 0
        self._max_range = 4.0
        self._min_range = 0.0
        self.bearings = []
        self._linitialized = False

        self.scan_mid = 0
        self.Q1 = 0
        self.Q2 = 0
        self.Q3 = 0
                
#-------------------------------------------------------------------------------------------------
#------------------------ Definition of the map class --------------------------------------------
#-------------------------------------------------------------------------------------------------

class Map():
    def __init__(self, xsize, ysize, res, pocc, pfree, pprior, k):
        #-------------------------------------------------------------------------------------------------
        #------------------------ Definition of the map     --------------------------------------------
        #-------------------------------------------------------------------------------------------------
        self.pocc = pocc
        self.pfree = pfree
        self.pprior = pprior
        print("Initializing map of %.2fx%.2f with resolution %.2f"%(xsize, ysize, res))
        self.map_size_x = xsize
        self.map_size_y = ysize
        self.map_resolution = res
        self.map_center_x = -ysize/2 #map_center_x          #meter
        self.map_center_y = -xsize/2 #map_center_y          #meter
        self.l_occ = p2l(pocc)
        self.l_free = p2l(pfree)
        self.l_prior = p2l(pprior)
        self.k = k
        self.map_rows = int(ysize / res)
        self.map_cols = int(xsize/ res)
        print("Setting map image size with %d rows and %d cols"%(self.map_rows, self.map_cols))
        self.gridmap = self.l_prior * np.ones((self.map_rows, self.map_cols))
        # occgrid publisher message
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = 'odom'
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = int(self.map_size_x / self.map_resolution)
        self.map_msg.info.height = int(self.map_size_y / self.map_resolution)
        self.map_msg.info.origin.position.x = self.map_center_x
        self.map_msg.info.origin.position.y = self.map_center_y
        q = Quaternion()
        q.x, q.y,q.z, q.w = getQuaternionFromEuler(0, 0, 0)
        self.map_msg.info.origin.orientation = q

        ### map is visualized as a CV image
        self.imgmap = 128 * np.ones((self.map_rows, self.map_cols, 3), np.uint8)
        self.img_center_j = self.map_rows/2
        self.img_center_i = self.map_cols/2

    def to_ij_img (self, x, y):
        i = (y / self.map_resolution) + self.img_center_i
        j = (x / self.map_resolution) + self.img_center_j
        #print(" x, y --> i,j:", x, y,i, j)
        return j, i #i, j
	
    ''' Given a start point, and a distance measured at a certain angle
	calculate the end point and the corresponding Breseham pixels
	Update the likelihood of the pixels accordingly
	'''
    def raycast_update(self, x0, y0, theta, d):
        x1 = x0 + d*np.cos(theta)
        y1 = y0 + d*np.sin(theta)
        i0, j0 = self.to_ij_img(x0, y0) # Robot pose in pixels
        i1, j1 = self.to_ij_img(x1, y1) # proyection in pixels
        # print("raycast update: %.2f %.2f (%d %d)--> %.2f %.2f (%d %d)"%(x0, y0, i0, j0, x1, y1, i1,j1))
        l = bresenham.bresenham(int(i0), int(j0), int(i1), int(j1))
        pl = list(l)
        jindex = -1
        for ip, jp in pl:
            jindex = jindex + 1
            # TODO
            if jindex < len(pl) - self.k:
                self.gridmap[jp, ip] += self.l_free - self.l_prior
            else:
                self.gridmap[jp,ip] += self.l_occ - self.l_prior
            # END TODO
            color = (1 - l2p(self.gridmap[int(jp), int(ip)]))*255
            self.imgmap[ip, jp] = (color, color, color)
            

    # Each time step, the scan obtained from the laser odometry pose is converted to
    # occupancy values lokelihood         
    def update(self, x, y, theta, scan, bearings):
        for i, z in enumerate(scan):
            if not m.isinf(z):
                self.raycast_update(x, y, (theta + bearings[i]), z)
        return self.gridmap
    
class GridMapping(Node):
    def __init__(self):
        super().__init__('occmapping_node')
        self.declare_parameter('laser_topic', 'scan')
        self.declare_parameter('odom_topic', 'odometry/filtered')
        self.declare_parameter('laser_frame', 'laser')
        self.laser_topic = self.get_parameter('laser_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.laser_frame = self.get_parameter('laser_frame').value
        
        self.get_logger().info('Subscribing to %s laser topic'%(self.laser_topic))
        self.get_logger().info('Subscribing to %s odom topic'%(self.odom_topic))
        self.robot = Robot()
        self.declare_parameter('msize_x', 14.0)
        self.declare_parameter('msize_y', 14.0)
        self.declare_parameter('mresolution', 0.1)
        self.declare_parameter('p_prior', 0.5)
        self.declare_parameter('p_occ', 0.75)
        self.declare_parameter('p_free', 0.45)
        self.declare_parameter('k', 2)
        msizex = self.get_parameter('msize_x').value
        msizey = self.get_parameter('msize_y').value
        res = self.get_parameter('mresolution').value
        pocc = self.get_parameter('p_occ').value
        pfree = self.get_parameter('p_free').value
        pprior = self.get_parameter('p_prior').value
        k = self.get_parameter('k').value
        self.map = Map(msizex, msizey, res, pocc, pfree, pprior, k)
        self.odom_msg = Odometry()
        self.odom_future = Future()
        self.scan_future = Future()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.processOdometry, 1)
        rclpy.spin_until_future_complete(self, self.odom_future)

        self.laser_sub_ = self.create_subscription(LaserScan,
                                                   self.laser_topic,
                                                   self.laserProcessing, 1)
        rclpy.spin_until_future_complete(self, self.scan_future)
        
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)

        
              
        self.create_timer(0.1, self.occmap_callback)
        
    def processOdometry(self, odom_msg):
        self.robot._rx = odom_msg.pose.pose.position.x
        self.robot._ry = odom_msg.pose.pose.position.y
        quat = odom_msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        yaw = np.arctan2(np.sin(yaw), np.cos(yaw))
        self.robot._ryaw = yaw
        # self.robot.odom_msg = odom_msg
        if not self.odom_future.done():
            self.robot.prev_pose = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, yaw)
            self.robot._pinitialized = True
            self.odom_future.set_result(True)
            self.get_logger().info(f'Odom initialized: {self.robot._pinitialized}')

    def laserProcessing(self, scan_msg):
        if not self.scan_future.done():
            self.robot._linitialized = True
            self.robot._scan_count = len(scan_msg.ranges)
            self.robot._angle_min = scan_msg.angle_min
            self.robot._angle_max = scan_msg.angle_max
            self.robot._angle_increment = scan_msg.angle_increment 
            self.robot._scan_count = len(scan_msg.ranges)
            self.robot._max_range = scan_msg.range_max
            self.robot._min_range = scan_msg.range_max
            self.robot.scan_mid = self.robot._scan_count // 2
            self.robot.Q1 = self.robot.scan_mid//2
            self.robot.Q2 = self.robot.scan_mid
            self.robot.Q3 = self.robot.scan_mid//2 + self.robot.scan_mid
            for i in range(0, self.robot._scan_count):
                angle = scan_msg.angle_increment*i
                angle = np.arctan2(np.sin(angle), np.cos(angle))
                self.robot.bearings.append(angle)

            self.get_logger().info(f'Laser initialized: {self.robot._linitialized}')
            self.scan_future.set_result(True)

        # Reorganize scan indexes to make it easier to work with. 
        # After reordering, 0 index corresponds to the back side of the robot for both, scan and bearings.
        self.robot._scan = [scan_msg.ranges[i - self.robot.scan_mid] for i in range(self.robot._scan_count)]

        try:
            '''
            Get transform between laser and robot
            '''
            from_frame = self.laser_frame
            to_frame = 'odom'
            tol = self.tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time())
            quat = tol.transform.rotation
            _, _, lodomyaw =  euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            self.robot.lodomyaw = np.arctan2(np.sin(lodomyaw), np.cos(lodomyaw))
            self.robot.lodomx = tol.transform.translation.x
            self.robot.lodomy = tol.transform.translation.y

            # Log the transformed pose in the laser frame
            if DEBUG_MODE:
                self.get_logger().info(f'Transformed Pose in odom frame: {self.robot._rx} {self.robot._ry} {self.robot._ryaw}') 
                self.get_logger().info(f'Transformed Pose in laser frame: {self.robot.lodomx} {self.robot.lodomy} {self.robot.lodomyaw}')

        except TransformException as ex:
            self.get_logger().info(f'Could not transform {from_frame} to {to_frame}: {ex}')
            pass


    def occmap_callback(self):
                
        '''Calculate robot motion since last update'''
        dist = (self.robot._rx - self.robot.prev_pose[0])**2+(self.robot._ry - self.robot.prev_pose[1])**2
        if np.sign(self.robot._ryaw) == np.sign(self.robot.prev_pose[2]):
            angle = abs(self.robot._ryaw - self.robot.prev_pose[2])
        else:
            angle = abs(self.robot._ryaw + self.robot.prev_pose[2])
        # return
        ''' Update when motion is detected '''
        if dist > 0.1 or angle > 0.1:
            if DEBUG_MODE:
                self.get_logger().info(f'Updating map. dist: {dist} angle: {angle}')
                self.get_logger().info(f'Odom pose. x: {self.robot._rx} y: {self.robot._ry} yaw: {self.robot._ryaw}')
                self.get_logger().info(f'Laser pose. x: {self.robot.lodomx} y: {self.robot.lodomy} yaw: {self.robot.lodomyaw}')

            gridmap = self.map.update(self.robot.lodomx, self.robot.lodomy, self.robot.lodomyaw, self.robot._scan, self.robot.bearings).flatten()
            self.robot.prev_pose = (self.robot._rx, self.robot._ry, self.robot._ryaw)
            ## Publish map
            gridmap_p = l2p(gridmap)
            # gridmap_int8 = (gridmap_p*100).astype(dtype=np.int8)
            
            # gridmap_int8 = np.interp(gridmap_p, (0,1), (-128,127)).astype(np.int8)
            gridmap_int8 = np.interp(gridmap_p, (0,1), (0,255)).astype(np.int8)
            self.map.map_msg.data = gridmap_int8.tolist()
            
        self.map_pub.publish(self.map.map_msg)

        cv2.imshow("Mapa", self.map.imgmap)
        cv2.waitKey(1)

''' MAIN program'''     
 
def main(args=None):
    cv2.namedWindow('Mapa', cv2.WINDOW_KEEPRATIO)
    rclpy.init(args=args) 
    occmap = GridMapping() 
    print("occmap created")
    rclpy.spin(occmap)
    cv2.destroyAllWindows()
    occmap.destroy_node()
    rclpy.shutdown()
    
if __name__=="__main__":
    print("Executing main")
    main()
