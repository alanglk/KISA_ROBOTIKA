#!/usr/bin/env python3

import rclpy
import rclpy.logging
from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node
from rclpy.time import Time
import math as m
import numpy as np
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Quaternion, Pose, PoseWithCovariance
from tf_transformations import euler_from_quaternion

from landmark_msgs.msg import LandmarkMap 
from angles import normalize_angle
import csv

from ament_index_python.packages import get_package_share_directory
import os
from rclpy.task import Future
DEBUG_MODE = False

def getQuaternionFromEuler(roll, pitch, yaw):
    qx = np.sin(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) - np.cos(roll/2)* np.sin(pitch/2)*np.sin(yaw/2)
    qy = np.cos(roll/2)*np.sin(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)* np.cos(pitch/2)*np.sin(yaw/2)
    qz = np.cos(roll/2)*np.cos(pitch/2)*np.sin(yaw/2) - np.sin(roll/2)* np.sin(pitch/2)*np.cos(yaw/2)
    qw = np.cos(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)* np.sin(pitch/2)*np.sin(yaw/2)
    return qx, qy, qz, qw

class mLandmark(object):
    def __init__(self, id=None, cx=None, cy=None):
        if id is None:
            self.id = 0
        else:
            self.id = id
        # Coordinates in robot's frame
        if cx is None:
            self.cx = 0
        else:
            self.cx = cx
        if cy is None:
            self.cy = 0
        else:
            self.cy = cy
                                        
class observation(object):
    def __init__(self):
        self.r = 0
        self.angle = 0
        # coordinates in robot frame
        self.cx = 0
        self.cy = 0
        self.id = -1
        # coordinates in map frame
        self.mx = 0
        self.my = 0

def read_ground_truth():
    map = []
    
    landmark_file = '/config/landmarks.csv'

    # Specify the package name
    package_name = 'ekf_localization'
    # Get the path to the package's share directory
    # file_path = os.getcwd() + '/src/ekf_stack' + package_name + landmark_file

    file_path = get_package_share_directory(package_name)+ landmark_file
    rclpy.logging.get_logger('read_ground_truth').info(f'Reading ground truth from {file_path}')
    try:
        with open(file_path, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=",")
            line_i = 0
            rclpy.logging.get_logger('read_ground_truth').info(f'{file_path} file opened')
            for row in csv_reader:
                if len(row) != 4:
                    continue
                
                # skip first line to avoid header
                if line_i > 0: 
                    id = int(row[0])
                    cx = float(row[1])
                    cy = float(row[2])                           
                    lnd = mLandmark(id, cx, cy)
                    map.append(lnd)
                line_i += 1
    except FileNotFoundError:
        rclpy.logging.get_logger('read_ground_truth').info(f'row: {row}')
    except Exception as e:
        rclpy.logging.get_logger('read_ground_truth').info(f'Error reading ground truth: {e}')
    return map



class EKF(object):
    def __init__(self, mu0, sigma0, alphas, range_sigma, bearing_sigma, t0):
        # Motion covariance
        self.covR = np.diag([[1e-7], [1e-7], [0.0002]])
        #sensor covariance
        self.sigma_range = range_sigma #0.4 #0.43 # 30
        self.sigma_bearing = bearing_sigma #0.6 #0.6 #1e+16
        self.covQ = np.array([[self.sigma_range*self.sigma_range, 0],[0, self.sigma_bearing*self.sigma_bearing]], dtype='float')

        self.a1 = alphas[0] 
        self.a2 = np.deg2rad(alphas[1]) 
        self.a3 = alphas[2] 
        self.a4 = alphas[3] 
        #if DEBUG_MODE:
        rclpy.logging.get_logger('EKF init').info("alpha values: %.3f %.3f %.3f %.3f"%(self.a1, self.a2, self.a3, self.a4))
        rclpy.logging.get_logger('EKF init').info("Sensor noise: %.3f %.3f"%(self.sigma_range, self.sigma_bearing))
        self.m = mu0
        self.sigma = sigma0
        
        self.t0 = Time()        
  
    # mu is the mean value of position at time t-1 (before prediction)
    def computeG(self, vx, w, dt):  
        ## According to http://andrewjkramer.net/intro-to-the-ekf-step-2/
  
        theta = self.m[2]
        rot = w * dt
        rot2 = rot * 0.5
        trans = vx * dt
        g = np.array([[1, 0, -trans*m.sin(theta+rot2)],
                       [0, 1, trans*m.cos(theta+rot2)],
                       [0,0,1]], dtype='float')
        
        return g

    def computeV(self, vx, w, dt):
        theta = self.m[2]
        rot = w*dt
        rot2 = rot*0.5
        #trans = v*dt
        ## According to http://andrewjkramer.net/intro-to-the-ekf-step-2/        
        v = np.array([[m.cos(theta+rot2), -0.5*m.sin(theta+rot2)],
                       [m.sin(theta+rot2), 0.5*m.cos(theta+rot2)],
                       [0, 1]])
        return v

    def computeM(self, vx, w, dt):
        ### Matrix M encodes the noise in the motion model

        m = np.array([[self.a1*vx*vx + self.a2*w*w, 0],
                       [0, self.a3*vx*vx + self.a4*w*w]],
                        dtype='float')
        return m

    def compute_correction(self, cobs, eobs):
        if DEBUG_MODE:
            rclpy.logging.get_logger('compute_correction').info("Number of expected observations; %d"%len(eobs))
        num = len(cobs)
        I = np.identity(3)

        for i in range(num):
            j = i #cobs[i].id
            if DEBUG_MODE:
                rclpy.logging.get_logger('compute_correction').info("Computed observation %d corresponds to landmark %d"%(i, cobs[i].id))
            diff_r = cobs[i].r - float(eobs[j].r)
            diff_a = normalize_angle(cobs[i].angle - eobs[j].angle)
            
            diff = np.array([[diff_r], diff_a])

            if DEBUG_MODE:
                rclpy.logging.get_logger('compute_correction').info("Diff %d: %.3f, %.3f"%(i, diff[0], diff[1]))
            # compute H
            q = (eobs[j].cx - self.m[0,0])*(eobs[j].cx - self.m[0,0]) + (eobs[j].cy - self.m[1,0])*(eobs[j].cy - self.m[1,0])
            H = np.array([[(eobs[j].cx - self.m[0,0])/m.sqrt(q), -(eobs[j].cy -self.m[1,0])/m.sqrt(q), 0],
                 [(eobs[j].cy -self.m[1,0])/q, (eobs[j].cx -self.m[0,0])/q, -1]])
            
            S = H.dot(self.sigma).dot(H.transpose()) + self.covQ
            K = self.sigma.dot(H.transpose()).dot(np.linalg.inv(S))
            self.m = self.m + K.dot(diff)
            self.m[2,0] = normalize_angle(self.m[2,0])
            self.sigma = (I - K.dot(H)).dot(self.sigma)
                     

    def compute_prediction(self, vx, w, dt):
        if DEBUG_MODE:
            rclpy.logging.get_logger('compute_prediction').info(self.m)
            rclpy.logging.get_logger('compute_prediction').info("dt = %f v = %f w = %f"%(dt, vx, w))
        
        theta = self.m[2,0]
        rot2 = w*dt*0.5
        new_mu = np.array([[vx * dt * m.cos(theta + rot2)],
                            [vx * dt * m.sin(theta + rot2)],
                            [w * dt]])
        
        G = self.computeG(vx, w, dt)
        V = self.computeV(vx, w, dt)
        M = self.computeM(vx, w, dt)
        self.m = self.m + new_mu
        self.m[2,0] = normalize_angle(self.m[2,0])
        new_sigma = G.dot(self.sigma).dot(G.transpose()) + V.dot(M).dot(V.transpose())
        if DEBUG_MODE:
            rclpy.logging.get_logger('compute_prediction').info("sigma before prediction update:")
            rclpy.logging.get_logger('compute_prediction').info(self.sigma)
        self.sigma = new_sigma
        if DEBUG_MODE:
            rclpy.logging.get_logger('compute_prediction').info("sigma After prediction update:")
            rclpy.logging.get_logger('compute_prediction').info(self.sigma)
        
    def computeEKF(self, vx, w, dt, current_obs, expected_obs):
        rclpy.logging.get_logger('computeEKF').info("Num observed landmarks: %d"%(len(current_obs)))
        self.compute_prediction(vx, w, dt)
        self.compute_correction(current_obs, expected_obs)                          
        return 
    
class EKFlocalization(Node):
    def __init__(self):
        super().__init__('ekf_loc_unknown')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('a1', 0.03),
                ('a2', 2.0),
                ('a3', 0.025),
                ('a4', 0.025),
                ('odom_topic', 'odom'),
                ('landmarks_topic', 'landmarks'),
                ('sigma_range', 0.45),
                ('sigma_angle', 0.65),
            ])
        
        a1 = self.get_parameter("a1").value
        a2 = self.get_parameter("a2").value
        a3 = self.get_parameter("a3").value
        a4 = self.get_parameter("a4").value
        alphas = [a1, a2, a3, a4]
        self.initial_mu = np.array([[0],
                               [0],
                               [0]], dtype=np.float64)
        
        self.get_logger().info("Creating gazebo pose subscriber")
        self.init_gzpose = Pose()
        self.gzpose_initialized = False
        self.gz_future = Future()
        self.sub_gzpose = self.create_subscription(Pose, 
                                                'gz_pose', 
                                                self.gzPoseCallback, 10)
        rclpy.spin_until_future_complete(self, self.gz_future)
        odom_topic = self.get_parameter('odom_topic').value
        self.get_logger().info("Creating odometry subscriber. Topic: %s"%(odom_topic))
        self.odom_future = Future()
        self.odom_initialized = False
        self.init_odom = Odometry()    
        self.current_timestamp = self.get_clock().now()
        self.last_timestamp =  self.get_clock().now()
        self.w = 0
        self.vx = 0
        self.dt = 0

        self.sub_odom = self.create_subscription(Odometry, 
                                                odom_topic, 
                                                self.odometryCallback, 10)
        rclpy.spin_until_future_complete(self, self.odom_future)

        landmarks_topic = self.get_parameter('landmarks_topic').value
        sigma_range = self.get_parameter('sigma_range').value
        sigma_bearings = self.get_parameter('sigma_angle').value
        self.get_logger().info("Parameter values: %.2f %.2f %.2f %.2f %.2f %.2f %s %s "% (a1, a2, a3, a4, sigma_range, sigma_bearings, odom_topic, landmarks_topic))
        
        
        
        
        self.gtruth = read_ground_truth()
        self.get_logger().info('Ground truth initialized')
        self.get_logger().info('%d landmarks in map'%len(self.gtruth))
        if DEBUG_MODE:
            for i in range(24):
                self.get_logger().info('GTRUTH %d at (%.3f, %.3f)'%(self.gtruth[i].id, self.gtruth[i].cx, self.gtruth[i].cy))
        
        
        self.current_observations = [observation()]
        self.expected_observations = [observation()]
        # Landmarks returned by gazebo in the corresponding topic
        self.landmarks_initialized = False  
        self.landmarks_future = Future()
        self.landmarks = LandmarkMap() 
        self.sub_landmarks = self.create_subscription(LandmarkMap, 
                                                       landmarks_topic, 
                                                       self.landmarksCallback, 1)
        rclpy.spin_until_future_complete(self, self.landmarks_future)
        self.get_logger().info('Landmarks initialized')
        
        self.get_logger().info('Odometry initialized')
        
        initial_sigma = np.zeros((3,3))
        self.get_logger().info("Initial mu: %.3f %.3f %.3f"%(self.initial_mu[0], self.initial_mu[1], self.initial_mu[2]))
        np.fill_diagonal(initial_sigma, 1e-10) #array([[1e-10, 0, 0],[0, 1e-10, 0],[0,0, 1e-10]], dtype='float')
        t0 = self.get_clock().now()
        self.ekf = EKF(self.initial_mu, initial_sigma, alphas, sigma_range, sigma_bearings, t0)

        self.ekf_odom_pub = self.create_publisher(Odometry, 'ekf_odom', 1)
        self.observation = None
        self.create_timer(0.1, self.run)

    def find_correspondence(self, lmx, lmy):
        # Calculate distances among all landmarks
        # gets the minimum corresponding to each one
        # returns the correspondenced landmark id
        num_markers = len(self.gtruth)
        idd = -1
        mind = 1000

        for j in range(0, num_markers):
            gtx = lmx - self.gtruth[j].cx 
            gty = lmy - self.gtruth[j].cy
            dist = m.sqrt(gtx*gtx+gty*gty)
            if DEBUG_MODE:
                self.get_logger().info("Map landmark %d at distance %.3f from robot"%(self.gtruth[j].id, dist))
                self.get_logger().info("Distance from observ. (%.3f, %.3f) to map landmark %d: %.3f"%(lmx, lmy, self.gtruth[j].id, dist))
            
            if dist < mind:
                mind = dist
                idd = self.gtruth[j].id
                if DEBUG_MODE:
                    self.get_logger().info("dist %.3f, updating idd to %d"%(dist, idd))
        return idd
    def maximumLikelihoodAssociation(self, cobs_i):
        num_map_landmarks = len(self.gtruth)

        limax = 0
        maxl_index = -1
        limax = -1
        r = cobs_i.r
        a = cobs_i.angle 
        cob = np.array([[r], [a]])
        for k in range(num_map_landmarks):
            # compute distance and angle between each landmark and the robot
            gtx = self.gtruth[k].cx  -  self.ekf.m[0,0] 
            gty = self.gtruth[k].cy  - self.ekf.m[1,0]
            gta = normalize_angle(m.atan2(gty,gtx)  -  self.ekf.m[2,0])
            q = gtx*gtx+gty*gty                
            eob = np.array([[m.sqrt(q)], [gta]])
            
            H  = np.array([[-gtx/m.sqrt(q), -gty/m.sqrt(q), 0],
                                [gty/q, gtx/q, -1]])
            S = H.dot(self.ekf.sigma).dot(H.transpose()) + self.ekf.covQ
            
            # compute likelihood
            error = cob - eob
            if DEBUG_MODE:
                self.get_logger().info("eob =", eob, eob.shape, eob.transpose().shape)
                self.get_logger().info("cob =", cob, cob.shape)
                self.get_logger().info("error:", error, error.shape)
                self.get_logger().info("S =", S, S.shape)
                self.get_logger().info("H =", H, H.shape)
                self.get_logger().info("sigma = ", self.ekf.sigma, self.ekf.sigma.shape)
            
            pow = error.transpose().dot(np.linalg.inv(S)).dot(error)
            li = (1.0/m.sqrt(np.linalg.det(2*m.pi*S)))*m.exp(-0.5*pow)
            if limax < li:
                limax = li
                maxl_index = k
        return maxl_index, li
                                        
    def compute_expected_observations(self):
        if DEBUG_MODE:
            self.get_logger().info("Compute expected observations")
        num = len(self.current_observations)
        eob = [observation() for i in range(num)]
        for i in range(num):
            idd = self.current_observations[i].id
            eob[i].cx = self.gtruth[idd].cx 
            eob[i].cy = self.gtruth[idd].cy 
            #######################################################
            ### Kalkulatu robotaren eta marka teorikoaren arteko r eta angle
            ## Hori litzateke espero dugun erreferentzia
            #######################################################

            q = (self.gtruth[idd].cx - self.ekf.m[0])*(self.gtruth[idd].cx - self.ekf.m[0])+(self.gtruth[idd].cy - self.ekf.m[1])*(self.gtruth[idd].cy - self.ekf.m[1])
            angle = m.atan2(self.gtruth[idd].cy - self.ekf.m[1], self.gtruth[idd].cx - self.ekf.m[0]) - self.ekf.m[2]
            eob[i].r = m.sqrt(q)
            eob[i].angle = normalize_angle(angle)
            eob[i].id = idd
            if DEBUG_MODE:
                for i in range(num):
                    self.get_logger().info("expected obs %d: id %d r=%.3f a=%.3f"%(i, eob[i].id, eob[i].r, eob[i].angle))
        return eob


    def compute_observations(self):
        if DEBUG_MODE:
            self.get_logger().info("Inside compute observations")
        num_landmarks = len(self.landmarks.cx)
        if DEBUG_MODE:
            self.get_logger().info("Detected %d landmarks: "% num_landmarks)
        self.current_observations = [observation() for i in range(num_landmarks)]
        rx = self.ekf.m[0,0]
        ry = self.ekf.m[1,0]
        ra = self.ekf.m[2,0]
        
        for i in range(num_landmarks):
            vl = observation()
            
            # get range and bearing assuming robot frame ref
            vl.cx = self.landmarks.cx[i]
            vl.cy = self.landmarks.cy[i]
            # compute r and angle of the landmark
            dist = m.sqrt(self.landmarks.cx[i]*self.landmarks.cx[i]+self.landmarks.cy[i]*self.landmarks.cy[i])           
            angle = normalize_angle(m.atan2(self.landmarks.cy[i], self.landmarks.cx[i]))
            # convert landmark positions in robot frame to map frame
            ## JUst to make it easier to find the corresponding landmark
            ## in the map, i.e the corresponding idd
            vl.mx = rx + dist*m.cos(angle + ra)
            vl.my = ry + dist*m.sin(angle + ra)
            #######################################################
            ## Hauek robotarekiko dira, ez munduarekiko.
            vl.r = dist
            vl.angle = angle
            #######################################################
            self.current_observations[i] = vl #.append(vl)
        num_landmarks = len(self.current_observations)
    
        for i in range(0, num_landmarks):
            vl = self.current_observations[i]
            ### UNKNOWN CORRESPONDENCES
            vl.id, li = self.maximumLikelihoodAssociation(vl)
            if DEBUG_MODE:
                self.get_logger().info("%d-th landmark in (%.3f %.3f)(r=%.3f a=%.3f) corresponds to id %d with probability %.3f"%(i, vl.cx, vl.cy,vl.r, vl.angle, vl.id, li))
        self.expected_observations = self.compute_expected_observations()

    def landmarksCallback(self, msg):
        if not self.landmarks_future.done():
            self.landmarks_initialized = True
            self.landmarks_future.set_result(True)
        self.landmarks = msg
        
    def gzPoseCallback(self, msg):
        if not self.gz_future.done():
            self.gzpose_initialized = True
            self.init_gzpose = msg
                     
            
            quat = self.init_gzpose.orientation
            _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            self.initial_mu = np.array([[self.init_gzpose.position.x],
                               [self.init_gzpose.position.y],
                               [yaw]])
            self.get_logger().info("INITIAL MU FROM GAZEBO: %.3f %.3f %.3f"%(self.initial_mu[0], self.initial_mu[1], self.initial_mu[2]))  
            self.gz_future.set_result(True)

    def odometryCallback(self, msg):
        if not self.odom_future.done():
            self.odom_initialized = True
            self.last_odom = msg
            self.current_odom = msg
            self.w = msg.twist.twist.angular.z
            self.vx = msg.twist.twist.linear.x
            self.last_timestamp = Time.from_msg(msg.header.stamp)
            self.current_timestamp = Time.from_msg(msg.header.stamp)
            self.init_odom = msg
            self.odom_future.set_result(True)
        else:          
            self.current_timestamp = Time.from_msg(msg.header.stamp)
            self.w = msg.twist.twist.angular.z
            self.vx = msg.twist.twist.linear.x
            tdiff = self.current_timestamp - self.last_timestamp
            self.dt = tdiff.nanoseconds * 1e-9
    
            
    def publishEkfEstimation(self):
        if not self.odom_initialized: pass
        omsg = Odometry()
        omsg.header.stamp = self.get_clock().now().to_msg()
        omsg.header.frame_id = 'map'
        omsg.child_frame_id = 'ekfodom' 
        # self.get_logger().info("publish estimation: (%.3f, %.3f, %.3f)"%(self.ekf.m[0,0], self.ekf.m[1,0], self.ekf.m[2,0]))

        omsg.pose.pose.position.x = self.ekf.m[0,0]
        omsg.pose.pose.position.y = self.ekf.m[1,0]
        if DEBUG_MODE:
            print("publish estimation: (%.3f, %.3f, %.3f)"%(self.ekf.m[0,0], self.ekf.m[1,0], self.ekf.m[2,0]))
        m2 = self.ekf.m[2,0]
        quat = Quaternion()
        quat.x, quat.y, quat.z, quat.w = getQuaternionFromEuler(0, 0, m2)
        omsg.pose.pose.orientation = quat
        cov = 1e5*np.identity(6)
        cov[0][0] = self.ekf.sigma[0,0]
        cov[0][1] = self.ekf.sigma[0,1]
        cov[0][5] = self.ekf.sigma[0,2]
        cov[1][1] = self.ekf.sigma[1,1]
        cov[1][5] = self.ekf.sigma[1,2]
        cov[5][5] = self.ekf.sigma[2,2]
        omsg.pose.covariance = cov.flatten()
        self.ekf_odom_pub.publish(omsg)

    def run(self):
        self.compute_observations()
        self.ekf.computeEKF(self.vx, self.w, self.dt, self.current_observations, self.expected_observations)
        self.last_timestamp = self.current_timestamp
        self.publishEkfEstimation()
    

def main(args=None):
    rclpy.init(args=args)
    ekf_loc = EKFlocalization()
    rclpy.spin(ekf_loc)
    ekf_loc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

    
