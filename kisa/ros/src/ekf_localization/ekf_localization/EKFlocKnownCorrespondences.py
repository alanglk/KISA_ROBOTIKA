#!/usr/bin/env python

import rospy
import math as m
import numpy as np
from nav_msgs.msg import Odometry
from time import sleep

from geometry_msgs.msg import Quaternion, Twist
import tf.transformations as tr
import tf_conversions
#from nuslam.msg import TurtleMap
from landmarks.msg import LandmarkMap 
from angles import normalize_angle
from threading import Lock

DEBUG_MODE = False

def getQuaternionFromEuler(roll, pitch, yaw):
    qx = np.sin(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) - np.cos(roll/2)* np.sin(pitch/2)*np.sin(yaw/2)
    qy = np.cos(roll/2)*np.sin(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)* np.cos(pitch/2)*np.sin(yaw/2)
    qz = np.cos(roll/2)*np.cos(pitch/2)*np.sin(yaw/2) - np.sin(roll/2)* np.sin(pitch/2)*np.cos(yaw/2)
    qw = np.cos(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)* np.sin(pitch/2)*np.sin(yaw/2)
    return qx, qy, qz, qw

class mLandmark(object):
    def __init__(self):
        self.cx = 0
        self.cy = 0
        self.id = 0
                                        
class observation(object):
    def __init__(self):
        self.r = 0
        self.angle = 0
#        self.angle2 = 0
        # coordinates in robot frame
        self.cx = 0
        self.cy = 0
        self.id = -1
        # coordinates in map frame
        self.mx = 0
        self.my = 0

def define_ground_truth():    
    map = [mLandmark() for i in range(24)]
    
# 0:   3.96666 4.06132
    map[0].cx = 3.96666
    map[0].cy =  4.06132
    map[0].id = 0

# 1:   2.59329 4.15473
    map[1].cx = 2.59329
    map[1].cy =  4.15473
    map[1].id = 1

# 2:   -0.046083 4.01678
    map[2].cx = -0.046083
    map[2].cy =  4.01678
    map[2].id = 2

# 3:   -2.00799 4.03967
    map[3].cx = -2.00799
    map[3].cy =  4.03967
    map[3].id = 3

# 4:   -4.00844 3.9916
    map[4].cx = -4.00844
    map[4].cy =  3.99162
    map[4].id = 4

# 5:   3.97041 1.89034
    map[5].cx = 3.97041
    map[5].cy =  1.89034
    map[5].id = 5

# 6:   1.46099 1.99578
    map[6].cx = 1.46099 
    map[6].cy = 1.99578
    map[6].id = 6

# 7:   -0.009716 2.4976
    map[7].cx = -0.009716
    map[7].cy =  2.4976
    map[7].id = 7

# 8:   -2.45518 1.9606
    map[8].cx = -2.45518 
    map[8].cy =1.96069
    map[8].id = 8
    
# 9:   -5.16728 2.0161
    map[9].cx = -5.16728
    map[9].cy =  2.01619
    map[9].id = 9

# 10:  3.93814 0.04486
    map[10].cx = 3.93814
    map[10].cy =  0.044862
    map[10].id = 10

# 11:  1.93129 0.317344
    map[11].cx = 1.93129 
    map[11].cy = 0.317344
    map[11].id = 11

# 12:  -2.24674 -0.387112
    map[12].cx = -2.24674
    map[12].cy =  -0.387112
    map[12].id = 12

# 0.643395 -4.34898
    map[13].cx = 0.643395   #-4.7681 
    map[13].cy = -4.34898 #0.418207
    map[13].id = 13

# 14:  3.50052 -1.26595
    map[14].cx = 3.50052 
    map[14].cy = -1.26595
    map[14].id = 14

# 15:  1.57872 -1.97923
    map[15].cx = 1.57872 
    map[15].cy = -1.97923
    map[15].id = 15

# 16:  -0.378342 -1.75731
    map[16].cx = -0.378342
    map[16].cy =  -1.75731
    map[16].id = 16


# 17:  -2.83111 -1.49656
    map[17].cx = -2.83111 
    map[17].cy = -1.49656
    map[17].id = 17

# 18:  -4.76678 -2.00149
    map[18].cx = -4.76678
    map[18].cy =  -2.00149
    map[18].id = 18

# 19:  3.98696 -3.99368
    map[19].cx = 3.98696
    map[19].cy = -3.99368
    map[19].id = 19

# 20:  2.50753 -4.85351
    map[20].cx = 2.50753
    map[20].cy =  -4.85351
    map[20].id = 20

# 21:  -4.7681 0.418207
    map[21].cx = -4.7681 
    map[21].cy = 0.418207
    map[21].id = 21


# 22: -1.99398 -3.50453
    map[22].cx = -1.99398 
    map[22].cy = -3.50453
    map[22].id = 22

# 23:  -3.9406 -3.99733
    map[23].cx = -3.9406
    map[23].cy =  -3.99733
    map[23].id = 23
    
    return map



class EKF(object):
    def __init__(self, mu0, sigma0, alphas, range_sigma, bearing_sigma):
        # Motion covariance
        self.covR = np.matrix([[1e-7], [1e-7], [0.0002]], dtype='float')
        #sensor covariance
        self.sigma_range = range_sigma #0.4 #0.43 # 30
        self.sigma_bearing = bearing_sigma #0.6 #0.6 #1e+16
        self.covQ = np.matrix([[self.sigma_range*self.sigma_range, 0],[0, self.sigma_bearing*self.sigma_bearing]], dtype='float')

        self.a1 = alphas[0] 
        self.a2 = np.deg2rad(alphas[1]) 
        self.a3 = alphas[2] 
        self.a4 = alphas[3] 
        #if DEBUG_MODE:
        print("alpha values: %.3f %.3f %.3f %.3f"%(self.a1, self.a2, self.a3, self.a4))
        print("Sensor noise: %.3f %.3f"%(self.sigma_range, self.sigma_bearing))
        self.m = mu0
        self.sigma = sigma0

        self.t0 = 0 
        
    def printState(self):
        print("Estimated position:", self.m)
        print("Estimated deviation:", self.sigma)

        
    # mu is the mean value of position at time t-1 (before prediction)
    def computeG(self, vx, w, dt):
        if DEBUG_MODE:
            print("Compute G:")

        theta = self.m[2]
        #print("types:", type(theta), type(self.rot1), type(self.trans))
        #print("self.trans:%.2f self.rot1: %.2f theta %.2f"%(self.trans, self.rot1, theta))
        ## According to "EKF_localization_known_correspondences.py":
        # f = vx*dt
        # g1 = np.matrix([[1, 0, -f*m.sin(theta)],
        #               [0, 1, f*m.cos(theta)],
        #               [0,0,1]], dtype='float')

        ## According to http://andrewjkramer.net/intro-to-the-ekf-step-2/

        rot = w * dt
        rot2 = rot * 0.5
        trans = vx * dt
        g = np.matrix([[1, 0, -trans*m.sin(theta+rot2)],
                       [0, 1, trans*m.cos(theta+rot2)],
                       [0,0,1]], dtype='float')
        
        if DEBUG_MODE:
            print(g)
        return g

    def computeV(self, vx, w, dt):
        if DEBUG_MODE:
            print("COmpute V:")

        theta = self.m[2]
        rot = w*dt
        rot2 = rot*0.5
        #trans = v*dt

        ## According to "EKF_localization_known_correspondences.py":
        ## No V is computed --> V=identity
        ## According to http://andrewjkramer.net/intro-to-the-ekf-step-2/
        v = np.matrix([[m.cos(theta+rot2), -0.5*m.sin(theta+rot2)],
                       [m.sin(theta+rot2), 0.5*m.cos(theta+rot2)],
                       [0, 1]], dtype='float')
        if DEBUG_MODE:
            print(v)
        return v

    def computeM(self, vx, w, dt):
        ### Matrix M encodes the noise in the motion model
        if DEBUG_MODE:
            print("COmpute M:")
#        print("m[0]:",[self.a1*self.rot1*self.rot1+ self.a2*self.trans*self.trans, 0, 0])
        ## According to "EKF_localization_known_correspondences.py":
        ## No M is computed --> V=identity
        ## According to http://andrewjkramer.net/intro-to-the-ekf-step-2/
        m = np.matrix([[self.a1*vx*vx + self.a2*w*w, 0],
                       [0, self.a3*vx*vx + self.a4*w*w]],
                        dtype='float')
        #print("m[i]:", m.size, type(m), type(m[0]), type(m[1]), type(m[2]))
        if DEBUG_MODE:
            print(m)
        return m

    def compute_correction(self, cobs, eobs):
        if DEBUG_MODE:
            print("Compute correction:", len(eobs))
        num = len(cobs)
        I = np.identity(3)
        # mu = self.m
        # sigma = self.sigma
        for i in range(num):
            j = i #cobs[i].id
            if DEBUG_MODE:
                print("compute_correction: Computed observation %d corresponds to landmark %d"%(i, cobs[i].id))
            diff = np.matrix([[cobs[i].r - eobs[j].r],
                             [normalize_angle(cobs[i].angle - eobs[j].angle)]], dtype='float')
            if DEBUG_MODE:
                print("Diff %d: %.3f, %.3f"%(i, diff[0], diff[1]))
            # compute H
            q = (eobs[j].cx - self.m[0])*(eobs[j].cx - self.m[0]) + (eobs[j].cy - self.m[1])*(eobs[j].cy - self.m[1])
            #print("q:", q)
            H = np.matrix([[-(eobs[j].cx - self.m[0])/m.sqrt(q), -(eobs[j].cy -self.m[1])/m.sqrt(q), 0],
                 [(eobs[j].cy -self.m[1])/q, (eobs[j].cx -self.m[0])/q, -1]], dtype='float')
            
            #print("Type(H)", H)
            S = H.dot(self.sigma).dot(H.transpose()) + self.covQ
            K = self.sigma.dot(H.transpose()).dot(np.linalg.inv(S))
            self.m = self.m + K.dot(diff)
            self.m[2] = normalize_angle(self.m[2])
            self.sigma = (I - K.dot(H)).dot(self.sigma)
            #print("Correcting...step ", i, ": mu = ", mu, "sigma = ", sigma)
                     

    def compute_prediction(self, vx, w, dt):
        if DEBUG_MODE:
            print("compute prediction:", self.m)
        ## According to "EKF_localization_known_correspondences.py":
        ## new_mu = np.matrix([[vx*dt*cos(theta)],
        ##                    [vx*dt*sin(theta)],
        ##                    [w*dt]], dtype='float')

        ## According to http://andrewjkramer.net/intro-to-the-ekf-step-2/
        if DEBUG_MODE:
            print("compute prediction: dt = %f v = %f w = %f"%(dt, vx, w))
        
        theta = self.m[2]
        rot2 = w*dt*0.5
        new_mu = np.matrix([[vx * dt * m.cos(theta + rot2)],
                            [vx * dt * m.sin(theta + rot2)],
                            [w * dt]], dtype='float')
        # print("Apply prediction. t: %.3f, angle: %.3f, mu %.3f angle+mu: %.3f"%( t, angle, self.m[2], angle+self.m[2]))
        # print("mu:", self.m)
        # print("New mu:", new_mu)
        # print("After prediction: (%.3f, %.3f, %.3f):"%(self.m[0], self.m[1], self.m[2]))

        G = self.computeG(vx, w, dt)
        V = self.computeV(vx, w, dt)
        M = self.computeM(vx, w, dt)
        self.m = self.m + new_mu
        self.m[2] = normalize_angle(self.m[2])
        new_sigma = G.dot(self.sigma).dot(G.transpose()) + V.dot(M).dot(V.transpose())
        if DEBUG_MODE:
            print("before prediction update:")
            print(self.sigma)
        self.sigma = new_sigma
        if DEBUG_MODE:
            print("After prediction update:")
            print(self.sigma)
        
    def computeEKF(self, vx, w, t, current_obs, expected_obs):
        if DEBUG_MODE:
            print("computeEKF")
        res = False
        # print("Computed motion: %.3f, %.3f %.3f"%(self.trans, self.rot1, self.rot2))
        #if abs(vx) > 0.0 or abs(w) > 0.0:
        if True:
            res = True
            dt = t - self.t0
            self.t0 = t
            self.compute_prediction(vx, w, dt)
            if DEBUG_MODE:
                print("ComputeEKF: %.3f"%(dt))
                print("Num observed landmarks: %d"%(len(current_obs)))
            self.compute_correction(current_obs, expected_obs)
            return res
    
class EKFlocalization(object):
    def __init__(self, odom_topic,   landmarks_topic, alphas, sigma_range, sigma_bearings):
        self.odom_initialized = False
        self.landmarks_initialized = False
        self.init_odom = Odometry()
        self.current_timestamp = rospy.Time()
        self.last_timestamp = rospy.Time()
        self.w = 0
        self.vx = 0
        self.dt = 0
        self.sub_odom = rospy.Subscriber(odom_topic, Odometry, self.odometryCallback)
        self.gtruth =  [mLandmark() for i in range(24)]
        self.gtruth = define_ground_truth()
        while not self.odom_initialized: pass
        quat = self.init_odom.pose.pose.orientation
        _, _, yaw = tr.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        initial_mu = np.matrix([[self.init_odom.pose.pose.position.x],
                               [self.init_odom.pose.pose.position.y],
                               [yaw]], dtype='float')
        initial_sigma = np.matrix([[1e-10, 0, 0],[0, 1e-10, 0],[0,0, 1e-10]], dtype='float')
        self.ekf = EKF(initial_mu, initial_sigma, alphas, sigma_range, sigma_bearings)
        self.current_observations = [observation()]
        self.expected_observations = [observation()]
        # Landmarks returned by gazebo in the corresponding topic
        self.landmarks = LandmarkMap() 
        self.sub_landmarks = rospy.Subscriber(landmarks_topic, LandmarkMap, self.landmarksCallback)
        # wait for Gazebo to produce any perception
        while not self.landmarks_initialized: pass

        self.ekf_odom_pub = rospy.Publisher('ekf_odom', Odometry, queue_size=1)
        self.observation = None
        
    def find_correspondence(self, lmx, lmy):
        # Calculate distances among all landmarks
        # gets the minimum corresponding to each one
        # returns the correspondenced landmark id
        num_markers = len(self.gtruth)
        #print("Finding correspondence of (%.3f, %.3f) among %d markers"%(lmx, lmy, num_markers))
        idd = -1
        mind = 1000

        for j in range(0, num_markers):
            gtx = lmx - self.gtruth[j].cx 
            gty = lmy - self.gtruth[j].cy
            dist = m.sqrt(gtx*gtx+gty*gty)
            #print("Map landmark %d at distance %.3f from robot"%(self.gtruth[j].id, r2marker))
#            print("Distance from observ. (%.3f, %.3f) to map landmark %d: %.3f"%(lmx, lmy, self.gtruth[j].id, dist))
            if dist < mind:
                mind = dist
                idd = self.gtruth[j].id
                #print("dist %.3f, updating idd to %d"%(dist, idd))
        return idd
    
                                        
    def compute_expected_observations(self):
        if DEBUG_MODE:
            print("Compute expected observations")
        num = len(self.current_observations)
        eob = [observation() for i in range(num)]
 #       print("Num of observations:", num)
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
                    print("expected obs %d: id %d r=%.3f a=%.3f"%(i, eob[i].id, eob[i].r, eob[i].angle))
        return eob


    def compute_observations(self):
        if DEBUG_MODE:
            print("compute observations")
        num_landmarks = len(self.landmarks.cx)
        if DEBUG_MODE:
            print("Compute observations. Detected: ", num_landmarks)
        self.current_observations = [observation() for i in range(num_landmarks)]
        rx = self.ekf.m[0]
        ry = self.ekf.m[1]
        ra = self.ekf.m[2]
        
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
            ## ZALANTZA:: hauek robotarekiko dira, ez munduarekiko...
            vl.r = dist
            vl.angle = angle
            #######################################################
            self.current_observations[i] = vl #.append(vl)
        num_landmarks = len(self.current_observations)
        for i in range(0, num_landmarks):
            vl = self.current_observations[i]
            vl.id = self.find_correspondence(vl.mx, vl.my)
#            print("%d-th landmark in (%.3f %.3f)(r=%.3f) corresponds to id %d"%(i, vl.cx, vl.cy,vl.r, vl.id))
            self.current_observations[i].id = vl.id #.append(vl)
        # print("--------------")
        # print(self.current_observations)
        # print("--------------")
        self.expected_observations = self.compute_expected_observations()
#        print("Computing observations:", len(self.expected_observations))
        # print("callback expected observations is none?", self.expected_observations == None)
        # print("callback current observations is none?", self.current_observations == None)

    def landmarksCallback(self, msg):
#        print("landmarks callback")

        if not self.landmarks_initialized:
            self.landmarks_initialized = True
        self.landmarks = msg
        
    def odometryCallback(self, msg):
#        print("odom callback")
#        self.mutex.acquire()
        if not self.odom_initialized:
            self.odom_initialized = True
            self.last_odom = msg
            self.current_odom = msg
            self.w = msg.twist.twist.angular.z
            self.vx = msg.twist.twist.linear.x
#            self.vy = msg.twist.twist.linear.y
            self.last_timestamp = msg.header.stamp.to_sec()
            self.current_timestamp = msg.header.stamp.to_sec()
            self.init_odom = msg
        else:
            self.current_timestamp = msg.header.stamp.to_sec()
            self.w = msg.twist.twist.angular.z
            self.vx = msg.twist.twist.linear.x
#            self.vy = msg.twist.twist.linear.y
            self.dt = self.current_timestamp - self.last_timestamp
            # self.current_odom = msg
            #print("Odom callback: %.3f %.3f %.3f"%(self.vx, self.w, self.dt))
#            sleep(0.02)

#        self.mutex.release()



        
    def publishEkfEstimation(self):
        if DEBUG_MODE:
            print("publish ekf estimation")
        omsg = Odometry()
        omsg.header.stamp = rospy.Time.now()
        omsg.header.frame_id = 'map'
        omsg.child_frame_id = 'ekfodom'
        omsg.pose.pose.position.x = self.ekf.m[0]
        omsg.pose.pose.position.y = self.ekf.m[1]
        if DEBUG_MODE:
            print("publish estimation: (%.3f, %.3f, %.3f)"%(self.ekf.m[0], self.ekf.m[1], self.ekf.m[2]))
        m2 = self.ekf.m[2]
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
        rate = rospy.Rate(10)
        # print("Odom initialized:", self.odom_initialized)
        # print("OdomLandmarks initialized:", self.landmarks_initialized)
        count = 0
        while not rospy.is_shutdown():
            self.compute_observations()
            #dt = self.current_timestamp - self.last_timestamp
            res = self.ekf.computeEKF(self.vx, self.w, self.current_timestamp, self.current_observations, self.expected_observations)
            # if res:
            #     self.last_timestamp = self.current_timestamp
            count += 1
            if count > 10:
                self.publishEkfEstimation()
                count = 0
            rate.sleep()
                                        
if __name__ == "__main__":
    rospy.init_node("ekf_localization")
    
    # This param only pretends to show what happens when correction is not applied, i.e how covariance matrix values grow. Correction is applied at every step in KF!

    alphas = [0.1, 0.1, 0.1, 0.1]
    alphas[0] = rospy.get_param("~a1")
    alphas[1] = rospy.get_param("~a2")
    alphas[2] = rospy.get_param("~a3")
    alphas[3] = rospy.get_param("~a4")
    sigma_range = rospy.get_param("~sigma_range")
    sigma_bearing = rospy.get_param("~sigma_angle")

    locEKF = EKFlocalization("/odom", "/landmarks", alphas, sigma_range, sigma_bearing)
    locEKF.run()

    
