#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_matrix

from math import *
import numpy as np
import tf.transformations as tr


class Robot:
    def __init__(self):
        rospy.init_node("square_motion")

        vel_topic = rospy.get_param("~vel_topic", "cmd_vel")
        print("vel_topic:", vel_topic)
        self._vel_pub = rospy.Publisher(vel_topic, Twist, queue_size = 1)
        self._first = 1
        rospy.on_shutdown(self.stop)

        
    def stop(self):
        vel_ = Twist()
        vel_.linear.x = 0
        vel_.linear.y = 0
        vel_.linear.z = 0
        vel_.angular.z = 0
        self._vel_pub.publish( vel_)

    def move(self, v, w, t):
        vel_ = Twist()
        vel_.linear.x = v
        vel_.linear.y = 0
        vel_.linear.z = 0
        vel_.angular.z = w
        self._vel_pub.publish( vel_)
        rospy.sleep(t)

    def square(self):
        print("drawing square with motion")
        for i in range(0, 50):
            self.move(0.35, 0, 18.0)
            self.move(0, 0, 1.0)
            self.move(0, 0.22, 7.15)
            self.move(0, 0, 1.0)
def main():
    robot = Robot()
    rospy.sleep(10)
    robot.square()

if __name__ == "__main__":
    main()
    
       

