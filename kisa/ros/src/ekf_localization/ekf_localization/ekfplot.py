
import matplotlib.pyplot as plt
import rclpy
import rclpy.callback_groups
import rclpy.executors
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

import numpy as np
import numpy.typing as npt
import matplotlib.animation as anim
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Ellipse
import threading

from ament_index_python.packages import get_package_share_directory
import csv
import os

landmark_file = '/config/landmarks.csv'

def read_ground_truth():
    cx = []
    cy = []
    # IGOR: get package name and path to the package's share directory
    file_path = get_package_share_directory("ekf_localization") + landmark_file
    #package_name = '/ekf_localization'
    #file_path = os.getcwd() + '/src/ekf_stack' + package_name + landmark_file

    with open(file_path) as csvFile:
        csv_line = csv.reader(csvFile, delimiter=',')
        line_i = 0
        for row in csv_line:
            if line_i > 0:
                ''' Note: gazebo has a rotation of -pi/2 in its axis definition.
                    Thus, we need to rotate the axis to match the visualization
                    xnew = -y and ynew = x
                '''
                cx.append(-float(row[2]))
                cy.append(float(row[1]))
            line_i += 1
    csvFile.close()

    return cx, cy



class Visualiser(Node):    
    def __init__(self):
        super().__init__('ekfloc_visualizer')
        # self.declare_parameter('odom_topic', 'odom')
        # odom_topic = self.get_parameter('odom_topic').value
        # self.get_logger().info('PLOT ODOM TOPIC: %s'% odom_topic)
        # print("EKF visualizer odometry topic:", odom_topic)
        
        '''
            Attributes:
                fig: figure object for matplotlib
                ax: Axes object for matplotlib
                x, y: ekf odom values to plot
                xx, yy: odom gtruth values to plot
                lock: lock for threading
    
        '''
        self.fig, self.ax = plt.subplots(figsize=(12,15))   #12,15
        # initial values t0 plot
        self.x_data, self.y_data = [] , []
        self.xx_data, self.yy_data = [] , []
        self.landmark_x, self.landmark_y = read_ground_truth()
        # ---
        ''' Create Thread lock to prevent multiaccess threading errors'''
        self.lock = threading.Lock()
        self.e = Ellipse((0,0), 0.1, 0.1, 0.0, color='m', ls ='--', lw=3.5, fc ='none')
        self.cov_ellipses = []
        self.first = 1
        ''' Create subscribers '''
        self.cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        # self.sub = self.create_subscription(
        #     Odometry, odom_topic, self.odom_callback, 10, callback_group=self.cbg
        # )
        self.sub = self.create_subscription(
            Pose, 'gz_pose', self.odom_callback, 10, callback_group=self.cbg
        )

        self.subekf = self.create_subscription(
            Odometry, 'ekf_odom', self.ekfodom_callback, 10, callback_group=self.cbg
        )
        
        

    def plot_init(self):
        self.ax.set_xlim(-9.2, 9.2)
        self.ax.set_ylim(-9.2, 9.2)
        self.ax.set_aspect('equal', 'box')
        self.ax.grid()
        #self.legend = self.ax.legend(["landmarks", "GroundTruth", "EKF"], loc='upper right')
        self.ax.legend(loc='upper right')

    def ekfodom_callback(self, msg):
        with self.lock:
            # self.get_logger().info("plot ekf odometry callback!")
            self.cov = msg.pose.covariance
            w = self.cov[0]
            h = self.cov[7]
            a = self.cov[35]
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            ''' Note: gazebo has a rotation of -pi/2 in its axis definition.    
                Thus, we need to rotate the axis to match the visualization
                xnew = -y and ynew = x
            '''            
            e = Ellipse( (-y, x),  w,  h,  a, color='m',  ls='--', lw=0.5, fc='none')
            self.cov_ellipses.append(e)
            self.xx_data.append(-y)
            self.yy_data.append(x)

    def odom_callback(self, msg):
        ## The ground truth is obtained from gazebo directly
        with self.lock:
            ''' 
                Note: gazebo has a rotation of -pi/2 in its axis definition.
                Thus, we need to rotate the axis to match the visualization
                xnew = -y and ynew = x
            '''
            self.x_data.append(-msg.position.y)
            self.y_data.append(msg.position.x)

    def update_plot(self, _):
        ''' Function for adding data to axis
            Args:
                _: Dummy variable is required for matplotlib animation
            Returns:
                Axes object for matplotlib
        '''
        
        with self.lock:
            # self.ax.clear()
            #self.ax.legend(["landmarks", "GroundTruth", "EKF"])
            l1 = self.ax.scatter(self.landmark_x, self.landmark_y, s=200, c='r', alpha=0.2, marker='*', label='Landmark Locations')
            l2 = self.ax.plot(self.x_data, self.y_data, color='blue', label='odom')#label="odom", color="blue") # odom
            l3 = self.ax.plot(self.xx_data, self.yy_data, color='red', marker='.', markersize=3, label='ekf') #ekf
            last = len(self.cov_ellipses) - 1
            self.ax.add_patch(self.cov_ellipses[last])
            # for e in self.cov_ellipses:
            #     # print(f"Adding ellipse at {e.center} with width={e.width}, height={e.height}, angle={e.angle}")
            #     self.ax.add_patch(e)
                # self.ax.add_artist(e)
            if self.first:
                self.first = 0
                self.ax.legend(loc='upper right')
            # self.ax.relim()
            # self.ax.autoscale_view()
            # self.ax.set_autoscale_on(False)
            
            # self.fig.canvas.draw_idle()
            #self.ax.legend(["GroundTruth", "EKF"], loc='upper right')
    
    def do_the_plot(self):
        ''' Function for initializing and showing matplotlib animation'''
        self.ini = anim.FuncAnimation(self.fig, self.update_plot, init_func=self.plot_init, interval=1000)
        plt.show()


def main(args = None):
    rclpy.init(args=args)    
    vis = Visualiser()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(vis)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    vis.do_the_plot()
    # rclpy.spin(vis)
    rclpy.destroy_node(vis)
    rclpy.shutdown()
                     
if __name__ == "__main__":
    main()