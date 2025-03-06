#!/usr/bin/env python3
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.task import Future

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, PoseArray
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.srv import GetMap
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Header, Int32 

# import tf_conversions
import tf_transformations as tr
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose, do_transform_point

import numpy as np
from bresenham import bresenham

from mcl.adaptive_particle_filter import AdaptiveParticleFilter
from mcl.resampler import generate_sample_index, compute_required_number_of_particles_kld

DEBUG_MODE = False
#------------------------------------------------------------------------------------
#------------------------------ Map client class ------------------------------------
#------------------------------------------------------------------------------------

class MapClient(object):

    def __init__(self, msg):
        # This represents a 2-D grid map
        #std_msgs/Header header

        # MetaData for the map
        # The origin of the map [m, m, rad].  This is the real-world pose of the
        # bottom left corner of cell (0,0) in the map.
        # geometry_msgs/Pose origin
        # MapMetaData info

        # The map data, in row-major order, starting with (0,0). 
        # Cell (1, 0) will be listed second, representing the next cell in the x direction. 
        # Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
        # The values inside are application dependent, but frequently, 
        # 0 represents unoccupied, 1 represents definitely occupied, and
        # -1 represents unknown. 
        # int8[] data

        info = msg.info
        data = msg.data
        print("Map info: width %d height %d resolution: %.2f"%(info.width, info.height, info.resolution))
        print("Map origin:", info.origin)
        print("Map size:", len(data))
        self.resolution = info.resolution
        self.origin = info.origin
        self.width = info.width
        self.height = info.height
        self.ymin =  info.origin.position.y 
        self.ymax =  info.origin.position.y + info.height * info.resolution
        self.xmin =  info.origin.position.x 
        self.xmax =  info.origin.position.x + info.width * info.resolution
        print("Map bounds:", self.xmin, self.xmax, self.ymin, self.ymax)
        
        self.ogm = np.array(data).reshape((info.height, info.width)) # map
        # self.print_map()
        
        

    def print_map(self):
        for i in range(self.width):
            for j in range(self.height):
                print(f"{self.ogm[j][i]:3}", end = " ")
            print("\n")
            
    def is_valid_state(self, x, y):
        # (x,y) is the particle position in world coordinates (m)
        # we need to convert them to occupancy grid cell coordinates
        inside = self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax
        gx = (x - self.origin.position.x) / self.resolution
        gy = (y - self.origin.position.y) / self.resolution
        outbounds = gy < 0 or gy >= self.height or gx < 0 or gx >= self.width
        #is_free = True
        ogm = -1

        if not outbounds:
            row = int(gx) 
            col = int(gy) 
            ogm = self.ogm[col][row]
        return (inside, outbounds, ogm==0, ogm)
    
    def simulate_scan_with_bresenham(self, xr, yr, yawr, angles, range_min, range_max):
        # (xr, yr, yawr) is the pose of the particle in world coordinates
        # Bresenham computes the list of cells that connect two cells in a grid map
        # Thus, we need to transform the world poses to grid coordinates and vv

        ranges = []
        range_step = self.resolution
        
        phi0 = yawr 
        phi0 = np.arctan2(np.sin(phi0), np.cos(phi0))

        # Origin of the scan (we assume the particle is in a valid state)
        ox = (xr - self.origin.position.x) / self.resolution
        oy = (yr - self.origin.position.y) / self.resolution
        ox = int(ox)
        oy = int(oy)
        for angle in angles:
            phi = phi0 + angle
            phi = np.arctan2(np.sin(phi), np.cos(phi))
            # Compute the end of the ray
            xm = xr + range_max*np.cos(phi)
            ym = yr + range_max*np.sin(phi)
            ex = (xm - self.origin.position.x) / self.resolution
            ey = (ym - self.origin.position.y) / self.resolution

            ex = int(ex)
            ey = int(ey)    
            
      
            # compute the presenham distance: all the cells in the straight line
            # print(f"particle pose: {xr}, {yr} ray pose: {xm}, {ym}")    
            # print(f"origin: ({ox}, {oy}) end: ({ex}, {ey})")
            cells = bresenham(ox, oy, ex, ey) #oy, ox, ey, ex)
            r = range_max
            for gx, gy in cells: 
                # outbounds           
                if gx < 0 or gx >= self.width or gy < 0 or gy >= self.height:
                    # print((f"outbounds: ({gx}, {gy})"))
                    # ranges.append(r)
                    break
                # unknown
                if self.ogm[gy][gx] == -1:
                    # print(f"unknown: ({gx}, {gy})")
                    # ranges.append(r)
                    break 
                # occupied             
                if self.ogm[gy][gx] == 100:
                    # print(f"occupied: ({gx}, {gy})")
                    x = gx * self.resolution + self.origin.position.x 
                    y = gy * self.resolution + self.origin.position.y
                    r = np.sqrt((x-xr)**2 + (y-yr)**2)
                    # ranges.append(r)
                    break
                # If Free, continue with the next cell in the line
            ranges.append(max(range_min, min(r, range_max)))
            
            # print("----------------------------------")
            # print(f"cells  list: {list(cells)}")
            # print(f"range list: {ranges}")
            # print("----------------------------------")

        # print("predict_laser_scan_for particle: %d", len(ranges))
        return ranges


    def simulate_scan_from_pose(self, xr, yr, yawr, angles, range_min, range_max):

        # for every relative angle in angles
        # 1. The absolute angle based on the robot's orientation is computed
        # 2. Ray tracing from (x,y) along the abosulte angle using step size range_step is done
        #    (a) If the currently examined point is within the bounds of the workspace
        #        stop if it meets an obstacle or if it reaches max_range
        #    (b) If the currently examined point is outside the bounds of the workspace
        #        stop if it reaches max_range
        # 3. The computed collection of ranges corresponding to the given angles is returned

        
        ranges = []
        range_step = self.resolution
        phi0 = yawr 
        phi0 = np.arctan2(np.sin(phi0), np.cos(phi0))

        for angle in angles:
            phi = phi0 + angle
            phi = np.arctan2(np.sin(phi), np.cos(phi))
            r = range_min
            while r <= range_max:
                xm = xr + r*np.cos(phi)
                ym = yr + r*np.sin(phi)
                inside, outbounds, is_free, icell = self.is_valid_state(xm, ym)
                if not inside or outbounds or icell == -1:
                    # No valid distance
                    r = range_max
                    break              
                if not is_free:
                    #r should be the correct reading
                    break
                # otherwise, continue trying
                r += range_step

            ranges.append(r )
        return ranges

#------------------------------------------------------------------------------------
#------------------------------Laser client class -----------------------------------
#------------------------------------------------------------------------------------

class LaserClient(object):
    def __init__(self, sample_size):
        self.sample_size = sample_size
        self.N = 0
        self.range_min = 0
        self.range_max = 0
        self.angle_min = 0
        self.angle_max = 0
        self.angle_increment = 0
        self.ranges = []
        self.bearings = []
        self.subsampled_angles = []
        self.subsampled_scan = []       
    
    def subsample_laser_scan(self):
        """Subsamples a set number of beams (self.eval_beams) from the incoming actual laser scan. It also
        converts the Inf range measurements into max_range range measurements, in order to be able to
        compute a difference."""
        N = len(self.ranges)
        step = N//self.sample_size
        # ranges = self.ranges[::step]
        actual_bearings = self.bearings
        # print(f"subsampling laser scan {ranges} ({len(ranges)}) {actual_bearings} ({len(actual_bearings)})")

        actual_ranges = []
        
        # assert (len(self.ranges) == len(sactual_bearings))
        for r in self.ranges:
            if r >= self.range_min and r <= self.range_max:
                actual_ranges.append(r)
            if r < self.range_min:
                actual_ranges.append(self.range_min)
            if r > self.range_max:
                actual_ranges.append(self.range_max)
        
            
        subsampled_ranges = actual_ranges[::step]
        subsampled_bearings = actual_bearings[::step]
        
        return subsampled_ranges, subsampled_bearings
    
    
#------------------------------------------------------------------------------------
#------------------------- Odometry client class ------------------------------------
#------------------------------------------------------------------------------------
    
    
class OdometryClient(object):
    def __init__(self):        
        self.prev_odom = Odometry()
        self.odom = Odometry()
        self.dx = 0
        self.dy = 0
        self.dyaw = 0
        self.prev_yaw = 0
        self.rx0 = self.ry0 = self.rtheta0 = 0
        self.rx = self.ry = self.ra = 0        
        self.has_moved = False
       

#------------------------------------------------------------------------------------
#----------------------   Adaptive Montecarlo Localization -----------------------------------
#------------------------------------------------------------------------------------

class AdaptiveMonteCarloLocalization(Node):
    def __init__(self):
        super().__init__('adaptive_monte_carlo_localization')
        '''
        Parameters:
            For a high-precision laser sensor, sigma_hit might be 0.1 to 0.2 meters. 
                A larger sigma_hit makes it more tolerant of noise or small deviations.
            lambda_short is the rate parameter of the exponential distribution used in the z_short component. 
                It models the probability of getting a short reading due to unexpected obstacles or reflections.
                Values typically range from 0.5 to 2.0
                A smaller value indicates that short readings are more likely.
            A high z_hit implies the sensor is reliable and accurately detects real obstacles.
            A higher z_max indicates that the sensor often reaches its maximum range without detecting anything (e.g., in open spaces).
            A high z_rand suggests the sensor data has more noise or spurious points. 
            A higher z_short suggests the environment may have reflective surfaces or obstacles not accounted for in the map.
        '''
        self.declare_parameters(
            namespace = '',
            parameters = [
                # ('odom_topic', '/odometry/filtered'),
                ('odom_topic', '/rosbot_base_controller/odom'),
                ('translation_noise_std_dev', 0.2),
                ('orientation_noise_std_dev', 0.1),
                ('laser_eval_beams', 15),
                ('initial_num_particles', 100),
                ('min_num_particles', 10),
                ('max_num_particles', 2000),
                ('epsilon', 0.05),
                ('upper_quantile', 2.3263),
                # ('globalloc', False),
                ('z_hit', 0.99),
                ('z_max', 0.005),
                ('z_rand', 0.000),
                ('z_short', 0.000),
                ('range_measurement_noise_hit', 0.15), #sigma_hit
                ('range_measurement_noise_short', 0.85) #lambda_short
    
            ]

        )
        # Particle motion model params
        self.translation_noise = self.get_parameter("translation_noise_std_dev").value
        self.orientation_noise = self.get_parameter("orientation_noise_std_dev").value
        
        # Range sensor model params
        self.laser_sample_size = self.get_parameter("laser_eval_beams").value
        self.z_hit = self.get_parameter("z_hit").value
        self.z_short = self.get_parameter("z_short").value
        self.z_rand = self.get_parameter("z_rand").value
        self.z_max = self.get_parameter("z_max").value
        self.sigma_hit = self.get_parameter("range_measurement_noise_hit").value
        self.lambda_short = self.get_parameter("range_measurement_noise_short").value
        ## KLD specific params
        self.resolutions_grid = [0.5, 0.5, 0.25]
        self.epsilon = self.get_parameter("epsilon").value
        self.upper_quantile = self.get_parameter("upper_quantile").value

        ## Adaptive PF params
        self.num_particles = self.get_parameter("initial_num_particles").value 
        self.min_num_particles = self.get_parameter("min_num_particles").value 
        self.max_num_particles = self.get_parameter("max_num_particles").value 
        
        # globalloc = self.get_parameter("globalloc").value
        odom_topic = self.get_parameter("odom_topic").value
        
        ## Print summary of parameters
        self.get_logger().info('Initial number of particles: {}'.format(self.num_particles))
        self.get_logger().info('Min number of particles: {}'.format(self.min_num_particles))
        self.get_logger().info('Max number of particles: {}'.format(self.max_num_particles))
        self.get_logger().info('Translation noise: {}'.format(self.translation_noise))
        self.get_logger().info('Orientation noise: {}'.format(self.orientation_noise))
        self.get_logger().info('Laser sample size: {}'.format(self.laser_sample_size))
        self.get_logger().info('Odometry topic: {}'.format(odom_topic))
        # self.get_logger().info('Resampling algorithm: {}'.format(self.resampling_algorithm))
        self.get_logger().info('Range measurement z_hit: {}'.format(self.z_hit))
        self.get_logger().info('Range measurement zshort: {}'.format(self.z_short))
        self.get_logger().info('Range measurement z_max: {}'.format(self.z_max))
        self.get_logger().info('Range measurement z_rand: {}'.format(self.z_rand))
        self.get_logger().info('Epsilon: {}'.format(self.epsilon))


        self.particles_pub = self.create_publisher(PoseArray, 'particles', 10)
        self.pub_estimated_pose = self.create_publisher(PoseArray, 'estimated_pose', 10)
        self.pub_laser_subsamples = self.create_publisher( Marker, 'laser_points', 10)
        self.pub_current_num_particles = self.create_publisher( Int32, 'current_num_particles', 10)
        ## TO BE REMOVED
        self.pub_expected_scan = self.create_publisher( Marker, 'expected_scan', 10)
        ### END TO BE REMOVED

        '''
            Initialize occupancy grid map contents and its info
        '''
        self.map_future = Future()
        self.map_client = self.create_client(GetMap, '/map_server/map')
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('######Waiting for /map_server/map service...')
        
        # Call the service once the map is available
        self.get_logger().info('######/map_server/map service is available, sending request...')
        req = GetMap.Request()

        # Send the request and wait for the response asynchronously
        future = self.map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        # Check if the result is valid
        if future.result() is not None:
            self.map = MapClient(future.result().map)   
        else:
            self.get_logger().error('Failed to get map!')

        self.get_logger().info('Map info after future: width %d height %d resolution: %.2f'%(self.map.width, self.map.height, self.map.resolution))
        
        
        ''' Initialize laser properties '''
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.odom_future = Future()
        self.odom = OdometryClient() 
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odometryCallback, 1)
        rclpy.spin_until_future_complete(self, self.odom_future)
        rclpy.logging.get_logger('OdometryClient').info("Robot initial pose:%.3f %.3f %.3f"%(self.odom.rx0, self.odom.ry0, self.odom.rtheta0))
        self.laser_future = Future()
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laserCallback, 1)
        rclpy.spin_until_future_complete(self, self.laser_future)
        rclpy.logging.get_logger('LaserClient').info("Laser initialized")

        map_limits = [self.map.xmin, self.map.xmax, self.map.ymin, self.map.ymax]
        process_noise = [self.translation_noise, self.orientation_noise]
        self.get_logger().info(f'First odom pose: {self.odom.rx0} {self.odom.ry0} {self.odom.rtheta0}')

        self.estimated_pose_mean = [self.odom.rx0, self.odom.ry0, self.odom.rtheta0]
        # # Initialize particles
        self.pf = AdaptiveParticleFilter(self.num_particles, 
                                         map_limits, 
                                         process_noise, 
                                         self.z_hit, 
                                         self.resolutions_grid, 
                                         self.epsilon, 
                                         self.upper_quantile, 
                                         self.min_num_particles, self.max_num_particles)
        self.global_initialization()
        self.create_timer(0.1, self.run)
    
    def laserCallback(self, msg):
        ''' subsample laser scan and save it '''
        
        if not self.laser_future.done():
            self.laser = LaserClient(self.laser_sample_size)
            self.laser.N = len(msg.ranges)
            self.laser.range_min = msg.range_min
            self.laser.range_max = msg.range_max
            self.laser.angle_min = msg.angle_min
            self.laser.angle_max = msg.angle_max
            self.laser.angle_increment = msg.angle_increment
            self.laser.ranges = msg.ranges
            self.laser.bearings = [(self.laser.angle_increment * i) for i in range(self.laser.N)] 
            self.laser.bearings = [(np.arctan2(np.sin(self.laser.bearings[i]), np.cos(self.laser.bearings[i]))) for i in range(self.laser.N)]   
            self.get_logger().info("Laser initialized: N=%d angle_min=%.3f angle_max=%.3f angle_increment=%.3f"%(self.laser.N, self.laser.angle_min, self.laser.angle_max, self.laser.angle_increment))
            self.get_logger().info("Laser initialized: bearings length: %d"%(len(self.laser.bearings)))
            self.get_logger().info("Laser 0: %.2f bearings[0]: %.3f"%(self.laser.ranges[0], self.laser.bearings[0]))
            self.get_logger().info("Laser 400: %.2f bearings[400]: %.3f"%(self.laser.ranges[400], self.laser.bearings[400]))
            self.get_logger().info("Laser 800: %.2f bearings[800]: %.3f"%(self.laser.ranges[800], self.laser.bearings[800]))
            self.get_logger().info("Laser 1200: %.2f bearings[1200]: %.3f"%(self.laser.ranges[1200], self.laser.bearings[1200]))
            self.laser_future.set_result(True) 
        # self.get_logger().info("lasercallback")
        self.laser.ranges = msg.ranges
        self.laser.subsampled_scan, self.laser.subsampled_angles = self.laser.subsample_laser_scan()
        self.publish_laser_pts()
    
    def compute_pose_from_msg(self, msg):
        px = msg.pose.pose.position.x 
        py = msg.pose.pose.position.y 
        quat = msg.pose.pose.orientation 
        _, _, pyaw = tr.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return px, py, pyaw
    
    def global_initialization(self):
        num_p = 0
        self.pf.particles = []
        weight = 1.0 / self.num_particles
        while num_p < self.num_particles:
            par = self.pf.initialize_particle_uniform()
            _, _, is_free,_ = self.map.is_valid_state(par[1][0], par[1][1])
            if is_free:
                num_p+= 1
                self.pf.particles.append(par)

        

    def odometryCallback(self, msg):
        if not self.odom_future.done():
            ''' I should initialize somehow the localization system...'''
            ''' Publish /initialpose ???'''

            self.odom.prev_odom = msg
            self.odom.rx0, self.odom.ry0, self.odom.rtheta0 = self.compute_pose_from_msg(msg)
            self.odom_future.set_result(True)
        # Get new pose
        self.odom.odom = msg
        self.odom.rx, self.odom.ry, self.odom.ra = self.compute_pose_from_msg(msg)

     
    def transform_laser_points(self):
        pts_in_map = []
        try:
            transform = self.tf_buffer.lookup_transform('odom', 'laser', rclpy.time.Time())
            
            _, _, lyaw = tr.euler_from_quaternion([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
            # self.get_logger().info(f'Laser to map transform: {transform.transform} {lyaw}')
            # self.get_logger().info('Robot pose: %.4f %.4f %.4f' % (self.odom.rx, self.odom.ry, self.odom.ra))
            pts_in_map = [ Point(x=transform.transform.translation.x + r*np.cos(theta + lyaw),
                        y=transform.transform.translation.y + r*np.sin(theta + lyaw),
                        z=0.2) for r,theta in zip(self.laser.subsampled_scan, self.laser.subsampled_angles)]
            return pts_in_map
        except Exception as e:
            self.get_logger().error('Laser to odom transform failed with exception: %s' % str(e)) 
            return pts_in_map




    def project_laser_points(self):
        # self.get_logger().info(f"subsampled ranges and angles: {self.laser.subsampled_scan} {self.laser.subsampled_angles}")
        pts_in_map = [ Point(x=r*np.cos(theta),
                        y=r*np.sin(theta),
                        z=0.2) for r,theta in zip(self.laser.subsampled_scan, self.laser.subsampled_angles)]
        return pts_in_map
    
    def transform_laser_points_to_map(self):
        laser_points = self.project_laser_points()
        # self.get_logger().info(f'transform laser points to map {laser_points}') 

        pts_in_map = []
        try:
            transform = self.tf_buffer.lookup_transform('map', 'laser', rclpy.time.Time())            
            _, _, yaw = tr.euler_from_quaternion([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
            translation = transform.transform.translation
            for p in laser_points:
                x_map = p.x * np.cos(yaw) - p.y*np.sin(yaw) + translation.x
                y_map = p.x * np.sin(yaw) + p.y * np.cos(yaw) + translation.y
                pts_in_map.append((x_map, y_map, 0.2))

            # for p in laser_points:
            #     pi = PointStamped(point=p, header=Header(frame_id="laser", stamp=transform.header.stamp))
            #     # self.get_logger().info(f'pi: {pi} {type(pi)}')
            #     pi2map = do_transform_point(pi, transform)

            #     #pts_in_map.append(pi2map.point)
            
            return pts_in_map
        except Exception as e:
            self.get_logger().error('Laser to odom transform failed with exception: %s' % str(e)) 
            return pts_in_map

    def transform_expected_scan_to_map(self, scan):
        laser_points = [ Point(x=r*np.cos(theta),
                        y=r*np.sin(theta),
                        z=0.2) for r,theta in zip(scan, self.laser.subsampled_angles)]
        
        pts_in_map = []
        try:
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())            
            _, _, yaw = tr.euler_from_quaternion([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
            for p in laser_points:
                pi = PointStamped(point=p, header=Header(frame_id="odom", stamp=transform.header.stamp))
                pi2map = do_transform_point(pi.point, transform)

                pts_in_map.append(pi2map)
            
            return pts_in_map
        except Exception as e:
            self.get_logger().error('Laser to odom transform failed with exception: %s' % str(e)) 
            return pts_in_map

    def publish_expected_scan_from_pose(self, px, py, pyaw, scan, color):
        """Publishes the currently received laser scan points from the robot, after we subsampled
        them in order to comparse them with the expected laser scan from each particle."""
        print(f"publishing expected scans in color {color}")
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.ns = 'laser_points2'
        msg.id = 35000
        msg.type = 6
        msg.action = 0
        msg.type = Marker.POINTS
            
        ''' Projections of the laser readings.
            These are with respect to the odom frame
            rviz should be able to transform them to the map frame'''
        msg.points = [ Point(x=px + r*np.cos(theta+pyaw),
                        y=py + r*np.sin(theta +pyaw),
                        z=0.2) for r,theta in zip(scan, self.laser.subsampled_angles)]
        # msg.points = msg.points[0:2]
        ''' commented out'''
        # color = ColorRGBA(r=pcolor[0], g=pcolor[1], b=pcolor[2])
        msg.colors = [color for pt in msg.points]
        msg.scale.x = self.map.resolution 
        msg.scale.y = self.map.resolution 
        msg.scale.z = self.map.resolution 
        if msg.points is not None:  
            for pt in msg.points:
                assert((not np.isnan([pt.x, pt.y, pt.z]).any()) and np.isfinite([pt.x, pt.y, pt.z]).all())
       
        self.pub_expected_scan.publish(msg) 

    def publish_laser_pts(self):
        """Publishes the currently received laser scan points from the robot, after we subsampled
        them in order to comparse them with the expected laser scan from each particle."""
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.ns = 'laser_points'
        msg.id = 30000
        msg.type = 6
        msg.action = 0
        msg.type = Marker.POINTS
            
        msg.points = [ Point(x=self.odom.rx + r*np.cos(theta+self.odom.ra),
                        y=self.odom.ry + r*np.sin(theta+self.odom.ra),
                        z=0.2) for r,theta in zip(self.laser.subsampled_scan, self.laser.subsampled_angles)]
        
        # msg.points = msg.points[0:2]


        ''' commented out'''
        color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        msg.colors = [color for pt in msg.points]
        msg.scale.x = self.map.resolution 
        msg.scale.y = self.map.resolution 
        msg.scale.z = self.map.resolution 
        if msg.points is not None:  
            for pt in msg.points:
                assert((not np.isnan([pt.x, pt.y, pt.z]).any()) and np.isfinite([pt.x, pt.y, pt.z]).all())
       
        self.pub_laser_subsamples.publish(msg)       

    def pub_particles(self):                 
        particle_pose = PoseArray()
        particle_pose.header.frame_id = 'map'
        particle_pose.header.stamp = self.get_clock().now().to_msg()
        particle_pose.poses = []
        
        for par in self.pf.particles:
            pose = Pose()
            pose.position.x = par[1][0]
            pose.position.y = par[1][1]
            pose.position.z = 0.0

            qx, qy, qz, qw = tr.quaternion_from_euler(0, 0, par[1][2]) 
            pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

            particle_pose.poses.append(pose)
        
        self.particles_pub.publish(particle_pose)

        estimated_pose = PoseArray()       
        estimated_pose.header.frame_id = 'map'
        estimated_pose.header.stamp = self.get_clock().now().to_msg()
        estimated_pose.poses = []
        pose = Pose()
        pose.position.x = self.estimated_pose_mean[0]
        pose.position.y = self.estimated_pose_mean[1]
        qx, qy, qz, qw = tr.quaternion_from_euler(0, 0, self.estimated_pose_mean[2]) 
        
        pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        estimated_pose.poses.append(pose)
        self.pub_estimated_pose.publish(estimated_pose)

    


    def compute_scan_likelihood(self, expected_scan, measured_scan): #, sigma_hit, l_short):
        """
        Compute likelihood p(z|sample) for a specific measurement given sample state and landmarks.

        :param sample: Sample (unweighted particle) that must be propagated
        
        :return Likelihood
        """
        # self.get_logger().info(f"Measured scan {measured_scan} ({len(measured_scan)}) expected scan {expected_scan} ({len(expected_scan)})")
        assert len(expected_scan) == len(measured_scan)

        # Initialize measurement likelihood
        likelihood_sample = 1.0
        n = len(measured_scan)
        '''
          Mixture weights for the laser range sensor model
          Values copied from ROS amcl launch file
        '''
        # z_max = 0.05 * 0
        # z_rand = 0.05 * 0
        # z_hit = 1.0 # 0.95
        # z_short = 0.1 * 0

        # Loop over all landmarks for current particle
        for i in range(n):
            # Map difference true and expected distance measurement to probability
            ''' Compute Phit'''
            p_hit = \
                np.exp(-(expected_scan[i]- measured_scan[i]) * (expected_scan[i]-measured_scan[i]) /
                       (2 * self.sigma_hit * self.sigma_hit))      
            p_hit *= 1/np.sqrt(2*np.pi*self.sigma_hit*self.sigma_hit) 

            ''' 
                Pshort: unexpected measurements produced by dynamic objects 
                Its likelihood decreases with range
            '''
            p_short = np.exp(self.lambda_short*measured_scan[i])
            n_short = self.lambda_short/(1-np.exp(-self.lambda_short*expected_scan[i]))
            p_short *= n_short

            ''' 
                Pmax: sometimes obstacles are missed altogether (laser e.g:
                sensing black, light absorbing objects) Typically the sensor 
                returns a max_range measurement
            '''
            p_max = 0
            if measured_scan[i] == self.laser.range_max:
                p_max = 1.0

            ''' 
                Prand: range finders ocassionally produce entirely unexplainable
                measurements 
            '''
            p_rnd = 0
            if measured_scan[i] >= self.laser.range_max:
                p_rnd = 1.0/self.laser.range_max
            
            p = self.z_hit * p_hit + self.z_short * p_short + self.z_max * p_max + self.z_rand * p_rnd
        #     # Incorporate likelihoods current landmark
            likelihood_sample *= p
        # self.get_logger().info(f'Phit: {p_hit}, Pshort: {p_short}, Pmax: {p_max}, Prnd: {p_rnd}')
        # self.get_logger().info(f'Likelihood: {likelihood_sample}')
        # Return importance weight based on all landmarks
        return likelihood_sample
    
    def transform_pose(self, px, py, pa, source_frame='laser', target_frame='map'):
        ppose = Pose()
        
        ppose.position.x = px
        ppose.position.y = py
        ppose.position.z = 0.2
        q = tr.quaternion_from_euler(0, 0, pa)
        ppose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        try:
            self.tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=2.0))
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            _, _, lyaw = tr.euler_from_quaternion([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
            # self.get_logger().info(f'Laser to odom transform: {transform.transform} {lyaw}')
            # self.get_logger().info('Robot pose: %.4f %.4f %.4f' % (self.odom.rx, self.odom.ry, self.odom.ra))   
            
            # Transform the pose

            transformed_pose = do_transform_pose(ppose, transform)
            
            # Extract the transformed position
            tx = transformed_pose.position.x
            ty = transformed_pose.position.y
            
            # Extract transformed orientation and convert back to Euler angle (theta)
            transformed_q = transformed_pose.orientation
            _, _, ta = tr.euler_from_quaternion(
                [transformed_q.x, transformed_q.y, transformed_q.z, transformed_q.w]
            )
            
            return [tx, ty, ta]
        
        except Exception as e:
            self.get_logger().error('Odom to laser transform failed with exception: %s' % str(e)) 
            return []
    def compare_scans(self, measured_scan, expected_scan):
        l = len(measured_scan)
        self.get_logger().info("--------Comparison of scans--------")

        for i in range(l):
            self.get_logger().info(f"Measured scan {measured_scan[i]} expected scan {expected_scan[i]}")
        self.get_logger().info("--------END Comparison of scans--------")

    def update_pf(self): # dist, a):
        
        """
        Update the particle filter with a new measurement from the laser.
        Note that this function is not called in the current implementation.
        """
        new_particles = []
        bins_with_support = []
        number_of_new_particles = 0
        number_of_bins_with_support = 0
        number_of_required_particles = self.pf.minimum_number_of_particles
        prevx, prevy, prevyaw = self.compute_pose_from_msg(self.odom.prev_odom)
        newx, newy, newyaw = self.compute_pose_from_msg(self.odom.odom)
        dist = np.sqrt((newx - prevx)**2 + (newy - prevy)**2)
        dyaw = newyaw - prevyaw
        a = np.arctan2(np.sin(dyaw), np.cos(dyaw))
        # self.get_logger().info("Robot motion newx: %.2f, newy: %.2f, newyaw: %.2f, dist: %.2f, a: %.2f"%(newx, newy, newyaw, dist, a)) 
        # self.odom.has_moved = abs(dist)>1e-5 or abs(a)>1e-4
        self.odom.prev_odom = self.odom.odom
        
        
        while number_of_new_particles < number_of_required_particles:
            # Get sample from discrete distribution given by particle weights
            index_j = generate_sample_index(self.pf.particles)
            # Propagate state of selected particle
            propaged_state = self.pf.propagate_sample(self.pf.particles[index_j][1], dist, a) 
            # tparticle = propaged_state
            # tparticle = self.transform_pose(tparticle[0], tparticle[1], tparticle[2])
            
            # self.get_logger().info(f"Particle pose in odom frame: {propaged_state}")
            # self.get_logger().info(f"Particle pose in laser frame: {tparticle}")
            # expected_scan = self.map.simulate_scan_from_pose(propaged_state[0], propaged_state[1], propaged_state[2], self.laser.subsampled_angles,self.laser.range_min, self.laser.range_max)
            expected_scan = self.map.simulate_scan_with_bresenham(propaged_state[0], propaged_state[1], propaged_state[2], self.laser.subsampled_angles,self.laser.range_min, self.laser.range_max)

            # expected_scan = self.map.simulate_scan_from_pose(tparticle[0], tparticle[1], tparticle[2], self.laser.subsampled_angles,self.laser.range_min, self.laser.range_max)
            importance_weight = self.compute_scan_likelihood(expected_scan, self.laser.subsampled_scan) 
            # Add sample to list of new particles
            new_particles.append([importance_weight, propaged_state])
            number_of_new_particles += 1
            # Next, we convert the discrete distribution of all new samples into a histogram. We must check if the new
            # state (propagated_state) falls in a histogram bin with support or in an empty bin. We keep track of the
            # number of bins with support. Instead of adopting a (more efficient) tree, a simple list is used to
            # store all bin indices with support since there is are no performance requirements for our use case.

            # Map state to bin indices
            indices = [np.floor(propaged_state[0] / self.pf.resolutions[0]),
                       np.floor(propaged_state[1] / self.pf.resolutions[1]),
                       np.floor(propaged_state[2] / self.pf.resolutions[2])]

            # Add indices if this bin is empty (i.e. is not in list yet)
            if indices not in bins_with_support:
                bins_with_support.append(indices)
                number_of_bins_with_support += 1

            # Update number of required particles (only defined if number of bins with support above 1)
            if number_of_bins_with_support > 1:
                number_of_required_particles = compute_required_number_of_particles_kld(number_of_bins_with_support,
                                                                                        self.epsilon,
                                                                                        self.upper_quantile)
            # Make sure number of particles constraints are not violated
            number_of_required_particles = max(number_of_required_particles, self.pf.minimum_number_of_particles)
            number_of_required_particles = min(number_of_required_particles, self.pf.maximum_number_of_particles)
        # Store new particle set and normalize weights
        # self.get_logger().info("NEW NUMBER OF PARTICLES: %d"%(len(new_particles)))
        self.pf.particles = self.pf.normalize_weights(new_particles)
        self.estimated_pose_mean = self.pf.get_average_state()
        
        current_N = Int32(data=len(new_particles))
        self.pub_current_num_particles.publish(current_N)

        ## end comment
        # self.get_logger().info(f"Current number of particles: {len(new_particles)}")
        # expected_scan = self.map.simulate_scan_with_bresenham(self.estimated_pose_mean[0], self.estimated_pose_mean[1], self.estimated_pose_mean[2], self.laser.subsampled_angles,self.laser.range_min, self.laser.range_max)
        # self.get_logger().info(f"Estimated pose mean: {self.estimated_pose_mean}")
        # self.get_logger().info(f"Expected scan from mean pose: {expected_scan}")
        # color = ColorRGBA(r=0.0, g=0.7, b=0.3, a=1.0)
        # self.publish_expected_scan_from_pose(self.estimated_pose_mean[0], self.estimated_pose_mean[1], self.estimated_pose_mean[2],expected_scan, color)        
        # self.compare_scans(expected_scan, self.laser.subsampled_scan)
        if DEBUG_MODE:
            for i in range(len(self.pf.particles)):
                self.get_logger().info(f"Particle {i}: {self.pf.particles[i]}")
        # expected_scan2 = self.map.simulate_scan_from_pose(self.odom.rx,self.odom.ry,self.odom.ra, self.laser.subsampled_angles,self.laser.range_min, self.laser.range_max)
        # expected_scan = self.map.simulate_scan_with_bresenham(self.odom.rx,self.odom.ry,self.odom.ra, self.laser.subsampled_angles,self.laser.range_min, self.laser.range_max)
        # probot = self.compute_scan_likelihood(expected_scan, self.laser.subsampled_scan)
        # self.get_logger().info(f"Expected scan likelihood from robot pose: {probot}")
        # self.compare_scans(expected_scan, expected_scan2)
        # color = ColorRGBA(r=0.0, g=1.0, b=0.0,a=1.0)
        # self.publish_expected_scan_from_pose(self.odom.rx, self.odom.ry,self.odom.ra, expected_scan, color)  
        
        

    def run(self):
        self.update_pf()           
        self.pub_particles()

def main(args=None):
    rclpy.init(args=args)
    mcl = AdaptiveMonteCarloLocalization()
    rclpy.spin(mcl)
    mcl.destroy_node()
    rclpy.shutdown()
     
 
if __name__ == '__main__':
    main()



    