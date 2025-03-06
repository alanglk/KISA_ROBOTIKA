import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker
import tf2_ros
from tf_transformations import euler_from_quaternion
import numpy as np

class LaserFrameTransformer(Node):
    def __init__(self):
        super().__init__('laser_frame_transformer')

        # Setup the TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info("TF buffer and listener initialized")
        # Laser subscriber
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        self.get_logger().info("Subscribed to /scan topic")
        # Publisher for transformed points
        self.marker_publisher = self.create_publisher(Marker, '/transformed_laser_points', 10)

    def laser_callback(self, laser_scan):
        # self.get_logger().info("Received laser scan")
        # Transform laser scan points
        points_in_target_frame = self.transform_laser_to_frame(laser_scan, 'odom')
        
        # Publish points as a Marker for visualization
        if points_in_target_frame:
            marker_msg = self.create_marker_msg(points_in_target_frame)
            self.marker_publisher.publish(marker_msg)

    def transform_laser_to_frame(self, laser_scan, target_frame):
        try:
            self.get_logger().info(f"Transforming laser scan to {target_frame}")
            # Get the transformation from laser frame to target frame (e.g., odom)
            transform = self.tf_buffer.lookup_transform(
                target_frame,  # target frame
                laser_scan.header.frame_id,  # source frame (laser frame)
                rclpy.time.Time(),  # time at which to get the transform
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

        except Exception as e:
            self.get_logger().error(f"Transform lookup failed: {str(e)}")
            return None

        # Extract translation and rotation from the transform
        trans = transform.transform.translation
        rot = transform.transform.rotation
        _, _, yaw = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

        # Calculate x, y coordinates for each range reading
        points_in_target_frame = []
        for i, r in enumerate(laser_scan.ranges):
            angle = laser_scan.angle_min + i * laser_scan.angle_increment
            if laser_scan.range_min <= r <= laser_scan.range_max:
                # Transform laser point to the laser frame (x, y)
                x_laser = r * np.cos(angle)
                y_laser = r * np.sin(angle)

                # Transform to the target frame (applying rotation and translation)
                x_target = x_laser * np.cos(yaw) - y_laser * np.sin(yaw) + trans.x
                y_target = x_laser * np.sin(yaw) + y_laser * np.cos(yaw) + trans.y

                points_in_target_frame.append((x_target, y_target, 0.0))
        self.get_logger().info(f"Transformed laser scan to {target_frame}: {points_in_target_frame}")
        return points_in_target_frame

    def create_marker_msg(self, points):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'laser_points'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        # Set the marker scale
        marker.scale.x = 0.05
        marker.scale.y = 0.05

        # Set the marker color
        marker.color.a = 1.0  # Alpha
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Fill marker points
        marker.points = [Point(x=pt[0], y=pt[1], z=pt[2]) for pt in points]

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = LaserFrameTransformer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
