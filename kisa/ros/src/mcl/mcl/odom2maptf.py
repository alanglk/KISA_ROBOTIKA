import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class StaticOdomToMap(Node):
    def __init__(self):
        super().__init__('odom2map_broadcaster_node')

        # Create a static transform broadcaster
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Define the static transform (odom -> map)
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'map'
        static_transform.child_frame_id = 'odom'

        # Translation
        static_transform.transform.translation.x = -4.0
        static_transform.transform.translation.y = -4.0
        static_transform.transform.translation.z = 0.0

        # Rotation (Quaternion for 0 radians yaw)
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0

        # Broadcast the transform
        self.static_broadcaster.sendTransform(static_transform)
        self.get_logger().info("Broadcasting static transform from 'map' to 'odom'")

def main():
    rclpy.init()
    node = StaticOdomToMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
