# EJEMPLO CLASE 13/02/2025
# Subscriptor de ejemplo
#

import rclpy
from sphere_interfaces.msg import Sphere
from rclpy.node import Node

DEFAULT_TOPIC = "/ejemplo_sphere"

class LectorSphere(Node):
    def __init__(self):
        super().__init__('Ejemplo_Lector_Sphere')
        self.lector_sub = self.create_subscription( Sphere, DEFAULT_TOPIC, self.lector_callback, 10)

    def lector_callback(self, msg:Sphere):
        print(f"Received Sphere ->  center: {msg.center} radius: {msg.radius}")

def main(args=None):
    rclpy.init(args=args)
    node = LectorSphere()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
