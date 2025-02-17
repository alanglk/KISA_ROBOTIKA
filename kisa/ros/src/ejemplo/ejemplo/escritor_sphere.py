# EJEMPLO CLASE 13/02/2025
# Subscriptor de ejemplo
#

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sphere_interfaces.msg import Sphere

import numpy as np

DEFAULT_TOPIC = "/ejemplo_sphere"

class EscritorSphere(Node):
    def __init__(self):
        super().__init__('Ejemplo_Escritor_Sphere')
        self.escritor_pub = self.create_publisher(Sphere, DEFAULT_TOPIC, 10)
        self.timer = self.create_timer(0.1, self.seng_msg)
        self.num_msgs = 0
        self.angle = 0.0

    def seng_msg(self,):
        self.num_msgs += 1
        self.angle += 0.01
        msg = Sphere(center = Point(x=np.cos(self.angle), y=np.sin(self.angle), z=0.5), radius=0.5)
        self.escritor_pub.publish(msg)
        print(f"Writing {self.num_msgs} Sphere")

def main(args=None):
    rclpy.init(args=args)
    node = EscritorSphere()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
