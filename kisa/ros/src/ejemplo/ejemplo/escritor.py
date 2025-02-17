# EJEMPLO CLASE 13/02/2025
# Subscriptor de ejemplo
#

import rclpy
from std_msgs.msg import String
from rclpy.node import Node

DEFAULT_TOPIC = "/ejemplo"

class Escritor(Node):
    def __init__(self):
        super().__init__('Ejemplo_Escritor')
        self.escritor_pub = self.create_publisher(String, DEFAULT_TOPIC, 10)
        self.timer = self.create_timer(0.1, self.seng_msg)
        self.num_msgs = 0

    def seng_msg(self,):
        self.num_msgs += 1
        msg = String(data=str(self.num_msgs))
        self.escritor_pub.publish(msg)
        print(f"Writing: {self.num_msgs}")

def main(args=None):
    rclpy.init(args=args)
    node = Escritor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
