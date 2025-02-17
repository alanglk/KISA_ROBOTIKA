# EJEMPLO CLASE 13/02/2025
# Subscriptor de ejemplo
#

import rclpy
from std_msgs.msg import String
from rclpy.node import Node

DEFAULT_TOPIC = "/ejemplo"

class Lector(Node):
    def __init__(self):
        super().__init__('Ejemplo_Lector')
        self.lector_sub = self.create_subscription( String, DEFAULT_TOPIC, self.lector_callback, 10)

    def lector_callback(self, msg:String):
        print(f"Received: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = Lector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
