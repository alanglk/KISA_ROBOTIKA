import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap

class MapClient(Node):
    def __init__(self):
        super().__init__('map_client_node')
        
        # Create a service client for GetMap, pointing to /map_server/map
        self.client = self.create_client(GetMap, '/map_server/map')
        
        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('######Waiting for /map_server/map service...')
        
        # Call the service once the map is available
        self.get_logger().info('######/map_server/map service is available, sending request...')
        self.call_map_service()

    def call_map_service(self):
        # Create a request for the GetMap service
        req = GetMap.Request()

        # Send the request and wait for the response asynchronously
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        # Check if the result is valid
        if future.result() is not None:
            self.process_map(future.result().map)
        else:
            self.get_logger().error('Failed to get map!')

    def process_map(self, map_data):
        # Process the map data (OccupancyGrid)
        self.get_logger().info(f'######### Map received: {map_data.info.width} x {map_data.info.height}')
        # You can access the map's data here, like the occupancy grid
        # map_data.data contains the occupancy grid in row-major order
        # map_data.info contains map metadata like resolution, origin, etc.

def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the MapClient
    map_client = MapClient()
    map_client.get_logger().info('####### Map client created')
    # Keep the node alive until shutdown
    rclpy.spin(map_client)

    # Clean up
    map_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()