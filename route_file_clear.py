import rclpy
from rclpy.node import Node
from autoware_adapi_v1_msgs.srv import ClearRoute

class ClearRouteClient(Node):
    def __init__(self):
        super().__init__('clear_route_client')
        # Create a client for the ClearRoute service
        self.client = self.create_client(ClearRoute, '/api/routing/clear_route')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ClearRoute service not available, waiting again...')

    def send_clear_request(self):
        # Create an empty service request (assuming no parameters are required)
        req = ClearRoute.Request()
        future = self.client.call_async(req)
        return future

def main():
    rclpy.init()
    client = ClearRouteClient()

    try:
        future = client.send_clear_request()
        rclpy.spin_until_future_complete(client, future)
        response = future.result()
        if response:
            client.get_logger().info('Route cleared successfully.')
        else:
            client.get_logger().info('Failed to clear route.')
    except Exception as e:
        client.get_logger().error('Service call failed: %s' % str(e))
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
