import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from autoware_adapi_v1_msgs.srv import SetRoute
from autoware_adapi_v1_msgs.msg import RouteSegment, RoutePrimitive
import yaml

def load_route_data(filename):
    with open(filename, 'r') as file:
        data = yaml.safe_load(file)
    return data

def create_pose(data):
    pose = Pose()
    pose.position.x = data['position']['x']
    pose.position.y = data['position']['y']
    pose.position.z = data['position']['z']
    pose.orientation.w = data['orientation']['w']
    pose.orientation.x = data['orientation']['x']
    pose.orientation.y = data['orientation']['y']
    pose.orientation.z = data['orientation']['z']
    return pose

def create_route_segments(data):
    segments = []
    for section in data['segments']:  # Change from 'route_sections' to 'segments'
        segment = RouteSegment()
        # Create a RoutePrimitive for the preferred route with the correct type and ID
        segment.preferred = RoutePrimitive(id=int(section['preferred']['id']), type=section['preferred']['type'])
        # Create RoutePrimitive for alternatives, if any
        segment.alternatives = [RoutePrimitive(id=int(alt['id']), type=alt['type']) for alt in section.get('alternatives', [])]
        segments.append(segment)
    return segments

class RouteClient(Node):
    def __init__(self):
        super().__init__('route_client')
        self.client = self.create_client(SetRoute, '/api/routing/set_route')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
    
    def send_route_request(self, pose, segments):
        req = SetRoute.Request()
        req.header.frame_id = 'map'
        req.header.stamp = self.get_clock().now().to_msg()
        req.goal = pose
        req.segments = segments
        future = self.client.call_async(req)
        return future

def main(args=None):
    rclpy.init(args=args)
    route_data = load_route_data('/home/zulfaqar/Downloads/odaiba_reference_1-2_Teleport_v1.6.route_v1')
    # route_data = load_route_data('/home/zulfaqar/Downloads/odaiba_reference_1-1_Miraikan_v1.6.route_v1')
    pose = create_pose(route_data['goal_pose'])
    segments = create_route_segments(route_data)

    route_client = RouteClient()
    future = route_client.send_route_request(pose, segments)
    rclpy.spin_until_future_complete(route_client, future)

    response = future.result()
    if response:
        print('Service call successful')
    else:
        print('Service call failed')
    
    route_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
