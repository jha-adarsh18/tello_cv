import rclpy
from rclpy.node import Node
from tello_msgs.srv import TelloAction

class TelloActionClient(Node):

    def __init__(self):
        super().__init__('tello_action_client')
        self.client = self.create_client(TelloAction, 'tello_action')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.send_request()

    def send_request(self):
        request = TelloAction.Request()
        request.cmd = 'takeoff'
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Response: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TelloActionClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
