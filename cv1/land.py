import rclpy
from rclpy.node import Node
from tello_msgs.srv import TelloAction

class TelloLandClient(Node):

    def __init__(self):
        super().__init__('tello_land_client')
        self.client = self.create_client(TelloAction, '/tello_action')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = TelloAction.Request()

    def send_request(self):
        self.request.cmd = 'land'
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    tello_land_client = TelloLandClient()
    response = tello_land_client.send_request()
    if response:
        tello_land_client.get_logger().info(f'Response: {response}')
    else:
        tello_land_client.get_logger().info('No response received.')
    tello_land_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
