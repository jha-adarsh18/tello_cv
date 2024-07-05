import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.declare_parameter('yaw_rate_degrees', 45.0)
        self.msg = Twist()
        self.send_velocity_command()

    def send_velocity_command(self):
        yaw_rate_degrees = self.get_parameter('yaw_rate_degrees').get_parameter_value().double_value
        twist_msg = Twist()
        twist_msg.angular.z = (yaw_rate_degrees * 3.14159) / 180.0  # Convert degrees to radians per second
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Yaw command sent: {twist_msg}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    # Allow some time for the message to be sent before shutting down
    rclpy.spin_once(node, timeout_sec=1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

