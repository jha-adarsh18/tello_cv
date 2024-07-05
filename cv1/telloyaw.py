import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.declare_parameter('yaw_rate_degrees', 45.0)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.msg = Twist()

    def timer_callback(self):
        yaw_rate_degrees = self.get_parameter('yaw_rate_degrees').get_parameter_value().double_value
        twist_msg = Twist()
        twist_msg.angular.z = (yaw_rate_degrees * 3.14159) / 180.0  # Convert degrees to radians per second
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Yaw command sent: {twist_msg}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
