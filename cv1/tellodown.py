import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.declare_parameter('move_distance_cm', 50.0)  # Adjust the distance as needed
        self.declare_parameter('move_rate_mps', 0.5)  # Move rate in meters per second
        self.send_move_up_command()

    def send_move_up_command(self):
        move_distance_cm = self.get_parameter('move_distance_cm').get_parameter_value().double_value
        move_rate_mps = self.get_parameter('move_rate_mps').get_parameter_value().double_value

        # Calculate move duration (time = distance / speed)
        move_duration_sec = move_distance_cm / 100.0 / move_rate_mps
        
        twist_msg = Twist()
        twist_msg.linear.z = move_rate_mps  # Set the upward movement rate
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Move up command sent: {twist_msg}')

        # Sleep for the calculated duration to achieve the desired move distance
        time.sleep(move_duration_sec)

        # Stop the upward movement by sending a zero linear velocity
        twist_msg.linear.z = 0.0
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Stop move up command sent: {twist_msg}')

    def send_move_down_command(self):
        move_distance_cm = self.get_parameter('move_distance_cm').get_parameter_value().double_value
        move_rate_mps = self.get_parameter('move_rate_mps').get_parameter_value().double_value

        # Calculate move duration (time = distance / speed)
        move_duration_sec = move_distance_cm / 100.0 / move_rate_mps
        
        twist_msg = Twist()
        twist_msg.linear.z = -move_rate_mps  # Set the downward movement rate (negative)
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Move down command sent: {twist_msg}')

        # Sleep for the calculated duration to achieve the desired move distance
        time.sleep(move_duration_sec)

        # Stop the downward movement by sending a zero linear velocity
        twist_msg.linear.z = 0.0
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Stop move down command sent: {twist_msg}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()

    node.send_move_down_command()

    rclpy.spin_once(node, timeout_sec=2)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
