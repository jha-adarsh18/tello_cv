import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.declare_parameter('climb_height_cm', 50.0)
        self.declare_parameter('climb_rate_mps', 0.5)  # Climb rate in meters per second
        self.send_climb_command()

    def send_climb_command(self):
        climb_height_cm = self.get_parameter('climb_height_cm').get_parameter_value().double_value
        climb_rate_mps = self.get_parameter('climb_rate_mps').get_parameter_value().double_value

        # Calculate climb duration (time = distance / speed)
        climb_duration_sec = climb_height_cm / 100.0 / climb_rate_mps
        
        twist_msg = Twist()
        twist_msg.linear.z = climb_rate_mps  # Set the climb rate
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Climb command sent: {twist_msg}')

        # Sleep for the calculated duration to achieve the desired climb height
        time.sleep(climb_duration_sec)

        # Stop the climb by sending a zero linear velocity
        twist_msg.linear.z = 0.0
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Stop climb command sent: {twist_msg}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    # Give some time for the command to be processed
    rclpy.spin_once(node, timeout_sec=2)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

