import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from djitellopy import Tello
import cv2
from ultralytics import YOLO

class TelloNode(Node):

    def __init__(self):
        super().__init__('tello_node')
        self.declare_parameter('model_path', r'C:\Users\jhaad\Downloads\best_int8_openvino_model-20240704T093940Z-001\best_int8_openvino_model')
        self.model = YOLO(self.get_parameter('model_path').get_parameter_value().string_value)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tello = Tello()
        self.tello.connect()
        self.tello.streamon()

        self.get_logger().info(f"Battery level: {self.tello.get_battery()}%")

        self.timer = self.create_timer(0.1, self.process_frame)

        self.startcounter = 0
        self.direction = "initial"

    def process_frame(self):
        frame = self.tello.get_frame_read().frame
        if frame is None:
            self.get_logger().warn("No frame captured from Tello camera.")
            return

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.model(frame_rgb)
        
        twist_msg = Twist()
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                direction = self.get_direction((x1, y1, x2, y2), frame.shape[1], frame.shape[0])

                cv2.rectangle(frame_rgb, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame_rgb, direction, (int(x1), int(y1) - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                twist_msg = self.compute_twist(direction)

        cv2.imshow("Tello Camera Feed", frame_rgb)
        
        if self.startcounter == 0:
            self.tello.takeoff()
            self.startcounter = 1

        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Published command: {twist_msg}')
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.tello.land()
            self.get_logger().info("Landing...")
            rclpy.shutdown()

    def get_direction(self, box, frame_width, frame_height):
        x1, y1, x2, y2 = box
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2

        cell_width = frame_width // 3
        cell_height = frame_height // 3

        cell_x = int(cx // cell_width)
        cell_y = int(cy // cell_height)

        if cell_y == 0:
            direction_y = "Upwards"
        elif cell_y == 2:
            direction_y = "Downwards"
        else:
            direction_y = ""

        if cell_x == 0:
            direction_x = "Left"
        elif cell_x == 2:
            direction_x = "Right"
        else:
            direction_x = ""

        if direction_x and direction_y:
            return f"{direction_y}-{direction_x}"
        elif direction_x:
            return direction_x
        elif direction_y:
            return direction_y
        else:
            return "Center"

    def compute_twist(self, direction):
        twist = Twist()
        if direction == "Left":
            twist.angular.z = -60
        elif direction == "Right":
            twist.angular.z = 60
        elif direction == "Upwards":
            twist.linear.z = 60
        elif direction == "Downwards":
            twist.linear.z = -60
        else:
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.z = 0
        return twist

def main(args=None):
    rclpy.init(args=args)
    node = TelloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.tello.streamoff()
        node.tello.end()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
