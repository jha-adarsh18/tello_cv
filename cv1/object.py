import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import cvzone

# Initialize YOLO model for object detection
model = YOLO("yolov8n.pt")  # Updated to use the uploaded file path
classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"
              ]

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()
        self.prev_frame_time = 0

        self.depth_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.callback,
            10)

        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Object Detection Node has been started.')

    def callback(self, img_msg):
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        # Object detection
        results = model(img, stream=True)

        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Class Name
                cls_idx = int(box.cls[0])
                detected_class = classNames[cls_idx]

                # Check if the detected class is "bottle"
                if detected_class == 'bottle':
                    # Bounding Box
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    w, h = x2 - x1, y2 - y1

                    # Crop the region within the bounding box for color detection
                    cropped_region = img[y1:y2, x1:x2]
                    # Calculate center coordinates
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2

                    # Get the average color of the cropped region
                    avg_color = np.mean(cropped_region, axis=(0, 1))
                    b, g, r = avg_color.astype(int)

                    # Draw rectangle above the bounding box with object color
                    rect_color = (int(r), int(g), int(b))
                    cv2.rectangle(img, (x1 + 10, y1 - 50), (x2, y1), list(rect_color), -1)

                    # Display color information and set background color
                    text = f'{detected_class}'
                    cv2.putText(img, text, (x1 + 10, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

                    # Draw bounding box
                    cvzone.cornerRect(img, (x1, y1, w, h))

                    # Calculate velocity commands based on the position of the bottle
                    twist_msg = Twist()
                    
                    # Example logic to control the drone based on the bottle's position
                    if center_x < img.shape[1] / 3:
                        twist_msg.angular.z = 0.1  # Rotate left
                    elif center_x > 2 * img.shape[1] / 3:
                        twist_msg.angular.z = -0.1  # Rotate right
                    else:
                        twist_msg.angular.z = 0.0  # Stop rotation

                    if center_y < img.shape[0] / 3:
                        twist_msg.linear.z = 0.1  # Move up
                    elif center_y > 2 * img.shape[0] / 3:
                        twist_msg.linear.z = -0.1  # Move down
                    else:
                        twist_msg.linear.z = 0.1  # Stop vertical movement

                    self.cmd_vel_pub.publish(twist_msg)
        
        cv2.imshow("TelloCamera", img)
        cv2.waitKey(1)

    def stop(self):
        pass  # Add any cleanup code if needed

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

