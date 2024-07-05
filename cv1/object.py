import rclpy
from rclpy.node import Node
import cv2
from ultralytics import YOLO
from djitellopy import Tello
import pygame
from PIL import Image
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.model = YOLO('/home/adarsh/Documents/runs/detect/train3/weights/best_openvino_model')
        self.tello = Tello()
        self.tello.connect()
        self.get_logger().info(f"Battery level: {self.tello.get_battery()}%")
        self.tello.streamon()

        pygame.init()
        self.screen = pygame.display.set_mode((960, 720))
        pygame.display.set_caption("Tello Camera Feed")

        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

    def timer_callback(self):
        frame_read = self.tello.get_frame_read()
        frame = frame_read.frame

        if frame is None:
            self.get_logger().warn("No frame captured from Tello camera.")
            return

        frame_pil = Image.fromarray(frame)
        frame_rgb = frame_pil.convert("RGB")
        frame_rgb = frame_rgb.transpose(Image.FLIP_LEFT_RIGHT)
        frame_rgb_np = np.array(frame_rgb)

        results = self.model(frame_rgb_np)
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                conf = box.conf[0]
                cls = box.cls[0]
                cv2.rectangle(frame_rgb_np, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                label = f"{self.model.names[int(cls)]} {conf:.2f}"
                cv2.putText(frame_rgb_np, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        frame_surface = pygame.surfarray.make_surface(frame_rgb_np)
        frame_surface = pygame.transform.rotate(frame_surface, -90)
        self.screen.blit(frame_surface, (0, 0))
        pygame.display.update()

        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_q):
                self.tello.streamoff()
                self.tello.end()
                pygame.quit()
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

