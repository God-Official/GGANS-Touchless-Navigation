#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.timer = self.create_timer(0.03, self.timer_callback)  # ~30 FPS
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("❌ Could not open camera")
        else:
            self.get_logger().info("📷 CameraPublisher started — publishing on /image_raw")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ Failed to read frame")
            return
        frame = cv2.flip(frame, 1)
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
