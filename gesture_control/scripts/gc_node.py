#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import mediapipe as mp
from ament_index_python.packages import get_package_share_directory


# --- MediaPipe setup ---
BaseOptions = mp.tasks.BaseOptions
GestureRecognizer = mp.tasks.vision.GestureRecognizer
GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
GestureRecognizerResult = mp.tasks.vision.GestureRecognizerResult
VisionRunningMode = mp.tasks.vision.RunningMode


class GestureTeleop(Node):
    def __init__(self):
        super().__init__("gesture_teleop_node")

        # --- ROS interfaces ---
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, "/image_raw", self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # --- Motion parameters ---
        self.linear_speed = 0.2
        self.angular_speed = 0.4
        self.current_action = "Waiting..."

        # --- Locate model file from package share dir ---
        pkg_share = get_package_share_directory("gesture_control")
        model_path = os.path.join(pkg_share, "model", "gesture_recognizer.task")

        if not os.path.exists(model_path):
            self.get_logger().error(f"❌ Model not found at: {model_path}")
            raise FileNotFoundError(model_path)

        # --- Setup MediaPipe Gesture Recognizer ---
        self.options = GestureRecognizerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=VisionRunningMode.LIVE_STREAM,
            result_callback=self.print_result,
        )

        self.recognizer = GestureRecognizer.create_from_options(self.options)
        self.get_logger().info("✅ Gesture teleoperation node started!")

    # --- MediaPipe result callback ---
    def print_result(self, result: GestureRecognizerResult, output_image: mp.Image, timestamp_ms: int):
        twist = Twist()

        if not result.gestures:
            self.current_action = "Unknown"
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        gesture = result.gestures[0][0].category_name

        # --- Map gesture to robot movement ---
        if gesture == "Thumb_Up":
            self.current_action = "Forward"
            twist.linear.x = self.linear_speed
        elif gesture == "Thumb_Down":
            self.current_action = "Backward"
            twist.linear.x = -self.linear_speed
        elif gesture == "Victory":
            self.current_action = "Turn Left"
            twist.angular.z = self.angular_speed
        elif gesture == "Pointing_Up":
            self.current_action = "Turn Right"
            twist.angular.z = -self.angular_speed
        elif gesture == "Open_Palm":
            self.current_action = "Stop"
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            self.current_action = "Unknown"

        self.cmd_pub.publish(twist)

    # --- ROS Image callback ---
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            timestamp_ms = int(self.get_clock().now().nanoseconds / 1e6)

            self.recognizer.recognize_async(mp_image, timestamp_ms)

            cv2.putText(frame, f"Gesture: {self.current_action}", (30, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 0), 3)
            cv2.imshow("Gesture Teleop", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Gesture teleop stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
