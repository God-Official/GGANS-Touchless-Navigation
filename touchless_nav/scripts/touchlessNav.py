#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import os


from map_process import MapProcessor
from handclick import HandClickDetector


class MapOverlayNode(Node):
    """ROS 2 node that overlays map and reacts to touchless tap gestures."""

    def __init__(self):
        super().__init__('map_overlay_node')
        self.bridge = CvBridge()

        pkg_share = get_package_share_directory('turtlebot_nav2')
        map_path = os.path.join(pkg_share, 'maps', 'tb3_world.pgm')
        yaml_path = os.path.join(pkg_share, 'maps', 'tb3_world.yaml')

        self.declare_parameter("map_path", map_path)
        self.declare_parameter("yaml_path", yaml_path)
        self.declare_parameter("alpha", 0.6)
        self.declare_parameter("image_topic", "/image_raw")

        map_path = self.get_parameter("map_path").get_parameter_value().string_value
        yaml_path = self.get_parameter("yaml_path").get_parameter_value().string_value
        self.alpha = self.get_parameter("alpha").get_parameter_value().double_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value

        self.processor = MapProcessor(map_path, yaml_path)
        self.overlay = self.processor.overlay
        # self.overlay = cv2.flip(self.overlay, 0)
        self.window_name = "Touchless Map Overlay"
        self.map_info = {
            "resolution": self.processor.resolution,
            "origin": self.processor.origin,
            "height": self.processor.height,
            "width": self.processor.width
        }


        self.detector = HandClickDetector()
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.get_logger().info("📍 Ready to publish Nav2 goals")

        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.get_logger().info(f"🖼️ Subscribed to {image_topic}")
        self.get_logger().info("✋ Use hand tap gesture to click on image (ESC to exit).")


    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        blended = self._blend_frame_with_overlay(frame)
        processed_frame, coords = self.detector.process_frame(blended)

        if coords:
            x, y = coords
            wx, wy = self.processor.image_to_world(x, y)
            self.get_logger().info(f"🟢 Tap at pixel ({x},{y}) → world ({wx:.3f},{wy:.3f}) m")
            cv2.circle(processed_frame, (x, y), 10, (0, 255, 0), 2)

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = wx
            goal.pose.position.y = wy
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.get_logger().info(f"🎯 Sent Nav2 goal: ({wx:.2f}, {wy:.2f})")


        cv2.imshow(self.window_name, processed_frame)

        if cv2.waitKey(1) & 0xFF == 27:
            self.get_logger().info("🛑 Exiting overlay viewer")
            rclpy.shutdown()

    def _blend_frame_with_overlay(self, frame):
        fh, fw = frame.shape[:2]
        mh, mw = self.overlay.shape[:2]
        target_aspect = mw / mh

        new_w = int(fh * target_aspect)
        if new_w <= fw:
            x_start = (fw - new_w) // 2
            frame_cropped = frame[:, x_start:x_start + new_w]
        else:
            new_h = int(fw / target_aspect)
            y_start = (fh - new_h) // 2
            frame_cropped = frame[y_start:y_start + new_h, :]

        frame_resized = cv2.resize(frame_cropped, (mw, mh))
        gray_map = cv2.cvtColor(self.overlay, cv2.COLOR_BGR2GRAY)
        mask = gray_map < 250
        blended = frame_resized.copy()
        blended[mask] = cv2.addWeighted(frame_resized[mask], 1 - self.alpha,
                                        self.overlay[mask], self.alpha, 0)
        return blended

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MapOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
