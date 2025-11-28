#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.bridge = CvBridge()
        self.frame_cnt = 0

        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_cb,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_cb,
            10
        )

    def color_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge color: {e}')
            return

        self.frame_cnt += 1
        if self.frame_cnt % 30 == 0:
            h, w = img.shape[:2]
            self.get_logger().info(f'Color frames ok, size={w}x{h}')

    def depth_cb(self, msg):
        # Just touch the depth frame to confirm subscription; no processing yet
        pass


def main():
    rclpy.init()
    node = VisionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
