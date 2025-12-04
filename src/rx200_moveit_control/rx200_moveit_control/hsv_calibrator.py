

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class HSVCalibrator(Node):
    def __init__(self):
        super().__init__('hsv_calibrator')
        self.bridge = CvBridge()
        self.snapshot_dir = '/tmp/hsv_debug'
        os.makedirs(self.snapshot_dir, exist_ok=True)
        
        # HSV range sliders (we'll adjust these)
        self.h_lower = 0
        self.h_upper = 180
        self.s_lower = 0
        self.s_upper = 255
        self.v_lower = 0
        self.v_upper = 255
        
        self.frame_count = 0
        self.last_image = None
        
        # Subscribe to camera
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info(
            'HSV Calibrator started.\n'
            'Press SPACE to save masks for current frame.\n'
            'Edit the HSV ranges below and rerun to see changes.'
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_count += 1
            
            # Convert BGR to HSV
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Save frames for analysis every 30 frames
            if self.frame_count % 30 == 0:
                self.save_hsv_analysis(cv_image, hsv_image)
                
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

    def save_hsv_analysis(self, bgr_image, hsv_image):
        """Save HSV values and masks for analysis"""
        
        # Get image dimensions
        height, width = bgr_image.shape[:2]
        
        # Create figure showing HSV channels separately
        h_channel = hsv_image[:, :, 0]
        s_channel = hsv_image[:, :, 1]
        v_channel = hsv_image[:, :, 2]
        
        # Save each channel
        cv2.imwrite(
            os.path.join(self.snapshot_dir, 'hue_channel.jpg'),
            h_channel
        )
        cv2.imwrite(
            os.path.join(self.snapshot_dir, 'saturation_channel.jpg'),
            s_channel
        )
        cv2.imwrite(
            os.path.join(self.snapshot_dir, 'value_channel.jpg'),
            v_channel
        )
        
        # Save original
        cv2.imwrite(
            os.path.join(self.snapshot_dir, 'original.jpg'),
            bgr_image
        )
        
        # Create RED mask and save
        red_lower1 = np.array([0, 50, 50])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([170, 50, 50])
        red_upper2 = np.array([180, 255, 255])
        mask_red = cv2.inRange(hsv_image, red_lower1, red_upper1) | \
                   cv2.inRange(hsv_image, red_lower2, red_upper2)
        cv2.imwrite(os.path.join(self.snapshot_dir, 'mask_red.jpg'), mask_red)
        
        # Create YELLOW mask and save
        yellow_lower = np.array([15, 100, 100])
        yellow_upper = np.array([35, 255, 255])
        mask_yellow = cv2.inRange(hsv_image, yellow_lower, yellow_upper)
        cv2.imwrite(os.path.join(self.snapshot_dir, 'mask_yellow.jpg'), mask_yellow)
        
        # Create BLUE mask and save
        blue_lower = np.array([100, 150, 50])
        blue_upper = np.array([130, 255, 255])
        mask_blue = cv2.inRange(hsv_image, blue_lower, blue_upper)
        cv2.imwrite(os.path.join(self.snapshot_dir, 'mask_blue.jpg'), mask_blue)
        
        self.get_logger().info(
            f'Frame {self.frame_count}: HSV analysis saved to {self.snapshot_dir}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = HSVCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
