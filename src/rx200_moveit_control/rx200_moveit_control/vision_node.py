import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.snapshot_dir = '/tmp/vision_snapshots'
        os.makedirs(self.snapshot_dir, exist_ok=True)
        self.frame_count = 0
        
        # Define RGB ranges for each color
        # Format: (B, G, R) because OpenCV uses BGR
        self.color_ranges = {
            'red': {
                'lower': np.array([0, 0, 150]),      # BGR: (B, G, R)
                'upper': np.array([100, 100, 255]),
                'color_bgr': (0, 0, 255)  # Blue in BGR
            },
            'yellow': {
                'lower': np.array([0, 150, 150]),    # BGR
                'upper': np.array([100, 255, 255]),
                'color_bgr': (0, 255, 255)
            },
            'blue': {
                'lower': np.array([57, 0, 0]),      # BGR
                'upper': np.array([255, 120, 120]),
                'color_bgr': (255, 0, 0)
            }
        }
        
        # Subscribe to camera
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # Timer for status updates
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.last_frame_count = 0
        
        self.get_logger().info('Vision Node started - RGB Detection')

    def timer_callback(self):
        """Status check every 2 seconds"""
        if self.frame_count == self.last_frame_count:
            self.get_logger().warn(f'No images: {self.frame_count} frames total')
        else:
            self.get_logger().info(f'Processing: {self.frame_count} frames')
        self.last_frame_count = self.frame_count

    def find_cubes(self, bgr_image, color_name):
        """
        Find cube of specific color in BGR image (RGB)
        Returns: centroid (x, y), bounding box (x, y, w, h), area
        """
        color_range = self.color_ranges[color_name]
        lower = color_range['lower']
        upper = color_range['upper']
        
        # Create mask for this color in BGR space
        mask = cv2.inRange(bgr_image, lower, upper)
        
        # Morphological operations to clean up noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Get largest contour (the cube)
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Filter out very small detections (noise)
        if area < 500:  # Adjust threshold as needed
            return None
        
        # Get bounding box
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        # Get centroid
        M = cv2.moments(largest_contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx, cy = x + w // 2, y + h // 2
        
        return {
            'centroid': (cx, cy),
            'bbox': (x, y, w, h),
            'area': area,
            'contour': largest_contour
        }

    def image_callback(self, msg):
        """Process each frame"""
        try:
            # Convert ROS Image to OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_count += 1
            
            # NO CONVERSION NEEDED - We're working directly with BGR (which is RGB)
            
            # Detect each colored cube
            detections = {}
            for color_name in ['red', 'yellow', 'blue']:
                detection = self.find_cubes(cv_image, color_name)
                detections[color_name] = detection
            
            # Log detections every 30 frames
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Frame {self.frame_count}:')
                for color_name, detection in detections.items():
                    if detection:
                        cx, cy = detection['centroid']
                        area = detection['area']
                        self.get_logger().info(
                            f'  {color_name.upper()}: centroid=({cx}, {cy}), area={int(area)}'
                        )
                    else:
                        self.get_logger().info(f'  {color_name.upper()}: NOT DETECTED')
            
            # Save annotated snapshot every 30 frames
            if self.frame_count % 30 == 0:
                annotated_image = cv_image.copy()
                
                # Draw detections on image
                for color_name, detection in detections.items():
                    if detection:
                        cx, cy = detection['centroid']
                        x, y, w, h = detection['bbox']
                        color_bgr = self.color_ranges[color_name]['color_bgr']
                        
                        # Draw bounding box
                        cv2.rectangle(annotated_image, (x, y), (x + w, y + h), color_bgr, 2)
                        
                        # Draw centroid circle
                        cv2.circle(annotated_image, (cx, cy), 5, color_bgr, -1)
                        
                        # Add label
                        cv2.putText(
                            annotated_image,
                            color_name.upper(),
                            (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            color_bgr,
                            2
                        )
                
                # Save annotated image
                filename = os.path.join(
                    self.snapshot_dir,
                    f'detection_{self.frame_count}.jpg'
                )
                cv2.imwrite(filename, annotated_image)
                self.get_logger().info(f'Snapshot saved: {filename}')
            
        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
