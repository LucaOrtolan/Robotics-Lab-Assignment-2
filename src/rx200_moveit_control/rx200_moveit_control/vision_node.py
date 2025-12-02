#!/usr/bin/env python3
"""
Vision Node - RGB Detection + 3D Position Estimation
Detects cubes and converts to world coordinates
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import tf2_ros
from tf2_geometry_msgs import do_transform_point

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.snapshot_dir = '/tmp/vision_snapshots'
        os.makedirs(self.snapshot_dir, exist_ok=True)
        self.frame_count = 0
        
        # Camera intrinsics (will be updated from camera_info)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        # Define RGB ranges for each color
        self.color_ranges = {
            'red': {
                'lower': np.array([0, 0, 150]),
                'upper': np.array([100, 100, 255]),
                'color_bgr': (0, 0, 255)
            },
            'yellow': {
                'lower': np.array([0, 150, 150]),
                'upper': np.array([100, 255, 255]),
                'color_bgr': (0, 255, 255)
            },
            'blue': {
                'lower': np.array([57, 0, 0]),      # BGR
                'upper': np.array([255, 120, 120]),
                'color_bgr': (255, 0, 0)
            }
        }
        
        # Subscribe to camera topics
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.depth_subscriber = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_infra1/image_raw',
            self.depth_callback,
            10
        )
        
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/camera/camera/camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publishers for detected cube poses
        self.red_publisher = self.create_publisher(PoseStamped, '/cube/red/pose', 10)
        self.yellow_publisher = self.create_publisher(PoseStamped, '/cube/yellow/pose', 10)
        self.blue_publisher = self.create_publisher(PoseStamped, '/cube/blue/pose', 10)
        
        # TF2 buffer and listener for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Store latest depth image
        self.latest_depth = None
        
        # Timer for status updates
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.last_frame_count = 0
        
        self.get_logger().info('Vision Node started - RGB Detection + 3D Positioning')

    def timer_callback(self):
        """Status check every 2 seconds"""
        if self.frame_count == self.last_frame_count:
            self.get_logger().warn(f'No images: {self.frame_count} frames total')
        else:
            self.get_logger().info(f'Processing: {self.frame_count} frames')
        self.last_frame_count = self.frame_count

    def camera_info_callback(self, msg):
        """Extract camera intrinsics"""
        if self.fx is None:  # Only set once
            self.fx = msg.K[0]
            self.fy = msg.K[4]
            self.cx = msg.K[2]
            self.cy = msg.K[5]
            self.get_logger().info(
                f'Camera intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}'
            )

    def depth_callback(self, msg):
        """Store the latest depth image"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f'Error in depth_callback: {str(e)}')

    def pixel_to_3d(self, u, v, depth_mm):
        """
        Convert pixel coordinates (u, v) and depth to 3D point in camera frame
        Args:
            u, v: pixel coordinates
            depth_mm: depth value in millimeters
        Returns:
            (x, y, z) in meters in camera frame
        """
        if self.fx is None or depth_mm == 0:
            return None
        
        # Convert depth from mm to meters
        z = depth_mm / 1000.0
        
        # Unproject using camera intrinsics
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        
        return np.array([x, y, z])

    def find_cubes(self, bgr_image, color_name):
        """
        Find cube of specific color in BGR image
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
        if area < 500:
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
            
            # Check if we have depth and intrinsics
            if self.latest_depth is None or self.fx is None:
                return
            
            # Detect each colored cube
            detections = {}
            for color_name in ['red', 'yellow', 'blue']:
                detection = self.find_cubes(cv_image, color_name)
                detections[color_name] = detection
            
            # Log and process detections every 30 frames
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Frame {self.frame_count}:')
                annotated_image = cv_image.copy()
                
                for color_name, detection in detections.items():
                    if detection:
                        cx, cy = detection['centroid']
                        x, y, w, h = detection['bbox']
                        area = detection['area']
                        
                        # Get depth at centroid
                        depth_mm = self.latest_depth[cy, cx]
                        depth_m = depth_mm / 1000.0
                        
                        # Convert pixel to 3D camera coordinates
                        point_3d = self.pixel_to_3d(cx, cy, depth_mm)
                        
                        if point_3d is not None:
                            self.get_logger().info(
                                f'  {color_name.upper()}: centroid=({cx}, {cy}), '
                                f'depth={depth_m:.3f}m, '
                                f'3D_camera=({point_3d[0]:.3f}, {point_3d[1]:.3f}, {point_3d[2]:.3f})'
                            )
                            
                            # Transform to world frame and publish
                            self.publish_cube_pose(color_name, point_3d, msg.header.stamp)
                        else:
                            self.get_logger().warn(f'  {color_name.upper()}: Invalid depth or intrinsics')
                        
                        # Draw on image
                        color_bgr = self.color_ranges[color_name]['color_bgr']
                        cv2.rectangle(annotated_image, (x, y), (x + w, y + h), color_bgr, 2)
                        cv2.circle(annotated_image, (cx, cy), 5, color_bgr, -1)
                        cv2.putText(
                            annotated_image,
                            f'{color_name.upper()} {depth_m:.2f}m',
                            (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            color_bgr,
                            2
                        )
                    else:
                        self.get_logger().info(f'  {color_name.upper()}: NOT DETECTED')
                
                # Save annotated image
                filename = os.path.join(
                    self.snapshot_dir,
                    f'detection_{self.frame_count}.jpg'
                )
                cv2.imwrite(filename, annotated_image)
                self.get_logger().info(f'Snapshot saved: {filename}')
            
        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {str(e)}')

    def publish_cube_pose(self, color_name, point_3d, timestamp):
        """
        Convert point from camera frame to world frame and publish
        """
        try:
            # Create point in camera frame
            point_stamped = PointStamped()
            point_stamped.header.frame_id = 'camera_link'
            point_stamped.header.stamp = timestamp
            point_stamped.point.x = point_3d[0]
            point_stamped.point.y = point_3d[1]
            point_stamped.point.z = point_3d[2]
            
            # Transform to world frame
            transform = self.tf_buffer.lookup_transform(
                'world',
                'camera_link',
                timestamp,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            point_world = do_transform_point(point_stamped, transform)
            
            # Create pose message
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = 'world'
            pose_msg.header.stamp = timestamp
            pose_msg.pose.position.x = point_world.point.x
            pose_msg.pose.position.y = point_world.point.y
            pose_msg.pose.position.z = point_world.point.z
            pose_msg.pose.orientation.w = 1.0  # No rotation specified
            
            # Publish to appropriate topic
            if color_name == 'red':
                self.red_publisher.publish(pose_msg)
            elif color_name == 'yellow':
                self.yellow_publisher.publish(pose_msg)
            elif color_name == 'blue':
                self.blue_publisher.publish(pose_msg)
            
            self.get_logger().info(
                f'{color_name.upper()} world pose: ({point_world.point.x:.3f}, '
                f'{point_world.point.y:.3f}, {point_world.point.z:.3f})'
            )
            
        except Exception as e:
            self.get_logger().warn(f'Error transforming {color_name}: {str(e)}')


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
