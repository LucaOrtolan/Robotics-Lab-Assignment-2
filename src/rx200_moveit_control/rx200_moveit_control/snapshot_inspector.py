#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np

class SnapshotInspector(Node):
    def __init__(self):
        super().__init__('snapshot_inspector')
        
        # Bridge to convert ROS images to OpenCV
        self.bridge = CvBridge()
        
        # Storage for latest frames
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_info = None
        
        # Subscribe to camera topics
        self.create_subscription(
            Image, 
            '/camera/camera/color/image_raw', 
            self.rgb_callback, 
            10
        )
        
        self.create_subscription(
            Image, 
            '/camera/camera/aligned_depth_to_color/image_raw', 
            self.depth_callback, 
            10
        )
        
        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.info_callback,
            10
        )
        
        # Service to trigger snapshot
        self.create_service(Trigger, 'snap_now', self.handle_snap)
        
        self.get_logger().info("Snapshot Inspector Ready!")
        self.get_logger().info("Move robot to scan position, then call:")
        self.get_logger().info("  ros2 service call /snap_now std_srvs/srv/Trigger")

    # =========================================================================
    #                    PASSIVE BUFFERING (Background)
    # =========================================================================
    
    def rgb_callback(self, msg):
        """Continuously store the latest RGB image"""
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Don't log here - would spam at 30 FPS

    def depth_callback(self, msg):
        """Continuously store the latest depth image"""
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def info_callback(self, msg):
        """Store camera intrinsics (only need once)"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info("Camera info received:")
            self.get_logger().info(f"  Resolution: {msg.width} x {msg.height}")
            self.get_logger().info(f"  Focal Length (fx, fy): ({msg.k[0]:.2f}, {msg.k[4]:.2f})")
            self.get_logger().info(f"  Principal Point (cx, cy): ({msg.k[2]:.2f}, {msg.k[5]:.2f})")

    # =========================================================================
    #                    THE SNAPSHOT TRIGGER
    # =========================================================================
    
    def handle_snap(self, request, response):
        """
        Called when you manually trigger the service.
        This is THE MOMENT the snapshot is taken.
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("📸 SNAPSHOT TRIGGERED!")
        self.get_logger().info("=" * 60)
        
        # Check if we have data
        if self.latest_rgb is None:
            response.success = False
            response.message = "No RGB image received yet"
            return response
        
        if self.latest_depth is None:
            response.success = False
            response.message = "No depth image received yet"
            return response
        
        if self.camera_info is None:
            response.success = False
            response.message = "No camera info received yet"
            return response
        
        # ===== THIS IS THE SNAP =====
        # Make frozen copies of the current frame
        snapshot_rgb = self.latest_rgb.copy()
        snapshot_depth = self.latest_depth.copy()
        # ============================
        
        # Now inspect what you have
        self.inspect_snapshot(snapshot_rgb, snapshot_depth)
        
        response.success = True
        response.message = "Snapshot captured and saved"
        return response

    # =========================================================================
    #                    INSPECTION & VISUALIZATION
    # =========================================================================
    
    def inspect_snapshot(self, rgb_img, depth_img):
        """
        Print out everything about the captured snapshot
        """
        self.get_logger().info("\n" + "─" * 60)
        self.get_logger().info("SNAPSHOT INSPECTION REPORT")
        self.get_logger().info("─" * 60)
        
        # === RGB Image Info ===
        self.get_logger().info("\n[RGB Image]")
        self.get_logger().info(f"  Shape: {rgb_img.shape}")  # (height, width, channels)
        self.get_logger().info(f"  Data type: {rgb_img.dtype}")  # uint8
        self.get_logger().info(f"  Value range: {rgb_img.min()} to {rgb_img.max()}")
        
        # === Depth Image Info ===
        self.get_logger().info("\n[Depth Image]")
        self.get_logger().info(f"  Shape: {depth_img.shape}")  # (height, width)
        self.get_logger().info(f"  Data type: {depth_img.dtype}")  # uint16
        self.get_logger().info(f"  Value range: {depth_img.min()} to {depth_img.max()} mm")
        self.get_logger().info(f"  Valid depth pixels: {np.count_nonzero(depth_img)} / {depth_img.size}")
        
        # === Sample some pixel values ===
        height, width = rgb_img.shape[:2]
        center_u = width // 2
        center_v = height // 2
        
        self.get_logger().info("\n[Sample Pixel at Image Center]")
        self.get_logger().info(f"  Pixel coords (u, v): ({center_u}, {center_v})")
        
        rgb_value = rgb_img[center_v, center_u]
        self.get_logger().info(f"  RGB value: B={rgb_value[0]}, G={rgb_value[1]}, R={rgb_value[2]}")
        
        depth_value_mm = depth_img[center_v, center_u]
        depth_value_m = depth_value_mm / 1000.0
        self.get_logger().info(f"  Depth value: {depth_value_mm} mm = {depth_value_m:.3f} m")
        
        # === Camera Intrinsics ===
        self.get_logger().info("\n[Camera Intrinsics Available]")
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        self.get_logger().info(f"  fx = {fx:.2f}")
        self.get_logger().info(f"  fy = {fy:.2f}")
        self.get_logger().info(f"  cx = {cx:.2f}")
        self.get_logger().info(f"  cy = {cy:.2f}")
        
        # === Example: What 3D point is at image center? ===
        if depth_value_mm > 0:
            self.get_logger().info("\n[3D Coordinates at Center Pixel - Camera Frame]")
            x_cam = (center_u - cx) * depth_value_m / fx
            y_cam = (center_v - cy) * depth_value_m / fy
            z_cam = depth_value_m
            self.get_logger().info(f"  X_camera: {x_cam:.4f} m")
            self.get_logger().info(f"  Y_camera: {y_cam:.4f} m")
            self.get_logger().info(f"  Z_camera: {z_cam:.4f} m")
            self.get_logger().info("  (Note: These are relative to camera, not robot base)")
        
        # === Save images to disk ===
        rgb_path = '/tmp/snapshot_rgb.png'
        depth_path = '/tmp/snapshot_depth.png'
        
        cv2.imwrite(rgb_path, rgb_img)
        self.get_logger().info(f"\n✓ RGB image saved to: {rgb_path}")
        
        # Normalize depth for visualization (0-255 range)
        depth_normalized = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_colorized = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
        cv2.imwrite(depth_path, depth_colorized)
        self.get_logger().info(f"✓ Depth image saved to: {depth_path}")
        
        self.get_logger().info("\n" + "─" * 60)
        self.get_logger().info("END INSPECTION REPORT")
        self.get_logger().info("─" * 60 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = SnapshotInspector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
