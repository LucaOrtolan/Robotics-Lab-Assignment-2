#!/usr/bin/env python3
"""
REWORKED Cube Detection & Localization Vision Node for RX200
- FIXED: TF transform timeout (now waits properly)
- FIXED: Depth/color synchronization (uses message filters)
- FIXED: 3D coordinate calculation (robust with fallbacks)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformListener, Buffer
from std_msgs.msg import String
import json
from message_filters import ApproximateTimeSynchronizer, Subscriber
import threading


class CubeDetectionNode(Node):
    """
    Complete cube detection with synchronized depth/color and world coordinate transformation
    """
    
    def __init__(self):
        super().__init__('cube_detection_node')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # TF2 for coordinate transformation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Camera intrinsics
        self.camera_matrix = None
        self.camera_info_received = False
        
        # Detection storage
        self.detected_cubes = []
        self.frame_count = 0
        self.lock = threading.Lock()

        # Cube positions publisher
        self.cube_positions_pub = self.create_publisher(
            String, 
            '/detected_cubes', 
            10
        )

        # Color ranges for RGB cubes (HSV format)
        self.color_ranges = {
            'RED': (np.array([170, 55, 50]), np.array([195, 255, 255])),
            'red':    (np.array([0, 50, 50]), np.array([15, 255, 255])),
            'Blue':   (np.array([90, 80, 40]), np.array([140, 255, 255])),
            'Yellow': (np.array([10, 50, 80]), np.array([50, 255, 255])),
        }

        # ===== SYNCHRONIZED SUBSCRIBERS (KEY FIX) =====
        # Subscribe to color image
        self.image_sub = Subscriber(
            self,
            Image,
            '/camera/camera/color/image_raw'
        )
        
        # Subscribe to depth image
        self.depth_sub = Subscriber(
            self,
            Image,
            '/camera/camera/depth/image_rect_raw'
        )
        
        # Synchronize color and depth with 50ms slop tolerance
        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub],
            queue_size=10,
            slop=0.05  # 50ms tolerance for sync
        )
        self.ts.registerCallback(self.synchronized_callback)
        
        # Camera info subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Logging timer (every 30 frames)
        self.create_timer(1.0, self.log_cube_positions)
        self.create_timer(1.0, self.publish_cube_positions)

        
        self.get_logger().info('✓ Cube Detection Node Initialized (WITH SYNC)')
        self.get_logger().info('Looking for 3 cubes: Red, Blue, Yellow')
        self.get_logger().info('Waiting for camera images...')


    def camera_info_callback(self, msg):
        """Extract camera intrinsics for 3D coordinate calculation"""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.camera_info_received = True
            self.get_logger().info('✓ Camera intrinsics received')


    def synchronized_callback(self, color_msg, depth_msg):
        """
        This callback receives SYNCHRONIZED color and depth frames
        Both arrive at the same time - no more NoneType errors!
        """
        try:
            # Convert ROS Image to OpenCV
            cv_color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
            
            # Detect cubes in color frame
            self.detect_cubes(cv_color)
            
            # Calculate 3D positions using SYNCHRONIZED depth
            if self.camera_info_received:
                self.calculate_3d_positions(cv_color, cv_depth)
            
            # Log results every 30 frames
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                pass  # Logging happens in separate timer
            
            # Display results
            self.display_results(cv_color)
            
        except Exception as e:
            self.get_logger().error(f'Error in sync callback: {str(e)}')


    def detect_cubes(self, image):
        """
        Main cube detection pipeline
        """
        
        # Convert to HSV for better color detection
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        with self.lock:
            self.detected_cubes = []
        
        # Detect each cube color
        for color_name, (lower, upper) in self.color_ranges.items():
            
            # Create mask for this color
            mask = cv2.inRange(hsv_image, lower, upper)
            
            # Clean up noise
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                          cv2.CHAIN_APPROX_SIMPLE)
            
            # Process each potential cube
            for contour in contours:
                cube_info = self.is_cube(contour, color_name)
                if cube_info:
                    with self.lock:
                        self.detected_cubes.append(cube_info)


    def is_cube(self, contour, color):
        """
        Determine if a contour is a cube
        """
        
        # Filter by area
        area = cv2.contourArea(contour)
        if area < 1000 or area > 50000:
            return None
        
        # Approximate to polygon
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # Must have 4 vertices
        if len(approx) != 4:
            return None
        
        # Get bounding rectangle
        x, y, w, h = cv2.boundingRect(contour)
        
        # Check aspect ratio
        aspect_ratio = float(w) / h if h > 0 else 0
        if not (0.7 < aspect_ratio < 1.4):
            return None
        
        # Calculate center point
        M = cv2.moments(contour)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            return None
        
        # This is a valid cube!
        return {
            'color': color,
            'center_x': cx,
            'center_y': cy,
            'area': area,
            'width': w,
            'height': h,
            'aspect_ratio': aspect_ratio,
            'bounding_box': (x, y, w, h),
            'contour': contour,
            'depth_mm': None,
            'world_x': None,
            'world_y': None,
            'world_z': None,
        }


    def calculate_3d_positions(self, color_image, depth_image):
        """
        Convert 2D pixel coordinates to 3D world coordinates
        NOW WITH SYNCHRONIZED DEPTH!
        """
        
        with self.lock:
            cubes = self.detected_cubes.copy()
        
        for cube in cubes:
            try:
                # Get depth at cube center
                cx, cy = cube['center_x'], cube['center_y']
                
                # Safety check
                if (cy < 0 or cy >= depth_image.shape[0] or
                    cx < 0 or cx >= depth_image.shape[1]):
                    continue
                
                depth_mm = depth_image[cy, cx]
                depth_m = depth_mm / 1000.0  # Convert to meters
                
                # Skip invalid/out-of-range depths
                if depth_m == 0 or depth_m > 3.0:  # Max 3 meters
                    continue
                
                cube['depth_mm'] = depth_mm
                
                # === CALCULATE 3D POINT IN CAMERA FRAME ===
                fx = self.camera_matrix[0, 0]
                fy = self.camera_matrix[1, 1]
                cx_intrinsic = self.camera_matrix[0, 2]
                cy_intrinsic = self.camera_matrix[1, 2]
                
                x_cam = (cx - cx_intrinsic) * depth_m / fx
                y_cam = (cy - cy_intrinsic) * depth_m / fy
                z_cam = depth_m
                
                # Create point in camera frame
                point_camera = PointStamped()
                point_camera.header.frame_id = 'camera_link'
                point_camera.header.stamp = self.get_clock().now().to_msg()
                point_camera.point.x = x_cam
                point_camera.point.y = y_cam
                point_camera.point.z = z_cam
                
                # === WAIT FOR TRANSFORM (FIX #1: LONGER TIMEOUT) ===
                try:
                    # Try to transform with generous timeout
                    point_base = self.tf_buffer.transform(
                        point_camera, 
                        'rx200/base_link',
                        timeout=rclpy.duration.Duration(seconds=1.0)  # 1 second timeout
                    )
                    
                    cube['world_x'] = point_base.point.x
                    cube['world_y'] = point_base.point.y
                    cube['world_z'] = point_base.point.z
                    
                except Exception as tf_error:
                    # If transform fails, try with latest available time
                    try:
                        self.get_logger().debug(f'Transform failed, retrying: {tf_error}')
                        
                        # Try to lookup without timeout (instant check)
                        transform = self.tf_buffer.lookup_transform(
                            'rx200/base_link',
                            'camera_link',
                            rclpy.time.Time()  # Latest available
                        )
                        
                        # Manual transform
                        from tf_transformations import quaternion_matrix
                        
                        q = [transform.transform.rotation.x,
                             transform.transform.rotation.y,
                             transform.transform.rotation.z,
                             transform.transform.rotation.w]
                        
                        t = [transform.transform.translation.x,
                             transform.transform.translation.y,
                             transform.transform.translation.z]
                        
                        T = quaternion_matrix(q)
                        T[0, 3] = t[0]
                        T[1, 3] = t[1]
                        T[2, 3] = t[2]
                        
                        cam_point = np.array([x_cam, y_cam, z_cam, 1.0])
                        base_point = T @ cam_point
                        
                        cube['world_x'] = float(base_point[0])
                        cube['world_y'] = float(base_point[1])
                        cube['world_z'] = float(base_point[2])
                        
                    except Exception as retry_error:
                        self.get_logger().warn(
                            f'{cube["color"]} cube: TF transform unavailable - {str(retry_error)}'
                        )
                    
            except Exception as e:
                self.get_logger().warn(f'3D calculation error for {cube["color"]}: {e}')
        
        # Update cubes with calculated positions
        with self.lock:
            self.detected_cubes = cubes


    def log_cube_positions(self):
        """
        Log detected cube positions every second
        """
        
        with self.lock:
            cubes = self.detected_cubes.copy()
        
        if not cubes:
            self.get_logger().info('❌ No cubes detected')
            return
        
        self.get_logger().info('═' * 100)
        self.get_logger().info(f'📦 DETECTED {len(cubes)} CUBE(S):')
        self.get_logger().info('═' * 100)
        
        # Sort by color for consistent output
        sorted_cubes = sorted(cubes, key=lambda x: x['color'])
        
        for i, cube in enumerate(sorted_cubes, 1):
            # Pixel coordinates
            px_info = f"Px: ({cube['center_x']:4}, {cube['center_y']:4})"
            
            # World coordinates
            if cube['world_x'] is not None:
                world_info = (f"World: X={cube['world_x']:7.3f}, "
                             f"Y={cube['world_y']:7.3f}, Z={cube['world_z']:7.3f} m")
            else:
                world_info = "World: <NO TRANSFORM>"
            
            # Depth
            depth_info = f"D: {cube['depth_mm']/1000:.3f}m" if cube['depth_mm'] else "D: N/A"
            
            self.get_logger().info(
                f"[{i}] {cube['color']:6} | {px_info} | {world_info} | {depth_info}"
            )
        
        self.get_logger().info('═' * 100)
        
        # Summary
        if len(cubes) == 3:
            self.get_logger().info('ALL 3 CUBES DETECTED!')
            world_cubes = [c for c in cubes if c['world_x'] is not None]
            if len(world_cubes) == 3:
                self.get_logger().info('ALL 3 CUBES LOCALIZED IN 3D WORLD COORDINATES!')
            else:
                self.get_logger().warn(f'Only {len(world_cubes)}/3 cubes have 3D coordinates (check TF tree)')
        else:
            self.get_logger().warn(f'Only {len(cubes)}/3 cubes detected')

    def publish_cube_positions(self):
        """Publish detected cubes as JSON string for manipulator"""
        with self.lock:
            cubes = self.detected_cubes.copy()
        
        if not cubes:
            return
        
        # Create dict with color -> [x,y,z] format expected by manipulator
        cubes_dict = {}
        for cube in cubes:
            if cube['world_x'] is not None:
                color_key = cube['color'].lower()  # 'red', 'blue', 'yellow'
                cubes_dict[color_key] = [
                    float(cube['world_x']),
                    float(cube['world_y']), 
                    float(cube['world_z']) + 0.03  # Hover offset for pick
                ]

        msg = String()
        msg.data = json.dumps(cubes_dict)
        self.cube_positions_pub.publish(msg)
        self.get_logger().info(f'Published {len(cubes_dict)} cubes: {msg.data}')



    def display_results(self, image):
        """Display image with detections"""
        
        try:
            with self.lock:
                cubes = self.detected_cubes.copy()
            
            display_image = image.copy()
            
            # Color mapping for display (BGR format)
            draw_colors = {
                'Red': (0, 0, 255),
                'Blue': (255, 0, 0),
                'Yellow': (0, 255, 255),
            }
            
            for cube in cubes:
                color = draw_colors.get(cube['color'], (255, 255, 255))
                
                # Draw bounding box
                x, y, w, h = cube['bounding_box']
                cv2.rectangle(display_image, (x, y), (x+w, y+h), color, 2)
                
                # Draw center point
                cv2.circle(display_image, 
                          (cube['center_x'], cube['center_y']),
                          8, color, -1)
                
                # Add crosshair
                cv2.line(display_image, 
                        (cube['center_x']-10, cube['center_y']),
                        (cube['center_x']+10, cube['center_y']),
                        color, 2)
                cv2.line(display_image,
                        (cube['center_x'], cube['center_y']-10),
                        (cube['center_x'], cube['center_y']+10),
                        color, 2)
                
                # Add label
                label = f"{cube['color']}"
                cv2.putText(display_image, label,
                           (cube['center_x'] - 40, cube['center_y'] - 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                
                # Add world coordinates if available
                if cube['world_x'] is not None:
                    world_label = f"({cube['world_x']:.2f}, {cube['world_y']:.2f}, {cube['world_z']:.2f})"
                    cv2.putText(display_image, world_label,
                               (cube['center_x'] - 60, cube['center_y'] + 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            # Add header
            cv2.putText(display_image, f'Cubes: {len(cubes)}/3',
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.putText(display_image, 'Press Q to exit',
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Show image
            cv2.imshow('Cube Detection (SYNCHRONIZED)', display_image)
            
            # Check for quit
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Shutting down...')
                raise KeyboardInterrupt
                
        except Exception as e:
            pass  # Silent fail if no display


    def get_cube_positions(self):
        """
        Return current detected cube positions
        Safe access with lock
        """
        with self.lock:
            return self.detected_cubes.copy()


def main(args=None):
    rclpy.init(args=args)
    
    node = CubeDetectionNode()
    
    try:
        #node.get_logger().info(node.get_cube_positions())
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Cube Detection Node shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
