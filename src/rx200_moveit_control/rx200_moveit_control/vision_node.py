#!/usr/bin/env python3

"""
FINAL FIXED Cube Detection & Localization Vision Node for RX200

- Correct 3D transformation for wrist-mounted RealSense
- Unproject in camera_depth_optical_frame
- Use depth_msg timestamp for TF
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformListener, Buffer
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf_transformations import quaternion_matrix
import threading
from std_msgs.msg import String
import json



class CubeDetectionNode(Node):
    """Cube detection with correct 3D transform from camera to base"""

    def __init__(self):
        super().__init__('cube_detection_node')
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_matrix = None
        self.camera_info_received = False
        self.detected_cubes = []
        self.frame_count = 0
        self.lock = threading.Lock()

        # Color ranges
        self.color_ranges = {
            'RED':   (np.array([0, 40, 40]),   np.array([10, 255, 255])),
            'red':  (np.array([160, 40, 40]), np.array([200, 255, 255])),
            'Blue':  (np.array([90, 80, 40]),  np.array([140, 255, 255])),
            'Yellow':(np.array([10, 50, 80]),  np.array([50, 255, 255])),
        }

        # Cube positions publisher
        self.cube_positions_pub = self.create_publisher(
            String, 
            '/detected_cubes', 
            10
        )

        # Subscribers (synchronized RGB + depth)
        self.image_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')
        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub],
            queue_size=10,
            slop=0.05
        )
        self.ts.registerCallback(self.synchronized_callback)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        self.create_timer(1.0, self.log_cube_positions)
        self.create_timer(1.0, self.publish_cube_positions)

        self.get_logger().info('✓ Cube Detection Node Initialized (FINAL FIX)')
        self.get_logger().info('Waiting for camera images...')

    def camera_info_callback(self, msg: CameraInfo):
        """Extract camera intrinsics"""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.camera_info_received = True
            self.get_logger().info(
                f'✓ Camera intrinsics: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, '
                f'cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}'
            )

    def synchronized_callback(self, color_msg: Image, depth_msg: Image):
        """Process synchronized color + depth frames"""
        try:
            cv_color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')

            self.detect_cubes(cv_color)
            if self.camera_info_received:
                self.calculate_3d_positions(cv_depth, depth_msg)

            self.frame_count += 1
            self.display_results(cv_color)
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

    def detect_cubes(self, image):
        """Detect colored cubes in RGB image"""
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        with self.lock:
            self.detected_cubes = []

        for color_name, (lower, upper) in self.color_ranges.items():
            mask = cv2.inRange(hsv_image, lower, upper)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                cube_info = self.is_cube(contour, color_name)
                if cube_info:
                    with self.lock:
                        self.detected_cubes.append(cube_info)

    def is_cube(self, contour, color):
        """Check if contour is a plausible cube"""
        area = cv2.contourArea(contour)
        if area < 1000 or area > 50000:
            return None

        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) != 4:
            return None

        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = float(w) / h if h > 0 else 0.0
        if not (0.7 < aspect_ratio < 1.4):
            return None

        M = cv2.moments(contour)
        if M['m00'] <= 0:
            return None

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        return {
            'color': color,
            'center_x': cx,
            'center_y': cy,
            'area': area,
            'width': w,
            'height': h,
            'aspect_ratio': aspect_ratio,
            'bounding_box': (x, y, w, h),
            'depth_mm': None,
            'world_x': None,
            'world_y': None,
            'world_z': None,
        }

    def calculate_3d_positions(self, depth_image, depth_msg: Image):
        """
        Correct 3D transformation using:
        - Unprojection in camera_depth_optical_frame
        - TF to rx200/base_link with depth timestamp
        """
        with self.lock:
            cubes = self.detected_cubes.copy()

        for cube in cubes:
            try:
                u, v = cube['center_x'], cube['center_y']
                if (v < 0 or v >= depth_image.shape[0] or
                    u < 0 or u >= depth_image.shape[1]):
                    continue

                depth_mm = depth_image[v, u]
                depth_m = depth_mm / 1000.0

                if depth_m == 0 or depth_m > 3.0:
                    continue

                cube['depth_mm'] = depth_mm

                # Step 1: unproject pixel (u, v, depth) into camera optical frame
                fx = self.camera_matrix[0, 0]
                fy = self.camera_matrix[1, 1]
                cx_intrinsic = self.camera_matrix[0, 2]
                cy_intrinsic = self.camera_matrix[1, 2]

                x_cam = (u - cx_intrinsic) * depth_m / fx
                y_cam = (v - cy_intrinsic) * depth_m / fy
                z_cam = depth_m

                # Step 2: transform point to base_link using TF at depth timestamp
                try:
                    point_camera = PointStamped()
                    point_camera.header.frame_id = 'camera_depth_optical_frame'
                    point_camera.header.stamp = depth_msg.header.stamp
                    point_camera.point.x = x_cam
                    point_camera.point.y = y_cam
                    point_camera.point.z = z_cam

                    point_base = self.tf_buffer.transform(
                        point_camera,
                        'rx200/base_link',
                        timeout=rclpy.duration.Duration(seconds=0.5)
                    )

                    cube['world_x'] = point_base.point.x
                    cube['world_y'] = point_base.point.y
                    cube['world_z'] = point_base.point.z

                except Exception as tf_error:
                    # Fallback: latest available transform
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            'rx200/base_link',
                            'camera_depth_optical_frame',
                            rclpy.time.Time()
                        )

                        q = [
                            transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                            transform.transform.rotation.w
                        ]
                        T = quaternion_matrix(q)
                        T[0, 3] = transform.transform.translation.x
                        T[1, 3] = transform.transform.translation.y
                        T[2, 3] = transform.transform.translation.z

                        point_cam_h = np.array([x_cam, y_cam, z_cam, 1.0])
                        point_base_h = T @ point_cam_h

                        cube['world_x'] = point_base_h[0]
                        cube['world_y'] = point_base_h[1]
                        cube['world_z'] = point_base_h[2]
                    except Exception:
                        pass
            except Exception:
                pass

        with self.lock:
            self.detected_cubes = cubes

    def log_cube_positions(self):
        """Log cube positions periodically"""
        with self.lock:
            cubes = self.detected_cubes.copy()

        if not cubes:
            return

        self.get_logger().info(f'Frame {self.frame_count}: {len(cubes)} cubes detected')
        for cube in sorted(cubes, key=lambda x: x['color']):
            if cube['world_x'] is not None:
                self.get_logger().info(
                    f"  {cube['color']}: "
                    f"({cube['world_x']:.3f}, {cube['world_y']:.3f}, {cube['world_z']:.3f})"
                )

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
        """Simple live feed with cube overlays"""
        try:
            with self.lock:
                cubes = self.detected_cubes.copy()

            display_image = image.copy()
            draw_colors = {
                'RED': (0, 0, 255),
                'red': (0, 0, 255),
                'Blue': (255, 0, 0),
                'Yellow': (0, 255, 255),
            }

            for cube in cubes:
                color = draw_colors.get(cube['color'], (255, 255, 255))
                x, y, w, h = cube['bounding_box']
                cv2.rectangle(display_image, (x, y), (x + w, y + h), color, 2)
                cv2.circle(display_image,
                           (cube['center_x'], cube['center_y']), 6, color, -1)

                if cube['world_x'] is not None:
                    text = f"{cube['world_x']:.3f}, {cube['world_y']:.3f}, {cube['world_z']:.3f}"
                    cv2.putText(display_image, text,
                                (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            cv2.putText(display_image, f'Cubes: {len(cubes)}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0), 2)

            cv2.imshow('Cube Detection', display_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                raise KeyboardInterrupt
        except Exception:
            pass

    def get_cube_positions(self):
        with self.lock:
            return self.detected_cubes.copy()


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
