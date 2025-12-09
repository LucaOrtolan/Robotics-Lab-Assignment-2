
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time
import math
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion



class CameraSceneUpdater(Node):
    def __init__(self):
        super().__init__('camera_scene_updater')
        
        # Publisher to MoveIt's planning scene
        self.scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )
        
        # Give MoveIt time to fully initialize
        time.sleep(2.0)
        
        # Add camera as collision object
        self.add_camera_collision()
        
        # Attach it to the wrist so it moves with the arm
        self.attach_camera_to_wrist()
        
        self.get_logger().info('✓ Camera added to MoveIt planning scene!')

    def add_camera_collision(self):
        
        collision_object = CollisionObject()
        # Frame must be namespaced to match URDF convention
        collision_object.header.frame_id = 'rx200/wrist_link'
        collision_object.id = 'realsense_d415'
        collision_object.operation = CollisionObject.ADD
        
        # Define camera as a box (RealSense D415 approximate dimensions)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        # dimensions = [length, width, height] in meters
        box.dimensions = [0.025, 0.1, 0.025]  # 10cm × 2.3cm × 2cm
        
        # Position the box relative to camera_mount frame
        pose = Pose()
        pose.position.x = 0.105
        pose.position.y = 0.00
        pose.position.z = 0.11  # Must match camera_link_tf offset!
        roll = 0.0
        pitch = math.radians(45)  # Camera faces downward
        yaw = 0.0
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(pose)
        
        # Create and publish planning scene
        planning_scene = PlanningScene()
        planning_scene.is_diff = True  # Only update what changed
        planning_scene.world.collision_objects.append(collision_object)
        
        self.scene_pub.publish(planning_scene)
        self.get_logger().info('Published collision object: realsense_d415')

    def attach_camera_to_wrist(self):
        """Attach camera to wrist so it moves during arm motion"""
        
        # Create attached collision object
        attached_object = AttachedCollisionObject()
        attached_object.link_name = 'rx200/wrist_link'  # Attach to wrist
        attached_object.object.id = 'realsense_d415'
        attached_object.object.operation = CollisionObject.ADD
        
        # Links that are allowed to touch the camera (won't create collisions)
        attached_object.touch_links = [
            'rx200/wrist_link',
            'rx200/gripper_link',
            'rx200/camera_mount',  # This frame doesn't exist in URDF but is safe
        ]
        
        # Create and publish planning scene
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        
        self.scene_pub.publish(planning_scene)
        self.get_logger().info('Attached camera to rx200/wrist_link')

def main(args=None):
    rclpy.init(args=args)
    node = CameraSceneUpdater()
    # Just run once and exit (publisher already sent message)
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
