
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
        
        self.scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )
        
        self.add_camera_collision()
        self.attach_camera_to_wrist()
        self.get_logger().info('✓ Camera added to MoveIt planning scene!')

    def add_camera_collision(self):
        
        collision_object = CollisionObject()
        collision_object.header.frame_id = 'rx200/wrist_link'
        collision_object.id = 'rs_cam'
        collision_object.operation = CollisionObject.ADD
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.025, 0.1, 0.025]

        pose = Pose()
        pose.position.x = 0.105
        pose.position.y = 0.00
        pose.position.z = 0.11
        roll = 0.0
        pitch = math.radians(45)
        yaw = 0.0
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(pose)
        
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(collision_object)
        
        self.scene_pub.publish(planning_scene)
        self.get_logger().info('Published collision object: rs_cam')

    def attach_camera_to_wrist(self):
        
        attached_object = AttachedCollisionObject()
        attached_object.link_name = 'rx200/wrist_link'
        attached_object.object.id = 'rs_cam'
        attached_object.object.operation = CollisionObject.ADD
        
        attached_object.touch_links = ['rx200/wrist_link',]
        
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        
        self.scene_pub.publish(planning_scene)
        self.get_logger().info('Attached camera to rx200/wrist_link')

def main(args=None):
    rclpy.init(args=args)
    node = CameraSceneUpdater()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
