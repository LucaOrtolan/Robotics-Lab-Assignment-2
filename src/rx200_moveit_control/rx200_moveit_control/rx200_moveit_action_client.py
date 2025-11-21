import math
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints,PositionConstraint,OrientationConstraint,MotionPlanRequest,JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler


class MoveItEEClient(Node):
    def __init__(self, control_group="arm"):
        super().__init__('rx200_moveit_control')

        self.motion_done = True
        self.gr_motion_done = True
        self._client = ActionClient(self, MoveGroup, '/move_action')

        while not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning("Waiting for MoveGroup action server...")

        self.group_name = "interbotix_gripper" if control_group == "gripper" else "interbotix_arm"
        self.ee_link = "rx200/ee_gripper_link"
        self.base_link = "rx200/base_link"
        self.gripper_joint = "left_finger"

    def send_gr_pose(self, open=True):
        self.gr_motion_done = False
        
        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1

        jc = JointConstraint()
        jc.joint_name = self.gripper_joint
        jc.position = 0.05 if open else 0.005
        jc.tolerance_above = jc.tolerance_below = 0.01
        jc.weight = 1.0

        goal_constraints = Constraints(joint_constraints=[jc])
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def send_pose(self, x, y, z, roll=0.0, pitch=1.4):
        self.motion_done = False
        self.get_logger().info(f"Moving to: ({x},{y},{z})")

        pose = PoseStamped()
        pose.header.frame_id = self.base_link
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z

        yaw = math.atan2(y, x)
        q = quaternion_from_euler(roll, pitch, yaw)

        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 3

        pc = PositionConstraint()
        pc.header.frame_id = self.base_link
        pc.link_name = self.ee_link
        sp = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.05])
        pc.constraint_region.primitives = [sp]
        pc.constraint_region.primitive_poses = [pose.pose]

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_link
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0

        goal_constraints = Constraints(
            position_constraints=[pc],
            orientation_constraints=[oc]
        )
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self.motion_done = self.gr_motion_done = True
            return

        self.get_logger().info("Goal accepted")
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        code = getattr(result.error_code, 'val', -1)
        self.get_logger().info(f"Result: error_code={code}")
        self.motion_done = self.gr_motion_done = True

    def _feedback_cb(self, feedback_msg):
        state = getattr(feedback_msg.feedback, "state", "<unknown>")
        self.get_logger().debug(f"[Feedback] {state}")

def main():
    rclpy.init()
    node_ar = MoveItEEClient(control_group="arm")
    gr_node = MoveItEEClient(control_group="gripper")
    
    # Upright position coordinates
    # Two separate motions: first move upwards, then change the EE's pitch 
    seq = [
        (0.1, 0.0, 0.3, True, None), # x, y, z, open gripper, pitch
    ]
    
    gr_pos = True

    for x, y, z, gr, pitch in seq:

        while not node_ar.motion_done or not gr_node.gr_motion_done:
            rclpy.spin_once(node_ar, timeout_sec=0.1)
            rclpy.spin_once(gr_node, timeout_sec=0.1)

        if pitch is None:
            pitch = math.atan2(z, math.sqrt(x ** 2 + y ** 2))
    
        node_ar.send_pose(x, y, z, pitch=pitch)
        while not node_ar.motion_done:
            rclpy.spin_once(node_ar, timeout_sec=0.1)

        gr_node.send_gr_pose(open=gr)
        if not gr_pos:
            gr_node.get_logger().info(f"Gripper: already closed")
        else:
            gr_node.get_logger().info(f"Gripper: {'Open' if open else 'Close'}")

        while not gr_node.gr_motion_done:
            rclpy.spin_once(gr_node, timeout_sec=0.1)
        
        gr_pos = gr

    rclpy.shutdown()



if __name__ == "__main__":
    main()


# Coordinates for initial pose snap
# x=0.15, y=0, z=.3, pitch=.6
# Once in this position, we snap the pose
# Then, we'll need to cluster the cubes' colors and store them 
# Lastly, we need to implement the pick and place motion based on the order specified by the user

# We can use 3 nodes: one for the arm, one for the camera and one for the clustering (maybe not necessary?)