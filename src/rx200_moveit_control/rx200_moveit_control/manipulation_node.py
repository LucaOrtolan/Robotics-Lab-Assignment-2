#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, MotionPlanRequest, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler


class MoveItEEClient(Node):
    def __init__(self, control_group="arm"):
        super().__init__('rx200_moveit_control_' + control_group)

        self.motion_done = True
        self._client = ActionClient(self, MoveGroup, '/move_action')

        while not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for MoveGroup action server...")

        self.group_name = "interbotix_arm"
        self.ee_link = "rx200/ee_gripper_link"
        self.base_link = "rx200/base_link"

    def send_pose(self, x, y, z, roll=0.0, pitch=0.0):
        self.motion_done = False
        self.get_logger().info(f"Scan pose: ({x:.3f}, {y:.3f}, {z:.3f})")

        pose = PoseStamped()
        pose.header.frame_id = self.base_link
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

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

        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.05]

        pc.constraint_region.primitives = [sp]
        pc.constraint_region.primitive_poses = [pose.pose]

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_link
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0

        goal_constraints = Constraints(
            position_constraints=[pc],
            orientation_constraints=[oc]
        )
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        fut = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        fut.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error("Scan pose goal rejected")
            self.motion_done = True
            return

        gh.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, fut):
        res = fut.result().result
        code = getattr(res.error_code, 'val', -1)
        self.get_logger().info(f"Scan pose result code: {code}")
        self.motion_done = True

    def _feedback_cb(self, fb):
        state = getattr(fb.feedback, "state", "")
        self.get_logger().debug(f"[Scan] {state}")


class ManipulationNode(Node):
    def __init__(self):
        super().__init__('manipulation_node')

        self.ee_client = MoveItEEClient(control_group="arm")

        # Hardcoded scan pose above workspace (tune as needed)
        x = 0.10
        y = 0.0
        z = 0.35
        pitch = math.radians(30.0)

        self.ee_client.send_pose(x, y, z, pitch=pitch)


def main():
    rclpy.init()
    node = ManipulationNode()

    time.sleep(2.0)  # wait for everything to initialize

    while rclpy.ok() and not node.ee_client.motion_done:
        rclpy.spin_once(node, timeout_sec=0.1)

    rclpy.spin(node)  # idle after reaching scan pose


if __name__ == '__main__':
    main()
