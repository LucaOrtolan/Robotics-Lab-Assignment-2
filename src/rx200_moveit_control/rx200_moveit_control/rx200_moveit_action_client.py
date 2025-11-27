#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, MotionPlanRequest, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler


class MoveItEEClient(Node):
    def __init__(self):
        super().__init__('rx200_moveit_control')

        self.motion_done = True
        self.gr_motion_done = True
        self._client = ActionClient(self, MoveGroup, '/move_action')

        while not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning("Waiting for MoveGroup action server...")

        # MoveIt group names used by this robot
        self.arm_group = "interbotix_arm"
        self.gripper_group = "interbotix_gripper"

        # Defaults
        self.group_name = self.arm_group
        self.ee_link = "rx200/ee_gripper_link"
        self.base_link = "rx200/base_link"
        self.gripper_joint = "left_finger"  # change if your gripper uses different joint names

    #
    # Gripper move: open=True -> open, open=False -> closed
    #
    def send_gr_pose(self, open=True):
        self.gr_motion_done = False

        req = MotionPlanRequest()
        req.group_name = self.gripper_group
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 2

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

    #
    # Move the end-effector to a desired pose (position+orientation)
    #
    def send_pose(self, x, y, z, roll=0.0, pitch=1.4):
        self.motion_done = False
        self.get_logger().info(f"Moving to: ({x:.3f}, {y:.3f}, {z:.3f})")

        pose = PoseStamped()
        pose.header.frame_id = self.base_link
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z

        yaw = math.atan2(y, x)
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        req = MotionPlanRequest()
        req.group_name = self.arm_group
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

        goal_constraints = Constraints(position_constraints=[pc], orientation_constraints=[oc])
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    #
    # Callbacks for goal handling
    #
    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            # ensure waiting loops don't block forever
            self.motion_done = self.gr_motion_done = True
            return

        self.get_logger().info("Goal accepted")
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        code = getattr(result.error_code, 'val', -1)
        self.get_logger().info(f"Result: error_code={code}")
        # set both flags True because single action server is used for both kinds of goals,
        # use whichever flag is currently False to keep consistent
        self.motion_done = True
        self.gr_motion_done = True

    def _feedback_cb(self, feedback_msg):
        state = getattr(feedback_msg.feedback, "state", "<unknown>")
        self.get_logger().debug(f"[Feedback] {state}")

    #
    # High-level pick & place loop
    #
    def pick_place_cubes(self, cubes_data: dict, color_order: list, place_position: list):
        # reorder according to color_order (preserves only those colors asked)
        cubes_data = {key: cubes_data[key] for key in color_order}
        x_place, y_place, z_place = place_position[0], place_position[1], place_position[2]

        for key, value in cubes_data.items():
            self.get_logger().info(f"Picking up {key} cube...")

            x_pick, y_pick, z_hover = value[0], value[1], value[2]

            # Approach above the cube (hover)
            pitch = math.atan2(z_hover, math.sqrt(x_pick ** 2 + y_pick ** 2))
            self.send_pose(x_pick, y_pick, z_hover, pitch=pitch)
            # wait until motion completes
            while not self.motion_done:
                rclpy.spin_once(self, timeout_sec=0.1)

            # Lower slightly to pick (you might want a separate lower altitude)
            z_low = max(0.0, z_hover - 0.05)  # small downward move; tune as needed
            self.motion_done = False
            self.send_pose(x_pick, y_pick, z_low, pitch=pitch)
            while not self.motion_done:
                rclpy.spin_once(self, timeout_sec=0.1)

            # Close gripper
            self.send_gr_pose(open=False)
            while not self.gr_motion_done:
                rclpy.spin_once(self, timeout_sec=0.1)

            # Lift back to hover height
            self.send_pose(x_pick, y_pick, z_hover, pitch=pitch)
            while not self.motion_done:
                rclpy.spin_once(self, timeout_sec=0.1)

            # Move to place position (with current z_place)
            self.get_logger().info(f"Placing down {key} cube...")
            pitch_place = math.atan2(z_place, math.sqrt(x_place ** 2 + y_place ** 2))
            self.send_pose(x_place, y_place, z_place + 0.02, pitch=pitch_place)  # approach a little above
            while not self.motion_done:
                rclpy.spin_once(self, timeout_sec=0.1)

            # Lower to release
            self.send_pose(x_place, y_place, z_place, pitch=pitch_place)
            while not self.motion_done:
                rclpy.spin_once(self, timeout_sec=0.1)

            # Open gripper to release
            self.send_gr_pose(open=True)
            while not self.gr_motion_done:
                rclpy.spin_once(self, timeout_sec=0.1)

            # Lift away after release
            self.send_pose(x_place, y_place, z_place + 0.05, pitch=pitch_place)
            while not self.motion_done:
                rclpy.spin_once(self, timeout_sec=0.1)

            # Increase the stacking height for the next cube
            z_place += 0.055  # tune to cube thickness
            self.get_logger().info(f"Stack height now {z_place:.3f}")


def main():
    rclpy.init()
    node = MoveItEEClient()

    # Upright position (open gripper and set pitch)
    seq = [
        (0.3, 0.0, 0.45, True, 0.3),  # x, y, z, open gripper (bool), pitch
    ]
    motion = seq[0]
    x, y, z, gr_open, pitch = motion

    cubes_data = {
        "red": [0.25, 0.15, 0.075],
        "blue": [0.25, -0.15, 0.075],
        "yellow": [0.25, -0.2, 0.075]
    }
    color_order = ["red", "blue", "yellow"]
    place_position = [0.3, 0.0, 0.055]

    # Move to upright
    node.send_pose(x, y, z, pitch=pitch)
    while not node.motion_done:
        rclpy.spin_once(node, timeout_sec=0.1)

    # Ensure gripper open
    node.send_gr_pose(open=gr_open)
    while not node.gr_motion_done:
        rclpy.spin_once(node, timeout_sec=0.1)

    # Run pick & place loop
    node.pick_place_cubes(cubes_data, color_order, place_position)

    # After finished
    node.get_logger().info("Pick and place finished.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
