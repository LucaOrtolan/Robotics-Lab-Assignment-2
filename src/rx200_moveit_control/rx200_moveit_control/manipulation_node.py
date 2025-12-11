#!/usr/bin/env python3
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, MotionPlanRequest, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler
import time
from std_msgs.msg import String
import json

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

        # Subscriber for cube detection
        self.cubes_data = None
        self.cubes_received = False
        self.subscription = self.create_subscription(
            String,
            '/detected_cubes',
            self.cubes_callback,
            10
        )

    def send_gr_pose(self, open=True):
        self.gr_motion_done = False

        req = MotionPlanRequest()
        req.group_name = self.gripper_group
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 10

        jc = JointConstraint()
        jc.joint_name = self.gripper_joint
        jc.position = 0.05 if open else 0.029
        jc.tolerance_above = jc.tolerance_below = 0.001
        jc.weight = 1.0

        goal_constraints = Constraints(joint_constraints=[jc])
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

        # block until this gripper motion finishes
        while not self.gr_motion_done and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)


    def send_pose(self, x, y, z, roll=0.0, pitch=0.0):
        """Send end-effector pose goal"""
        self.motion_done = False
        self.get_logger().info(f"[ARM] Moving to: ({x:.3f}, {y:.3f}, {z:.3f})")
        
        pose = PoseStamped()
        pose.header.frame_id = self.base_link
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        
        yaw = math.atan2(y, x)
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        req = MotionPlanRequest()
        req.group_name = self.arm_group
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 10

        # Slow down motion: values in (0, 1]
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1
        
        pc = PositionConstraint()
        pc.header.frame_id = self.base_link
        pc.link_name = self.ee_link
        sp = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005])
        pc.constraint_region.primitives = [sp]
        pc.constraint_region.primitive_poses = [pose.pose]
        
        oc = OrientationConstraint()
        oc.header.frame_id = self.base_link
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = 0.001
        oc.weight = 1.0
        
        goal_constraints = Constraints(position_constraints=[pc], orientation_constraints=[oc])
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
        self.motion_done = True
        self.gr_motion_done = True

    def _feedback_cb(self, feedback_msg):
        state = getattr(feedback_msg.feedback, "state", "<unknown>")
        self.get_logger().debug(f"[Feedback] {state}")

    def move_upright(self, upright_coords=[0.12, 0.0, 0.39], pitch=math.radians(30)):
        x, y, z = upright_coords
        self.send_pose(x, y, z, pitch=pitch)
        while not self.motion_done:
            rclpy.spin_once(self, timeout_sec=0.1)

    def cubes_callback(self, msg):
        """Receive cube positions from vision node"""
        if not self.cubes_received: 
            try:
                self.cubes_data = dict(json.loads(msg.data))
                if {k.lower() for k in self.cubes_data} == {"yellow", "red", "blue"}:
                    self.cubes_received = True
            except json.JSONDecodeError as e:
                self.get_logger().error(f'Invalid JSON from vision: {e}')

    def pick_place_cubes(self, cubes_data: dict, color_order: list, place_position: list, stacking_height=0.0425):
        # reorder according to color_order
        cubes_data = {k.lower(): v for k, v in cubes_data.items()} # conver all keys into lowercase
        cubes_data = {key: cubes_data[key] for key in color_order}
        x_place, y_place, z_place = place_position[0], place_position[1], place_position[2]

        # open gripper
        self.send_gr_pose(open=True)
        while not self.gr_motion_done:
            rclpy.spin_once(self, timeout_sec=0.1)

        for key, value in cubes_data.items():
            self.get_logger().info(f"Picking up {key} cube...")

            x_pick, y_pick, z_hover = value[0], value[1], 0.2
            z_pick = 0.02

            # hover over the cube
            pitch = 1.57
            pitch_place = 1.57

            self.send_pose(x_pick, y_pick, z_hover, pitch=pitch)
            while not self.motion_done:
                rclpy.spin_once(self, timeout_sec=0.1)

            # lower to pick
            self.send_pose(x_pick, y_pick, z_pick, pitch=pitch)
            while not self.motion_done:
                rclpy.spin_once(self, timeout_sec=0.1)

            # close gripper
            self.send_gr_pose(open=False)
            while not self.gr_motion_done:
                rclpy.spin_once(self, timeout_sec=0.1)

            # lift up
            self.send_pose(x_pick, y_pick, z_hover, pitch=pitch)
            while not self.motion_done:
                rclpy.spin_once(self, timeout_sec=0.1)

            # move to place
            self.get_logger().info(f"Placing down {key} cube...")
            self.send_pose(x_place, y_place, z_hover, pitch=pitch_place)
            while not self.motion_done:
                rclpy.spin_once(self, timeout_sec=0.1)

            # lower to place
            self.send_pose(x_place, y_place, z_place, pitch=pitch_place)
            while not self.motion_done:
                rclpy.spin_once(self, timeout_sec=0.1)

            # release cube
            self.send_gr_pose(open=True)
            while not self.gr_motion_done:
                rclpy.spin_once(self, timeout_sec=0.1)

            # lift away
            self.send_pose(x_place, y_place, z_hover, pitch=pitch_place)
            while not self.motion_done:
                rclpy.spin_once(self, timeout_sec=0.1)

            self.move_upright()

            # increase stacking height for next cube
            z_place += stacking_height


def main(args=None):
    # Expect: sys.argv[1] = cube_order (e.g. "red,blue,yellow")
    #         sys.argv[2] = place_x (float)
    #         sys.argv[3] = place_y (float)
    # They come from the launch file via `arguments=[cube_order, place_x, place_y]`. [web:39][web:48]
    if len(sys.argv) < 4:
        print("Usage: manipulation_node <cube_order> <place_x> <place_y>")
        return

    cube_order_str = sys.argv[1]
    place_x_str = sys.argv[2]
    place_y_str = sys.argv[3]

    cube_order_str = "red, yellow, blue"
    place_x_str = 0.2
    place_y_str = 0.15

    # Parse inputs
    color_order = [c.strip() for c in cube_order_str.split(',')]
    place_x = float(place_x_str)
    place_y = float(place_y_str)

    # Fixed Z for placing (can be parameterised later)
    place_z = 0.03
    place_position = [place_x, place_y, place_z]

    time.sleep(5.0)  # wait for everything to initialize
    rclpy.init(args=args)
    node = MoveItEEClient()

    node.get_logger().info(
        f"Received order: {color_order}, place position: ({place_x:.3f}, {place_y:.3f}, {place_z:.3f})"
    )

    # Move to upright
    node.move_upright()
    # Vision node needs to be called now

    while True:
        if node.cubes_received:
            node.get_logger().info("All cubes detected!")
            cubes_data = node.cubes_data
            node.get_logger().info(f"########### {cubes_data} ###########")
            # Run pick & place loop
            node.pick_place_cubes(cubes_data, color_order, place_position)
            node.get_logger().info("Pick and place finished.")
            break

        node.get_logger().info(f"All cubes detected: {node.cubes_received}")

        # node.get_logger().info("Not all cubes have been detected yet")
        rclpy.spin_once(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
