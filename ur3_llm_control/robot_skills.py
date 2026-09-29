#!/usr/bin/env python3
import time
import copy
from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint, JointConstraint,
    BoundingVolume, PlanningOptions, MoveItErrorCodes,
    AttachedCollisionObject, CollisionObject, PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene

from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

class SkillStatus(Enum):
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    INVALID_OBJECT = "INVALID_OBJECT"
    INVALID_ZONE = "INVALID_ZONE"
    PLANNING_FAILED = "PLANNING_FAILED"

CUBE_SIZE = 0.04

HOME_JOINTS = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -1.57,
    "elbow_joint": -1.57,
    "wrist_1_joint": -1.57,
    "wrist_2_joint": 1.57,
    "wrist_3_joint": 0.0,
}

GAZEBO_ZONE_POSES = {
    "zone_a":    {"x": 0.40, "y":  0.15, "z": 0.025},
    "zone_b":    {"x": 0.40, "y":  0.00, "z": 0.025},
    "zone_c":    {"x": 0.40, "y": -0.15, "z": 0.025},
    "temp_zone": {"x": 0.34, "y":  0.30, "z": 0.025},
}

class RobotSkills:
    def __init__(self, node: Node,
                 group_name: str = "ur_manipulator",
                 ee_link: str = "tool0",
                 base_frame: str = "base_link",
                 approach_offset_z: float = 0.12,
                 planning_time: float = 7.0):
        self.node = node
        self.group_name = group_name
        self.ee_link = ee_link
        self.base_frame = base_frame
        self.approach_offset_z = approach_offset_z
        
        self.grasp_offset_z = 0.06
        self.planning_time = planning_time

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        self._move_client = ActionClient(node, MoveGroup, "move_action")
        self._scene_client = node.create_client(ApplyPlanningScene, "apply_planning_scene")
        self._set_pose_client = node.create_client(SetEntityState, "/gazebo/set_entity_state")
        
        self._attached_object = None
        self._carry_timer = None
        self._carry_future = None

        self.node.get_logger().info("Cho move_action server...")
        self._move_client.wait_for_server(timeout_sec=10.0)
        self._set_pose_client.wait_for_service(timeout_sec=10.0)

    def _spin_wait(self, future, timeout_sec=30.0):
        start = time.time()
        while not future.done():
            if time.time() - start > timeout_sec:
                raise TimeoutError("Het thoi gian cho future hoan thanh")
            time.sleep(0.02)
        return future.result()

    def _lookup_pose(self, frame_name: str) -> PoseStamped | None:
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, frame_name,
                rclpy.time.Time(), timeout=Duration(seconds=2.0)
            )
        except TransformException as ex:
            self.node.get_logger().warn(f"TF lookup that bai cho '{frame_name}': {ex}")
            return None

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = trans.transform.translation.x
        pose.pose.position.y = trans.transform.translation.y
        pose.pose.position.z = trans.transform.translation.z
        pose.pose.orientation = trans.transform.rotation
        return pose

    def _build_pose_goal(self, pose: PoseStamped) -> MoveGroup.Goal:
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = self.ee_link
        prim = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.02])
        bv = BoundingVolume(primitives=[prim], primitive_poses=[pose.pose])
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0

        pose.pose.orientation.x = 1.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = self.base_frame
        ori_constraint.link_name = self.ee_link
        ori_constraint.orientation = pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.6
        ori_constraint.absolute_y_axis_tolerance = 0.6
        ori_constraint.absolute_z_axis_tolerance = 3.14
        ori_constraint.weight = 1.0

        constraints = Constraints(position_constraints=[pos_constraint], orientation_constraints=[ori_constraint])

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.goal_constraints = [constraints]
        goal.request.allowed_planning_time = self.planning_time
        goal.request.num_planning_attempts = 15
        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.15
        goal.planning_options = PlanningOptions(plan_only=False)
        return goal

    def _build_joint_goal(self, joint_positions: dict) -> MoveGroup.Goal:
        constraints = Constraints()
        joint_constraints = []
        for name, val in joint_positions.items():
            jc = JointConstraint(joint_name=name, position=val, tolerance_above=0.05, tolerance_below=0.05, weight=1.0)
            joint_constraints.append(jc)
        constraints.joint_constraints = joint_constraints

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.goal_constraints = [constraints]
        goal.request.allowed_planning_time = self.planning_time
        goal.request.num_planning_attempts = 10
        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.15
        goal.planning_options = PlanningOptions(plan_only=False)
        return goal

    def _send_and_wait(self, goal_msg: MoveGroup.Goal) -> bool:
        send_future = self._move_client.send_goal_async(goal_msg)
        goal_handle = self._spin_wait(send_future)
        if not goal_handle.accepted:
            self.node.get_logger().error("MoveGroup goal bi tu choi")
            return False

        self.node.get_logger().info("Dang cho robot di chuyen...")
        result_future = goal_handle.get_result_async()
        result = self._spin_wait(result_future, timeout_sec=60.0)
        error_code = result.result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            self.node.get_logger().error(f"MoveGroup that bai, error_code={error_code}")
            return False
        return True

    def _move_to_pose(self, pose: PoseStamped, max_retries: int = 3) -> bool:
        for attempt in range(1, max_retries + 1):
            goal = self._build_pose_goal(pose)
            try:
                if self._send_and_wait(goal):
                    return True
            except TimeoutError:
                self.node.get_logger().warn(f"Het thoi gian cho MoveGroup (lan {attempt}/{max_retries})")
            else:
                self.node.get_logger().warn(f"Move that bai (lan {attempt}/{max_retries}), thu lai...")
            
            self.node.get_logger().info("Homing robot before retry...")
            self.home()
            time.sleep(1.0)

        return False

    def _apply_scene(self, scene_diff: PlanningScene):
        req = ApplyPlanningScene.Request(scene=scene_diff)
        self._scene_client.call_async(req)

    def _teleport_model(self, model_name: str, pose: Pose, reference_frame: str = "world"):
        if not self._set_pose_client.service_is_ready():
            return False

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = model_name
        req.state.pose = pose
        req.state.reference_frame = reference_frame

        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = 0.0
        req.state.twist.angular.y = 0.0
        req.state.twist.angular.z = 0.0

        self._set_pose_client.call_async(req)
        return True

    def _start_carry(self):
        self._stop_carry()
        self._carry_future = None
        self._carry_timer = self.node.create_timer(0.2, self._carry_cb)

    def _stop_carry(self):
        if self._carry_timer is not None:
            self._carry_timer.cancel()
            self.node.destroy_timer(self._carry_timer)
            self._carry_timer = None
        f, t0 = self._carry_future, time.time()
        while f is not None and not f.done() and time.time() - t0 < 3.0:
            time.sleep(0.02)

    def _carry_cb(self):
        if not self._attached_object:
            return
        if self._carry_future is not None and not self._carry_future.done():
            return
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.ee_link, rclpy.time.Time())
        except TransformException:
            return
        req = SetEntityState.Request()
        req.state.name = self._attached_object
        req.state.pose.position.x = t.transform.translation.x
        req.state.pose.position.y = t.transform.translation.y
        req.state.pose.position.z = t.transform.translation.z - self.grasp_offset_z
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = "world"
        self._carry_future = self._set_pose_client.call_async(req)

    def _remove_world_object(self, name: str):
        co = CollisionObject(id=name, operation=CollisionObject.REMOVE)
        co.header.frame_id = self.base_frame
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [co]
        self._apply_scene(scene)

    def _add_world_box(self, name: str, pose: PoseStamped, size: float = CUBE_SIZE):
        co = CollisionObject(id=name, operation=CollisionObject.ADD)
        co.header.frame_id = self.base_frame
        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[size, size, size])
        co.primitives = [prim]
        co.primitive_poses = [pose.pose]
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [co]
        self._apply_scene(scene)

    def _close_gripper(self, object_name: str):
        aco = AttachedCollisionObject()
        aco.link_name = self.ee_link
        aco.object.id = object_name
        aco.object.header.frame_id = self.ee_link
        aco.object.operation = CollisionObject.ADD
        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[CUBE_SIZE, CUBE_SIZE, CUBE_SIZE])
        rel_pose = Pose()
        
        rel_pose.position.z = -self.grasp_offset_z
        rel_pose.orientation.w = 1.0
        aco.object.primitives = [prim]
        aco.object.primitive_poses = [rel_pose]
        aco.touch_links = [self.ee_link, "wrist_3_link", "wrist_2_link", "flange", "work_table"]

        scene = PlanningScene(is_diff=True)
        scene.robot_state.attached_collision_objects = [aco]
        scene.robot_state.is_diff = True
        self._apply_scene(scene)
        self._attached_object = object_name

    def _open_gripper(self, drop_pose: PoseStamped = None):
        if not self._attached_object:
            return
        name = self._attached_object
        self._stop_carry()

        aco = AttachedCollisionObject()
        aco.link_name = self.ee_link
        aco.object.id = name
        aco.object.operation = CollisionObject.REMOVE

        scene = PlanningScene(is_diff=True)
        scene.robot_state.attached_collision_objects = [aco]
        scene.robot_state.is_diff = True
        self._apply_scene(scene)
        self._attached_object = None

    def home(self) -> SkillStatus:
        self._stop_carry()
        goal = self._build_joint_goal(HOME_JOINTS)
        return SkillStatus.SUCCESS if self._send_and_wait(goal) else SkillStatus.PLANNING_FAILED

    def pick(self, object_name: str) -> SkillStatus:
        pose = self._lookup_pose(object_name)
        if pose is None: return SkillStatus.INVALID_OBJECT
            
        grasp_pose = copy.deepcopy(pose)
        grasp_pose.pose.position.z += self.grasp_offset_z

        above = copy.deepcopy(grasp_pose)
        above.pose.position.z += self.approach_offset_z

        
        self._remove_world_object(object_name)

        if not self._move_to_pose(above): return SkillStatus.PLANNING_FAILED

        if not self._move_to_pose(grasp_pose): return SkillStatus.PLANNING_FAILED

        self._close_gripper(object_name)
        self._start_carry()

        if not self._move_to_pose(above): return SkillStatus.PLANNING_FAILED

        return SkillStatus.SUCCESS

    def place(self, object_name: str, zone_name: str) -> SkillStatus:
        pose = self._lookup_pose(zone_name)
        if pose is None: return SkillStatus.INVALID_ZONE

        place_pose = copy.deepcopy(pose)
        place_pose.pose.position.z += self.grasp_offset_z

        above = copy.deepcopy(place_pose)
        above.pose.position.z += self.approach_offset_z
        if not self._move_to_pose(above): return SkillStatus.PLANNING_FAILED
        if not self._move_to_pose(place_pose): return SkillStatus.PLANNING_FAILED

        
        drop = copy.deepcopy(pose)
        drop.pose.position.z += CUBE_SIZE / 2
        self._open_gripper(drop_pose=None)
        self._remove_world_object(object_name)
        self._teleport_model(object_name, drop.pose)

        if not self._move_to_pose(above): return SkillStatus.PLANNING_FAILED

        
        self._add_world_box(object_name, drop, size=CUBE_SIZE + 0.04)
        return SkillStatus.SUCCESS
