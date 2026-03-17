from typing import Dict, List, Optional

from geometry_msgs.msg import Pose, TwistStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

from .math_utils import clamp_vector, matrix_to_quaternion, quaternion_to_matrix, rotation_matrix_to_rotvec
from .visual_servo_node import _DEFAULT_JOINT_NAMES, _fk_and_jacobian


def _pose_to_matrix(pose: Pose) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = quaternion_to_matrix(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    transform[:3, 3] = np.array(
        [pose.position.x, pose.position.y, pose.position.z],
        dtype=np.float64,
    )
    return transform


def _matrix_to_pose(transform: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x = float(transform[0, 3])
    pose.position.y = float(transform[1, 3])
    pose.position.z = float(transform[2, 3])
    quat = matrix_to_quaternion(transform[:3, :3])
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return pose


def _rotvec_to_matrix(rotvec: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-9:
        return np.eye(3, dtype=np.float64)
    axis = rotvec / angle
    x, y, z = float(axis[0]), float(axis[1]), float(axis[2])
    c = float(np.cos(angle))
    s = float(np.sin(angle))
    one_minus_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s],
            [y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s],
            [z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c],
        ],
        dtype=np.float64,
    )


class DragTargetServoNode(Node):
    def __init__(self) -> None:
        super().__init__("drag_target_servo")

        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("joint_names", _DEFAULT_JOINT_NAMES)
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("ee_command_frame", "tool0")
        self.declare_parameter("interactive_marker_server", "ee_drag_target")
        self.declare_parameter("interactive_marker_name", "tool0_target")
        self.declare_parameter("interactive_marker_scale", 0.10)
        self.declare_parameter("enable_axis_controls", True)
        self.declare_parameter("enable_plane_controls", True)
        self.declare_parameter("enable_6dof_controls", True)
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("linear_gain", 1.2)
        self.declare_parameter("angular_gain", 1.0)
        self.declare_parameter("max_linear_speed", 0.20)
        self.declare_parameter("max_angular_speed", 0.80)
        self.declare_parameter("position_tolerance_m", 0.005)
        self.declare_parameter("angle_tolerance_rad", 0.05)
        self.declare_parameter("workspace_min_xyz", [-0.20, -0.25, -0.05])
        self.declare_parameter("workspace_max_xyz", [0.30, 0.25, 0.28])
        self.declare_parameter("workspace_center_xyz", [0.0, 0.0, 0.12])
        self.declare_parameter("workspace_radius_m", 0.26)
        self.declare_parameter("max_target_lead_m", 0.08)
        self.declare_parameter("max_target_angle_from_current_rad", 0.75)
        self.declare_parameter("enable_twist_output", False)

        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.twist_topic = str(self.get_parameter("twist_topic").value)
        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_command_frame = str(self.get_parameter("ee_command_frame").value)
        self.marker_server_name = str(self.get_parameter("interactive_marker_server").value)
        self.marker_name = str(self.get_parameter("interactive_marker_name").value)
        self.marker_scale = float(self.get_parameter("interactive_marker_scale").value)
        self.enable_axis_controls = bool(self.get_parameter("enable_axis_controls").value)
        self.enable_plane_controls = bool(self.get_parameter("enable_plane_controls").value)
        self.enable_6dof_controls = bool(self.get_parameter("enable_6dof_controls").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.position_tolerance_m = float(self.get_parameter("position_tolerance_m").value)
        self.angle_tolerance_rad = float(self.get_parameter("angle_tolerance_rad").value)
        self.workspace_min_xyz = np.array(self.get_parameter("workspace_min_xyz").value, dtype=np.float64)
        self.workspace_max_xyz = np.array(self.get_parameter("workspace_max_xyz").value, dtype=np.float64)
        self.workspace_center_xyz = np.array(self.get_parameter("workspace_center_xyz").value, dtype=np.float64)
        self.workspace_radius_m = float(self.get_parameter("workspace_radius_m").value)
        self.max_target_lead_m = float(self.get_parameter("max_target_lead_m").value)
        self.max_target_angle_from_current_rad = float(
            self.get_parameter("max_target_angle_from_current_rad").value
        )
        self.enable_twist_output = bool(self.get_parameter("enable_twist_output").value)

        if self.control_rate_hz <= 0.0:
            raise ValueError(f"control_rate_hz must be > 0.0. Got {self.control_rate_hz}")
        if self.marker_scale <= 0.0:
            raise ValueError(f"interactive_marker_scale must be > 0.0. Got {self.marker_scale}")
        if self.workspace_min_xyz.shape != (3,) or self.workspace_max_xyz.shape != (3,):
            raise ValueError("workspace_min_xyz and workspace_max_xyz must each contain 3 values.")
        if self.workspace_center_xyz.shape != (3,):
            raise ValueError("workspace_center_xyz must contain 3 values.")
        if np.any(self.workspace_min_xyz > self.workspace_max_xyz):
            raise ValueError("workspace_min_xyz must be <= workspace_max_xyz elementwise.")
        if self.workspace_radius_m <= 0.0:
            raise ValueError(f"workspace_radius_m must be > 0.0. Got {self.workspace_radius_m}")
        if self.max_target_lead_m < 0.0:
            raise ValueError(f"max_target_lead_m must be >= 0.0. Got {self.max_target_lead_m}")
        if self.max_target_angle_from_current_rad < 0.0:
            raise ValueError(
                "max_target_angle_from_current_rad must be >= 0.0. "
                f"Got {self.max_target_angle_from_current_rad}"
            )

        self._current_q: Optional[Dict[str, float]] = None
        self._target_pose_world: Optional[np.ndarray] = None
        self._marker_seeded_from_joint_state = False
        self._last_disabled_warn_ns = 0
        self._warn_period_ns = 2_000_000_000
        self._last_clamp_warn_ns = 0

        self.server = InteractiveMarkerServer(self, self.marker_server_name)
        self.twist_pub = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self.create_subscription(JointState, self.joint_state_topic, self._joint_state_cb, 10)
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self._control_step)

        initial_pose_world = self._compute_nominal_tool_pose_world()
        self._target_pose_world = initial_pose_world.copy()
        self._insert_interactive_marker(initial_pose_world)

        self.get_logger().info(
            f"Drag target servo active. server=/{self.marker_server_name}, "
            f"twist_output_enabled={self.enable_twist_output}"
        )
        self.get_logger().info("Initialized EE drag target at the nominal tool0 pose.")
        if not self.enable_twist_output:
            self.get_logger().warn("Twist output is disabled. Set enable_servo:=true when ready to move.")

    def _joint_state_cb(self, msg: JointState) -> None:
        self._current_q = {str(name): float(pos) for name, pos in zip(msg.name, msg.position)}

        if not self._marker_seeded_from_joint_state:
            current_pose_world = self._compute_current_tool_pose_world()
            if current_pose_world is None:
                return
            self._target_pose_world = self._clamp_target_pose(current_pose_world.copy(), current_pose_world)
            self.server.setPose(
                self.marker_name,
                _matrix_to_pose(self._target_pose_world),
                header=Header(frame_id=self.base_frame),
            )
            self.server.applyChanges()
            self._marker_seeded_from_joint_state = True
            self.get_logger().info("Updated EE drag target to the current tool0 pose from /joint_states.")

    def _compute_current_tool_pose_world(self) -> Optional[np.ndarray]:
        if self._current_q is None:
            return None
        try:
            q = np.array([self._current_q[name] for name in self.joint_names], dtype=np.float64)
        except KeyError:
            return None
        transform_world_to_tool0, _ = _fk_and_jacobian(q)
        return transform_world_to_tool0

    def _compute_nominal_tool_pose_world(self) -> np.ndarray:
        q = np.zeros(len(self.joint_names), dtype=np.float64)
        transform_world_to_tool0, _ = _fk_and_jacobian(q)
        return transform_world_to_tool0

    def _insert_interactive_marker(self, pose_world: np.ndarray) -> None:
        marker = InteractiveMarker()
        marker.header.frame_id = self.base_frame
        marker.name = self.marker_name
        marker.description = "Drag tool0 target"
        marker.scale = float(self.marker_scale)
        marker.pose = _matrix_to_pose(pose_world)

        visual_marker = Marker()
        visual_marker.type = Marker.SPHERE
        visual_marker.scale.x = self.marker_scale * 0.40
        visual_marker.scale.y = self.marker_scale * 0.40
        visual_marker.scale.z = self.marker_scale * 0.40
        visual_marker.color.r = 0.95
        visual_marker.color.g = 0.40
        visual_marker.color.b = 0.15
        visual_marker.color.a = 0.85

        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.interaction_mode = InteractiveMarkerControl.NONE
        visual_control.markers.append(visual_marker)
        marker.controls.append(visual_control)

        if self.enable_6dof_controls:
            marker.controls.append(self._make_move_rotate_3d_control("move_rotate_3d"))
        if self.enable_plane_controls:
            marker.controls.extend(
                [
                    self._make_plane_control("move_xy", 1.0, 0.0, 1.0, 0.0),
                    self._make_plane_control("move_xz", 1.0, 0.0, 0.0, 1.0),
                    self._make_plane_control("move_yz", 1.0, 1.0, 0.0, 0.0),
                ]
            )
        if self.enable_axis_controls:
            marker.controls.extend(
                [
                    self._make_axis_control("move_x", 1.0, 1.0, 0.0, 0.0),
                    self._make_axis_control("move_y", 1.0, 0.0, 1.0, 0.0),
                    self._make_axis_control("move_z", 1.0, 0.0, 0.0, 1.0),
                    self._make_rotate_control("rotate_x", 1.0, 1.0, 0.0, 0.0),
                    self._make_rotate_control("rotate_y", 1.0, 0.0, 1.0, 0.0),
                    self._make_rotate_control("rotate_z", 1.0, 0.0, 0.0, 1.0),
                ]
            )

        self.server.insert(marker, feedback_callback=self._feedback_cb)
        self.server.applyChanges()

    @staticmethod
    def _make_axis_control(name: str, w: float, x: float, y: float, z: float) -> InteractiveMarkerControl:
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation.w = w
        control.orientation.x = x
        control.orientation.y = y
        control.orientation.z = z
        control.orientation_mode = InteractiveMarkerControl.FIXED
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        return control

    @staticmethod
    def _make_plane_control(name: str, w: float, x: float, y: float, z: float) -> InteractiveMarkerControl:
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation.w = w
        control.orientation.x = x
        control.orientation.y = y
        control.orientation.z = z
        control.orientation_mode = InteractiveMarkerControl.FIXED
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        return control

    @staticmethod
    def _make_rotate_control(name: str, w: float, x: float, y: float, z: float) -> InteractiveMarkerControl:
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation.w = w
        control.orientation.x = x
        control.orientation.y = y
        control.orientation.z = z
        control.orientation_mode = InteractiveMarkerControl.FIXED
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        return control

    @staticmethod
    def _make_move_rotate_3d_control(name: str) -> InteractiveMarkerControl:
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation_mode = InteractiveMarkerControl.INHERIT
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        return control

    def _apply_workspace_clamp(self, pose_world: np.ndarray) -> np.ndarray:
        clamped = pose_world.copy()
        position = clamped[:3, 3].copy()
        position = np.clip(position, self.workspace_min_xyz, self.workspace_max_xyz)

        radial = position - self.workspace_center_xyz
        radial_norm = float(np.linalg.norm(radial))
        if radial_norm > self.workspace_radius_m:
            position = self.workspace_center_xyz + radial * (self.workspace_radius_m / radial_norm)

        clamped[:3, 3] = position
        return clamped

    def _apply_current_pose_lead_limit(self, pose_world: np.ndarray, current_pose_world: Optional[np.ndarray]) -> np.ndarray:
        if current_pose_world is None:
            return pose_world

        limited = pose_world.copy()
        delta = limited[:3, 3] - current_pose_world[:3, 3]
        delta_norm = float(np.linalg.norm(delta))
        if self.max_target_lead_m > 0.0 and delta_norm > self.max_target_lead_m:
            limited[:3, 3] = current_pose_world[:3, 3] + delta * (self.max_target_lead_m / delta_norm)

        if self.max_target_angle_from_current_rad > 0.0:
            rot_error = current_pose_world[:3, :3].T @ limited[:3, :3]
            rotvec = rotation_matrix_to_rotvec(rot_error)
            angle = float(np.linalg.norm(rotvec))
            if angle > self.max_target_angle_from_current_rad and angle > 1e-9:
                limited[:3, :3] = current_pose_world[:3, :3] @ _rotvec_to_matrix(
                    rotvec * (self.max_target_angle_from_current_rad / angle)
                )
        return limited

    def _clamp_target_pose(self, pose_world: np.ndarray, current_pose_world: Optional[np.ndarray]) -> np.ndarray:
        clamped = self._apply_workspace_clamp(pose_world)
        clamped = self._apply_current_pose_lead_limit(clamped, current_pose_world)
        return clamped

    def _update_target_pose(self, pose_world: np.ndarray, current_pose_world: Optional[np.ndarray]) -> None:
        clamped_pose = self._clamp_target_pose(pose_world, current_pose_world)
        self._target_pose_world = clamped_pose

        position_error = float(np.linalg.norm(clamped_pose[:3, 3] - pose_world[:3, 3]))
        rotation_error = rotation_matrix_to_rotvec(pose_world[:3, :3].T @ clamped_pose[:3, :3])
        angle_error = float(np.linalg.norm(rotation_error))

        if position_error > 1e-5 or angle_error > 1e-4:
            self.server.setPose(
                self.marker_name,
                _matrix_to_pose(clamped_pose),
                header=Header(frame_id=self.base_frame),
            )
            self.server.applyChanges()
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_clamp_warn_ns > self._warn_period_ns:
                self.get_logger().warn(
                    f"Clamped drag target to reachable region "
                    f"(position_adjust={position_error:.3f} m, angle_adjust={angle_error:.2f} rad)."
                )
                self._last_clamp_warn_ns = now_ns

    def _feedback_cb(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type not in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            return
        current_pose_world = self._compute_current_tool_pose_world()
        self._update_target_pose(_pose_to_matrix(feedback.pose), current_pose_world)

    def _control_step(self) -> None:
        if self._target_pose_world is None:
            return

        current_pose_world = self._compute_current_tool_pose_world()
        if current_pose_world is None:
            return

        self._update_target_pose(self._target_pose_world, current_pose_world)
        if self._target_pose_world is None:
            return

        transform_tool_to_target = np.linalg.inv(current_pose_world) @ self._target_pose_world
        pos_error = transform_tool_to_target[:3, 3]
        rot_error = rotation_matrix_to_rotvec(transform_tool_to_target[:3, :3])

        dist_error = float(np.linalg.norm(pos_error))
        angle_error = float(np.linalg.norm(rot_error))

        linear_cmd = np.zeros(3, dtype=np.float64)
        angular_cmd = np.zeros(3, dtype=np.float64)
        if dist_error > self.position_tolerance_m:
            linear_cmd = clamp_vector(self.linear_gain * pos_error, self.max_linear_speed)
        if angle_error > self.angle_tolerance_rad:
            angular_cmd = clamp_vector(self.angular_gain * rot_error, self.max_angular_speed)

        self._publish_twist(linear_cmd, angular_cmd)

    def _publish_twist(self, linear: np.ndarray, angular: np.ndarray) -> None:
        if not self.enable_twist_output:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_disabled_warn_ns > self._warn_period_ns:
                lin_norm = float(np.linalg.norm(linear))
                ang_norm = float(np.linalg.norm(angular))
                if lin_norm > 1e-4 or ang_norm > 1e-4:
                    self.get_logger().warn(
                        f"Twist output disabled while dragger command is non-zero "
                        f"(lin={lin_norm:.3f}, ang={ang_norm:.3f})."
                    )
                    self._last_disabled_warn_ns = now_ns
            return

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.ee_command_frame
        twist_msg.twist.linear.x = float(linear[0])
        twist_msg.twist.linear.y = float(linear[1])
        twist_msg.twist.linear.z = float(linear[2])
        twist_msg.twist.angular.x = float(angular[0])
        twist_msg.twist.angular.y = float(angular[1])
        twist_msg.twist.angular.z = float(angular[2])
        self.twist_pub.publish(twist_msg)


def main() -> None:
    rclpy.init()
    node = DragTargetServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
