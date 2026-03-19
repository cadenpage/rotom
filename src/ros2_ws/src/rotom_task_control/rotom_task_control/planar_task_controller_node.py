import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Vector3
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

from .kinematics import RotomKinematics


def _clamp_vector_norm(vec: np.ndarray, max_norm: float) -> np.ndarray:
    norm = np.linalg.norm(vec)
    if max_norm <= 0.0 or norm <= max_norm or norm <= 1e-12:
        return vec
    return vec * (max_norm / norm)


def _wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class PlanarTaskControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("planar_task_controller")

        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_command_topic", "/joint_commands")
        self.declare_parameter("target_topic", "/rotom_task/target")
        self.declare_parameter("delta_topic", "/rotom_task/delta_cmd")
        self.declare_parameter("pitch_target_topic", "/rotom_task/pitch_target")
        self.declare_parameter("pitch_delta_topic", "/rotom_task/pitch_delta")
        self.declare_parameter("current_pose_topic", "/rotom_task/current_pose")
        self.declare_parameter("target_pose_topic", "/rotom_task/target_pose")
        self.declare_parameter("command_pose_topic", "/rotom_task/command_pose")
        self.declare_parameter("current_pitch_topic", "/rotom_task/current_pitch")
        self.declare_parameter("target_pitch_debug_topic", "/rotom_task/target_pitch")
        self.declare_parameter("command_pitch_debug_topic", "/rotom_task/command_pitch")
        self.declare_parameter("enabled_topic", "/rotom_task/controller_enabled")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("delta_timeout_s", 0.25)
        self.declare_parameter("initialize_target_from_current_pose", True)
        self.declare_parameter("initialize_fixed_z_from_current_pose", True)
        self.declare_parameter("fixed_z_enabled", True)
        self.declare_parameter("fixed_z", 0.0)
        self.declare_parameter("initialize_pitch_from_current_pose", True)
        self.declare_parameter("pitch_target_rad", 0.0)
        self.declare_parameter("tool_axis_local", [0.0, 1.0, 0.0])
        self.declare_parameter("task_weights", [1.0, 1.0, 2.0])
        self.declare_parameter("task_servo_gain", 0.6)
        self.declare_parameter("ik_damping", 1e-3)
        self.declare_parameter("smoothness_weight", 5e-3)
        self.declare_parameter("joint_center_weight", 5e-4)
        self.declare_parameter("position_tolerance_m", 5e-3)
        self.declare_parameter("pitch_tolerance_rad", 0.03)
        self.declare_parameter("max_joint_step", 0.08)
        self.declare_parameter("command_smoothing_alpha", 0.35)
        self.declare_parameter("max_target_linear_speed", 0.08)
        self.declare_parameter("max_target_pitch_rate", 1.2)
        self.declare_parameter("dependent_c_search_window_rad", 0.35)
        self.declare_parameter("workspace_radius_min", 0.05)
        self.declare_parameter("workspace_radius_max", 0.30)
        self.declare_parameter("workspace_z_min", -0.05)
        self.declare_parameter("workspace_z_max", 0.30)
        self.declare_parameter("warn_period_s", 2.0)

        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.joint_command_topic = str(self.get_parameter("joint_command_topic").value)
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.delta_topic = str(self.get_parameter("delta_topic").value)
        self.pitch_target_topic = str(self.get_parameter("pitch_target_topic").value)
        self.pitch_delta_topic = str(self.get_parameter("pitch_delta_topic").value)
        self.current_pose_topic = str(self.get_parameter("current_pose_topic").value)
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.command_pose_topic = str(self.get_parameter("command_pose_topic").value)
        self.current_pitch_topic = str(self.get_parameter("current_pitch_topic").value)
        self.target_pitch_debug_topic = str(self.get_parameter("target_pitch_debug_topic").value)
        self.command_pitch_debug_topic = str(self.get_parameter("command_pitch_debug_topic").value)
        self.enabled_topic = str(self.get_parameter("enabled_topic").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.delta_timeout_s = float(self.get_parameter("delta_timeout_s").value)
        self.initialize_target_from_current_pose = bool(self.get_parameter("initialize_target_from_current_pose").value)
        self.initialize_fixed_z_from_current_pose = bool(self.get_parameter("initialize_fixed_z_from_current_pose").value)
        self.fixed_z_enabled = bool(self.get_parameter("fixed_z_enabled").value)
        self.fixed_z = float(self.get_parameter("fixed_z").value)
        self.initialize_pitch_from_current_pose = bool(self.get_parameter("initialize_pitch_from_current_pose").value)
        self.pitch_target_rad = float(self.get_parameter("pitch_target_rad").value)
        self.tool_axis_local = np.asarray(self.get_parameter("tool_axis_local").value, dtype=float)
        self.task_weights = np.asarray(self.get_parameter("task_weights").value, dtype=float)
        self.task_servo_gain = float(self.get_parameter("task_servo_gain").value)
        self.ik_damping = float(self.get_parameter("ik_damping").value)
        self.smoothness_weight = float(self.get_parameter("smoothness_weight").value)
        self.joint_center_weight = float(self.get_parameter("joint_center_weight").value)
        self.position_tolerance_m = float(self.get_parameter("position_tolerance_m").value)
        self.pitch_tolerance_rad = float(self.get_parameter("pitch_tolerance_rad").value)
        self.max_joint_step = float(self.get_parameter("max_joint_step").value)
        self.command_smoothing_alpha = float(self.get_parameter("command_smoothing_alpha").value)
        self.max_target_linear_speed = float(self.get_parameter("max_target_linear_speed").value)
        self.max_target_pitch_rate = float(self.get_parameter("max_target_pitch_rate").value)
        self.dependent_c_search_window_rad = float(self.get_parameter("dependent_c_search_window_rad").value)
        self.workspace_radius_min = float(self.get_parameter("workspace_radius_min").value)
        self.workspace_radius_max = float(self.get_parameter("workspace_radius_max").value)
        self.workspace_z_min = float(self.get_parameter("workspace_z_min").value)
        self.workspace_z_max = float(self.get_parameter("workspace_z_max").value)
        self.warn_period_ns = int(float(self.get_parameter("warn_period_s").value) * 1e9)

        self.kin = RotomKinematics()
        self._current_q: Optional[np.ndarray] = None
        self._last_command_q: Optional[np.ndarray] = None
        self._desired_target_xyz: Optional[np.ndarray] = None
        self._command_target_xyz: Optional[np.ndarray] = None
        self._desired_pitch_target_rad = self.pitch_target_rad
        self._command_pitch_target_rad = self.pitch_target_rad
        self._latest_delta_cmd = np.zeros(3, dtype=float)
        self._latest_delta_time_ns: Optional[int] = None
        self._last_warn_ns = 0

        self.command_pub = self.create_publisher(JointState, self.joint_command_topic, 10)
        self.current_pose_pub = self.create_publisher(PointStamped, self.current_pose_topic, 10)
        self.target_pose_pub = self.create_publisher(PointStamped, self.target_pose_topic, 10)
        self.command_pose_pub = self.create_publisher(PointStamped, self.command_pose_topic, 10)
        self.current_pitch_pub = self.create_publisher(Float64, self.current_pitch_topic, 10)
        self.target_pitch_pub = self.create_publisher(Float64, self.target_pitch_debug_topic, 10)
        self.command_pitch_pub = self.create_publisher(Float64, self.command_pitch_debug_topic, 10)
        self.enabled_pub = self.create_publisher(Bool, self.enabled_topic, 10)

        self.create_subscription(JointState, self.joint_state_topic, self._joint_state_cb, 10)
        self.create_subscription(PointStamped, self.target_topic, self._target_cb, 10)
        self.create_subscription(Vector3, self.delta_topic, self._delta_cb, 10)
        self.create_subscription(Float64, self.pitch_target_topic, self._pitch_target_cb, 10)
        self.create_subscription(Float64, self.pitch_delta_topic, self._pitch_delta_cb, 10)
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self._control_step)

        self.get_logger().info(
            "planar_task_controller active. "
            f"joint_state_topic={self.joint_state_topic}, joint_command_topic={self.joint_command_topic}, "
            f"target_topic={self.target_topic}, delta_topic={self.delta_topic}"
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in self.kin.JOINT_NAMES):
            return
        q = np.array([positions[name] for name in self.kin.JOINT_NAMES], dtype=float)
        self._current_q = self.kin.clamp(q)

    def _target_cb(self, msg: PointStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id not in ("ground", "world"):
            self._warn(f"ignoring task target in unsupported frame '{msg.header.frame_id}'")
            return
        if self._desired_target_xyz is None:
            self._desired_target_xyz = np.zeros(3, dtype=float)
        self._desired_target_xyz[0] = float(msg.point.x)
        self._desired_target_xyz[1] = float(msg.point.y)
        if self.fixed_z_enabled:
            self._desired_target_xyz[2] = self.fixed_z
        else:
            self._desired_target_xyz[2] = float(msg.point.z)
        self._desired_target_xyz = self._clamp_target(self._desired_target_xyz)

    def _delta_cb(self, msg: Vector3) -> None:
        self._latest_delta_cmd = np.array((msg.x, msg.y, msg.z), dtype=float)
        self._latest_delta_time_ns = self.get_clock().now().nanoseconds

    def _pitch_target_cb(self, msg: Float64) -> None:
        self._desired_pitch_target_rad = _wrap_to_pi(float(msg.data))

    def _pitch_delta_cb(self, msg: Float64) -> None:
        self._desired_pitch_target_rad = _wrap_to_pi(self._desired_pitch_target_rad + float(msg.data))

    def _publish_bool(self, value: bool) -> None:
        msg = Bool()
        msg.data = value
        self.enabled_pub.publish(msg)

    def _publish_point(self, publisher, xyz: np.ndarray) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ground"
        msg.point.x = float(xyz[0])
        msg.point.y = float(xyz[1])
        msg.point.z = float(xyz[2])
        publisher.publish(msg)

    def _publish_pitch(self, publisher, pitch_rad: float) -> None:
        msg = Float64()
        msg.data = float(pitch_rad)
        publisher.publish(msg)

    def _warn(self, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_warn_ns > self.warn_period_ns:
            self.get_logger().warn(message)
            self._last_warn_ns = now_ns

    def _current_pose(self, q: np.ndarray) -> np.ndarray:
        return self.kin.forward_kinematics(q).position

    def _pitch_measure(self, q: np.ndarray) -> float:
        fk = self.kin.forward_kinematics(q)
        axis_world = fk.rotation @ self.tool_axis_local
        radial_vec = np.array(
            (
                fk.position[0] - self.kin.base_origin[0],
                fk.position[1] - self.kin.base_origin[1],
                0.0,
            ),
            dtype=float,
        )
        radial_norm = np.linalg.norm(radial_vec[:2])
        if radial_norm <= 1e-9:
            radial_hat = np.array((1.0, 0.0, 0.0), dtype=float)
        else:
            radial_hat = radial_vec / radial_norm
        return math.atan2(float(np.dot(axis_world, radial_hat)), float(-axis_world[2]))

    def _solve_dependent_c(self, oab: np.ndarray, seed_c: float, pitch_target_rad: float) -> float:
        oab_np = np.asarray(oab, dtype=float)
        c_lo = float(self.kin.JOINT_LOWER[3])
        c_hi = float(self.kin.JOINT_UPPER[3])

        def error_for(c_value: float) -> float:
            q = np.array((oab_np[0], oab_np[1], oab_np[2], c_value), dtype=float)
            return abs(_wrap_to_pi(self._pitch_measure(q) - pitch_target_rad))

        seed_c = float(np.clip(seed_c, c_lo, c_hi))
        window = max(self.dependent_c_search_window_rad, 0.02)
        coarse_lo = max(c_lo, seed_c - window)
        coarse_hi = min(c_hi, seed_c + window)
        coarse = np.linspace(coarse_lo, coarse_hi, 81, dtype=float)
        best_c = seed_c
        best_err = error_for(best_c)
        for c_value in coarse:
            err = error_for(float(c_value))
            if err < best_err:
                best_err = err
                best_c = float(c_value)

        coarse_step = (c_hi - c_lo) / max(len(coarse) - 1, 1)
        refine_lo = max(c_lo, best_c - coarse_step)
        refine_hi = min(c_hi, best_c + coarse_step)
        for c_value in np.linspace(refine_lo, refine_hi, 41, dtype=float):
            err = error_for(float(c_value))
            if err < best_err:
                best_err = err
                best_c = float(c_value)

        return best_c

    def _full_q_from_reduced(self, reduced_q: np.ndarray, seed_c: float, pitch_target_rad: float) -> np.ndarray:
        reduced_np = np.asarray(reduced_q, dtype=float).copy()
        reduced_np[0] = float(np.clip(reduced_np[0], self.kin.JOINT_LOWER[0], self.kin.JOINT_UPPER[0]))
        reduced_np[1] = float(np.clip(reduced_np[1], self.kin.JOINT_LOWER[1], self.kin.JOINT_UPPER[1]))
        reduced_np[2] = float(np.clip(reduced_np[2], self.kin.JOINT_LOWER[2], self.kin.JOINT_UPPER[2]))

        c_value = self._solve_dependent_c(reduced_np, seed_c, pitch_target_rad)
        return self.kin.clamp(np.array((reduced_np[0], reduced_np[1], reduced_np[2], c_value), dtype=float))

    def _reduced_position_jacobian(
        self, reduced_q: np.ndarray, seed_c: float, pitch_target_rad: float, eps: float = 1e-4
    ) -> tuple[np.ndarray, np.ndarray]:
        base_q = self._full_q_from_reduced(reduced_q, seed_c, pitch_target_rad)
        base_position = self.kin.forward_kinematics(base_q).position
        jac = np.zeros((3, 3), dtype=float)
        for joint_idx in range(3):
            reduced_perturbed = np.asarray(reduced_q, dtype=float).copy()
            reduced_perturbed[joint_idx] += eps
            q_perturbed = self._full_q_from_reduced(reduced_perturbed, base_q[3], pitch_target_rad)
            pos_perturbed = self.kin.forward_kinematics(q_perturbed).position
            jac[:, joint_idx] = (pos_perturbed - base_position) / eps
        return jac, base_q

    def _compute_servo_command(
        self, target_xyz: np.ndarray, pitch_target_rad: float, q_current: np.ndarray, q_prev: np.ndarray
    ) -> np.ndarray:
        reduced_q = np.asarray(q_current[:3], dtype=float)
        prev_reduced_q = np.asarray(q_prev[:3], dtype=float)
        reduced_mid = self.kin.joint_midpoints()[:3]
        jac, q_projected = self._reduced_position_jacobian(reduced_q, float(q_prev[3]), pitch_target_rad)
        error = (target_xyz - self.kin.forward_kinematics(q_projected).position) * self.task_servo_gain
        weights = np.diag(self.task_weights)
        lhs = jac.T @ weights @ jac
        lhs += (self.ik_damping + self.smoothness_weight + self.joint_center_weight) * np.eye(3, dtype=float)
        rhs = jac.T @ weights @ error
        rhs -= self.smoothness_weight * (reduced_q - prev_reduced_q)
        rhs -= self.joint_center_weight * (reduced_q - reduced_mid)

        try:
            du = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            du = np.linalg.lstsq(lhs, rhs, rcond=None)[0]

        du = _clamp_vector_norm(du, self.max_joint_step)
        reduced_next = reduced_q + du
        alpha = min(max(self.command_smoothing_alpha, 0.0), 1.0)
        reduced_smooth = (1.0 - alpha) * prev_reduced_q + alpha * reduced_next
        return self._full_q_from_reduced(reduced_smooth, q_prev[3], pitch_target_rad)

    def _clamp_target(self, target_xyz: np.ndarray) -> np.ndarray:
        clamped = target_xyz.copy()
        radial_vec = clamped[:2] - self.kin.base_origin[:2]
        radial_norm = float(np.linalg.norm(radial_vec))
        if radial_norm > 1e-9:
            clamped_radius = min(max(radial_norm, self.workspace_radius_min), self.workspace_radius_max)
            clamped[:2] = self.kin.base_origin[:2] + radial_vec * (clamped_radius / radial_norm)
        clamped[2] = min(max(clamped[2], self.workspace_z_min), self.workspace_z_max)
        return clamped

    def _maybe_initialize_target(self) -> bool:
        if self._current_q is None:
            self._publish_bool(False)
            return False
        if self._desired_target_xyz is not None:
            return True
        if not self.initialize_target_from_current_pose:
            return False
        current_pose = self._current_pose(self._current_q)
        if self.fixed_z_enabled and self.initialize_fixed_z_from_current_pose:
            self.fixed_z = float(current_pose[2])
        if self.fixed_z_enabled:
            current_pose[2] = self.fixed_z
        self._desired_target_xyz = current_pose.copy()
        self._command_target_xyz = current_pose.copy()
        if self.initialize_pitch_from_current_pose:
            self._desired_pitch_target_rad = self._pitch_measure(self._current_q)
        self._command_pitch_target_rad = self._desired_pitch_target_rad
        self._last_command_q = self._full_q_from_reduced(
            self._current_q[:3], float(self._current_q[3]), self._command_pitch_target_rad
        )
        self.get_logger().info(
            f"initialized task target from current pose: x={current_pose[0]:.3f}, y={current_pose[1]:.3f}, z={current_pose[2]:.3f}"
        )
        return True

    def _integrate_delta_cmd(self, dt_s: float) -> None:
        if self._desired_target_xyz is None or self._latest_delta_time_ns is None:
            return
        age_s = (self.get_clock().now().nanoseconds - self._latest_delta_time_ns) * 1e-9
        if age_s > self.delta_timeout_s:
            return
        self._desired_target_xyz += self._latest_delta_cmd * dt_s
        if self.fixed_z_enabled:
            self._desired_target_xyz[2] = self.fixed_z
        self._desired_target_xyz = self._clamp_target(self._desired_target_xyz)

    def _slew_command_targets(self, dt_s: float) -> None:
        if self._desired_target_xyz is None:
            return
        if self._command_target_xyz is None:
            self._command_target_xyz = self._desired_target_xyz.copy()

        max_linear_step = max(self.max_target_linear_speed, 0.0) * dt_s
        target_delta = self._desired_target_xyz - self._command_target_xyz
        self._command_target_xyz = self._command_target_xyz + _clamp_vector_norm(target_delta, max_linear_step)
        if self.fixed_z_enabled:
            self._command_target_xyz[2] = self.fixed_z
        self._command_target_xyz = self._clamp_target(self._command_target_xyz)

        pitch_error = _wrap_to_pi(self._desired_pitch_target_rad - self._command_pitch_target_rad)
        max_pitch_step = max(self.max_target_pitch_rate, 0.0) * dt_s
        if max_pitch_step <= 0.0 or abs(pitch_error) <= max_pitch_step:
            self._command_pitch_target_rad = self._desired_pitch_target_rad
        else:
            self._command_pitch_target_rad = _wrap_to_pi(
                self._command_pitch_target_rad + math.copysign(max_pitch_step, pitch_error)
            )

    def _publish_joint_command(self, q_cmd: np.ndarray) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.kin.JOINT_NAMES)
        msg.position = [float(value) for value in q_cmd]
        self.command_pub.publish(msg)

    def _control_step(self) -> None:
        if self._current_q is None:
            self._publish_bool(False)
            return

        current_pose = self._current_pose(self._current_q)
        self._publish_point(self.current_pose_pub, current_pose)

        if not self._maybe_initialize_target():
            return

        dt_s = 1.0 / self.control_rate_hz
        self._integrate_delta_cmd(dt_s)
        self._slew_command_targets(dt_s)
        assert self._desired_target_xyz is not None
        assert self._command_target_xyz is not None
        self._desired_target_xyz = self._clamp_target(self._desired_target_xyz)
        self._command_target_xyz = self._clamp_target(self._command_target_xyz)
        self._publish_point(self.target_pose_pub, self._desired_target_xyz)
        self._publish_point(self.command_pose_pub, self._command_target_xyz)
        current_pitch = self._pitch_measure(self._current_q)
        self._publish_pitch(self.current_pitch_pub, current_pitch)
        self._publish_pitch(self.target_pitch_pub, self._desired_pitch_target_rad)
        self._publish_pitch(self.command_pitch_pub, self._command_pitch_target_rad)

        position_error = np.linalg.norm(self._command_target_xyz - current_pose)
        pitch_error = abs(_wrap_to_pi(current_pitch - self._command_pitch_target_rad))
        if position_error <= self.position_tolerance_m and pitch_error <= self.pitch_tolerance_rad:
            self._publish_bool(True)
            self._last_command_q = self._full_q_from_reduced(
                self._current_q[:3], float(self._current_q[3]), self._command_pitch_target_rad
            )
            self._publish_joint_command(self._last_command_q)
            return

        self._publish_bool(True)
        q_prev = self._last_command_q.copy() if self._last_command_q is not None else self._current_q.copy()
        q_cmd = self._compute_servo_command(
            self._command_target_xyz, self._command_pitch_target_rad, self._current_q, q_prev
        )
        self._last_command_q = q_cmd
        self._publish_joint_command(q_cmd)


def main() -> None:
    rclpy.init()
    node = PlanarTaskControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
