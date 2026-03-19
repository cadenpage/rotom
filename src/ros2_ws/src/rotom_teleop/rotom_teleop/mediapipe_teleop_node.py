import json
import math
import socket
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


def _vector_norm(vec: tuple[float, float, float]) -> float:
    return math.sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2])


def _normalize_vector(vec: tuple[float, float, float]) -> tuple[float, float, float]:
    magnitude = _vector_norm(vec)
    if magnitude <= 1e-12:
        return (0.0, 0.0, 0.0)
    return (vec[0] / magnitude, vec[1] / magnitude, vec[2] / magnitude)


def _clamp_vector(vec: tuple[float, float, float], max_magnitude: float) -> tuple[float, float, float]:
    if max_magnitude <= 0.0:
        return (0.0, 0.0, 0.0)

    magnitude = _vector_norm(vec)
    if magnitude <= max_magnitude or magnitude <= 1e-12:
        return vec

    scale = max_magnitude / magnitude
    return (vec[0] * scale, vec[1] * scale, vec[2] * scale)


def _apply_deadband(value: float, deadband: float) -> float:
    if abs(value) <= deadband:
        return 0.0
    return math.copysign(abs(value) - deadband, value)


def _smooth_vector(
    previous: tuple[float, float, float], current: tuple[float, float, float], alpha: float
) -> tuple[float, float, float]:
    if alpha >= 1.0:
        return current
    return (
        alpha * current[0] + (1.0 - alpha) * previous[0],
        alpha * current[1] + (1.0 - alpha) * previous[1],
        alpha * current[2] + (1.0 - alpha) * previous[2],
    )


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _quat_rotate_vector(
    quat_xyzw: tuple[float, float, float, float], vec: tuple[float, float, float]
) -> tuple[float, float, float]:
    qx, qy, qz, qw = quat_xyzw
    q_vec = (qx, qy, qz)
    t = _cross(q_vec, vec)
    t = (2.0 * t[0], 2.0 * t[1], 2.0 * t[2])
    c = _cross(q_vec, t)
    return (
        vec[0] + qw * t[0] + c[0],
        vec[1] + qw * t[1] + c[1],
        vec[2] + qw * t[2] + c[2],
    )


class MediaPipeTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("mediapipe_teleop")

        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("bind_port", 8765)
        self.declare_parameter("twist_topic", "/rotom_servo/cartesian_cmd")
        self.declare_parameter("raw_pose_topic", "/rotom_teleop/raw_hand_pose")
        self.declare_parameter("enabled_topic", "/rotom_teleop/enabled")
        self.declare_parameter("raw_cmd_topic", "/rotom_teleop/cmd_raw")
        self.declare_parameter("frame_id", "teleop_hand")
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("packet_timeout_s", 0.25)
        self.declare_parameter("min_confidence", 0.5)
        self.declare_parameter("require_clutch", True)
        self.declare_parameter("reset_origin_on_clutch_release", True)
        self.declare_parameter("deadband", 0.015)
        self.declare_parameter("smoothing_alpha", 0.35)
        self.declare_parameter("max_linear_speed", 0.12)
        self.declare_parameter("max_angular_speed", 0.50)
        self.declare_parameter("linear_scale_xyz", [0.50, 0.45, 0.45])
        self.declare_parameter("axis_map", [2, 0, 1])
        self.declare_parameter("axis_signs", [-1.0, -1.0, -1.0])
        self.declare_parameter("table_slide_mode", False)
        self.declare_parameter("table_slide_axis_map_xy", [1, 0])
        self.declare_parameter("table_slide_axis_signs_xy", [-1.0, -1.0])
        self.declare_parameter("table_slide_scale_xy", [0.30, 0.30])
        self.declare_parameter("table_slide_world_frame", "ground")
        self.declare_parameter("table_slide_frame", "tool0")
        self.declare_parameter("table_slide_hold_orientation", True)
        self.declare_parameter("table_slide_tool_axis_local", [0.0, 1.0, 0.0])
        self.declare_parameter("table_slide_target_axis_world", [0.0, 0.0, -1.0])
        self.declare_parameter("table_slide_orientation_gain", 1.5)
        self.declare_parameter("publish_zero_when_stale", True)
        self.declare_parameter("warn_period_s", 2.0)

        self.bind_host = str(self.get_parameter("bind_host").value)
        self.bind_port = int(self.get_parameter("bind_port").value)
        self.twist_topic = str(self.get_parameter("twist_topic").value)
        self.raw_pose_topic = str(self.get_parameter("raw_pose_topic").value)
        self.enabled_topic = str(self.get_parameter("enabled_topic").value)
        self.raw_cmd_topic = str(self.get_parameter("raw_cmd_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.packet_timeout_s = float(self.get_parameter("packet_timeout_s").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.require_clutch = bool(self.get_parameter("require_clutch").value)
        self.reset_origin_on_clutch_release = bool(self.get_parameter("reset_origin_on_clutch_release").value)
        self.deadband = float(self.get_parameter("deadband").value)
        self.smoothing_alpha = float(self.get_parameter("smoothing_alpha").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.linear_scale_xyz = tuple(float(x) for x in self.get_parameter("linear_scale_xyz").value)
        self.axis_map = tuple(int(x) for x in self.get_parameter("axis_map").value)
        self.axis_signs = tuple(float(x) for x in self.get_parameter("axis_signs").value)
        self.table_slide_mode = bool(self.get_parameter("table_slide_mode").value)
        self.table_slide_axis_map_xy = tuple(int(x) for x in self.get_parameter("table_slide_axis_map_xy").value)
        self.table_slide_axis_signs_xy = tuple(float(x) for x in self.get_parameter("table_slide_axis_signs_xy").value)
        self.table_slide_scale_xy = tuple(float(x) for x in self.get_parameter("table_slide_scale_xy").value)
        self.table_slide_world_frame = str(self.get_parameter("table_slide_world_frame").value)
        self.table_slide_frame = str(self.get_parameter("table_slide_frame").value)
        self.table_slide_hold_orientation = bool(self.get_parameter("table_slide_hold_orientation").value)
        self.table_slide_tool_axis_local = tuple(float(x) for x in self.get_parameter("table_slide_tool_axis_local").value)
        self.table_slide_target_axis_world = tuple(float(x) for x in self.get_parameter("table_slide_target_axis_world").value)
        self.table_slide_orientation_gain = float(self.get_parameter("table_slide_orientation_gain").value)
        self.publish_zero_when_stale = bool(self.get_parameter("publish_zero_when_stale").value)
        self.warn_period_ns = int(float(self.get_parameter("warn_period_s").value) * 1e9)

        if self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be > 0.0")
        if self.packet_timeout_s < 0.0:
            raise ValueError("packet_timeout_s must be >= 0.0")
        if not 0.0 < self.smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in (0.0, 1.0]")
        if len(self.linear_scale_xyz) != 3 or len(self.axis_map) != 3 or len(self.axis_signs) != 3:
            raise ValueError("linear_scale_xyz, axis_map, and axis_signs must all have length 3")
        if len(self.table_slide_axis_map_xy) != 2 or len(self.table_slide_axis_signs_xy) != 2 or len(self.table_slide_scale_xy) != 2:
            raise ValueError("table_slide_axis_map_xy, table_slide_axis_signs_xy, and table_slide_scale_xy must all have length 2")
        if len(self.table_slide_tool_axis_local) != 3 or len(self.table_slide_target_axis_world) != 3:
            raise ValueError("table_slide_tool_axis_local and table_slide_target_axis_world must both have length 3")
        if any(axis < 0 or axis > 2 for axis in self.axis_map):
            raise ValueError("axis_map values must be in [0, 2]")
        if any(axis < 0 or axis > 2 for axis in self.table_slide_axis_map_xy):
            raise ValueError("table_slide_axis_map_xy values must be in [0, 2]")

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.bind_host, self.bind_port))
        self._sock.setblocking(False)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        self.twist_pub = self.create_publisher(Twist, self.twist_topic, 10)
        self.raw_cmd_pub = self.create_publisher(Twist, self.raw_cmd_topic, 10)
        self.raw_pose_pub = self.create_publisher(PoseStamped, self.raw_pose_topic, 10)
        self.enabled_pub = self.create_publisher(Bool, self.enabled_topic, 10)

        self._latest_seq = -1
        self._latest_packet_time_ns: Optional[int] = None
        self._latest_hand_xyz: Optional[tuple[float, float, float]] = None
        self._latest_clutch = False
        self._latest_confidence = 0.0
        self._latest_tracked = False
        self._origin_hand_xyz: Optional[tuple[float, float, float]] = None
        self._smoothed_linear_cmd = (0.0, 0.0, 0.0)
        self._smoothed_angular_cmd = (0.0, 0.0, 0.0)
        self._last_warn_ns = 0
        self._last_status_log_ns = 0

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self._control_step)

        self.get_logger().info(
            "mediapipe_teleop active. "
            f"udp={self.bind_host}:{self.bind_port}, twist_topic={self.twist_topic}, frame={self.frame_id}"
        )
        if self.table_slide_mode:
            self.get_logger().info(
                "table_slide_mode enabled. "
                f"radial<-hand[{self.table_slide_axis_map_xy[0]}], "
                f"tangential<-hand[{self.table_slide_axis_map_xy[1]}], "
                f"hold_axis={self.table_slide_tool_axis_local} -> {self.table_slide_target_axis_world}"
            )

    def destroy_node(self) -> bool:
        self._sock.close()
        return super().destroy_node()

    def _drain_socket(self) -> None:
        while True:
            try:
                payload, _addr = self._sock.recvfrom(4096)
            except BlockingIOError:
                return

            try:
                packet = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                self.get_logger().warn(f"dropping malformed teleop packet: {exc}")
                continue

            if not isinstance(packet, dict):
                self.get_logger().warn("dropping non-dict teleop packet")
                continue

            seq = int(packet.get("seq", self._latest_seq + 1))
            if seq <= self._latest_seq:
                continue

            hand_xyz_raw = packet.get("hand_xyz")
            if not isinstance(hand_xyz_raw, list) or len(hand_xyz_raw) != 3:
                continue

            try:
                hand_xyz = (float(hand_xyz_raw[0]), float(hand_xyz_raw[1]), float(hand_xyz_raw[2]))
            except (TypeError, ValueError):
                continue

            self._latest_seq = seq
            self._latest_packet_time_ns = self.get_clock().now().nanoseconds
            self._latest_hand_xyz = hand_xyz
            self._latest_clutch = bool(packet.get("clutch", False))
            self._latest_confidence = float(packet.get("confidence", 0.0))
            self._latest_tracked = bool(packet.get("tracked", True))

    def _publish_enabled(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = enabled
        self.enabled_pub.publish(msg)

    def _publish_raw_pose(self, hand_xyz: tuple[float, float, float]) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = hand_xyz[0]
        msg.pose.position.y = hand_xyz[1]
        msg.pose.position.z = hand_xyz[2]
        msg.pose.orientation.w = 1.0
        self.raw_pose_pub.publish(msg)

    def _publish_twist(
        self,
        publisher,
        linear: tuple[float, float, float],
        angular: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        msg = Twist()
        msg.linear.x = linear[0]
        msg.linear.y = linear[1]
        msg.linear.z = linear[2]
        msg.angular.x = angular[0]
        msg.angular.y = angular[1]
        msg.angular.z = angular[2]
        publisher.publish(msg)

    def _publish_zero(self) -> None:
        self._smoothed_linear_cmd = (0.0, 0.0, 0.0)
        self._smoothed_angular_cmd = (0.0, 0.0, 0.0)
        self._publish_twist(self.raw_cmd_pub, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self._publish_twist(self.twist_pub, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    def _packet_is_fresh(self) -> bool:
        if self._latest_packet_time_ns is None:
            return False
        age_s = (self.get_clock().now().nanoseconds - self._latest_packet_time_ns) * 1e-9
        return age_s <= self.packet_timeout_s

    def _warn_stale(self, reason: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_warn_ns > self.warn_period_ns:
            self.get_logger().warn(reason)
            self._last_warn_ns = now_ns

    def _map_hand_delta_to_robot(self, hand_delta: tuple[float, float, float]) -> tuple[float, float, float]:
        mapped = []
        for robot_axis in range(3):
            hand_axis = self.axis_map[robot_axis]
            value = self.axis_signs[robot_axis] * hand_delta[hand_axis]
            value = _apply_deadband(value, self.deadband)
            mapped.append(value * self.linear_scale_xyz[robot_axis])
        return tuple(mapped)

    def _lookup_slide_frame_transform(
        self,
    ) -> Optional[tuple[tuple[float, float, float], tuple[float, float, float, float]]]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.table_slide_world_frame, self.table_slide_frame, Time()
            )
        except TransformException as exc:
            self._warn_stale(
                f"waiting for transform {self.table_slide_world_frame}->{self.table_slide_frame}: {exc}"
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _compute_table_slide_cmd(
        self, hand_delta: tuple[float, float, float]
    ) -> Optional[tuple[tuple[float, float, float], tuple[float, float, float]]]:
        radial_input = self.table_slide_axis_signs_xy[0] * hand_delta[self.table_slide_axis_map_xy[0]]
        tangential_input = self.table_slide_axis_signs_xy[1] * hand_delta[self.table_slide_axis_map_xy[1]]
        radial_input = _apply_deadband(radial_input, self.deadband) * self.table_slide_scale_xy[0]
        tangential_input = _apply_deadband(tangential_input, self.deadband) * self.table_slide_scale_xy[1]

        transform = self._lookup_slide_frame_transform()
        if transform is None:
            return None

        position_world, quat_world_slide = transform
        radial_basis = _normalize_vector((position_world[0], position_world[1], 0.0))
        if _vector_norm(radial_basis) <= 1e-9:
            self._warn_stale("table_slide_mode: slide frame is too close to the world origin to define a radial axis")
            return None

        tangential_basis = (-radial_basis[1], radial_basis[0], 0.0)
        linear_cmd = (
            radial_input * radial_basis[0] + tangential_input * tangential_basis[0],
            radial_input * radial_basis[1] + tangential_input * tangential_basis[1],
            0.0,
        )
        linear_cmd = _clamp_vector(linear_cmd, self.max_linear_speed)

        angular_cmd = (0.0, 0.0, 0.0)
        if self.table_slide_hold_orientation:
            tool_axis_world = _normalize_vector(_quat_rotate_vector(quat_world_slide, self.table_slide_tool_axis_local))
            target_axis_world = _normalize_vector(self.table_slide_target_axis_world)
            angular_cmd = _cross(tool_axis_world, target_axis_world)
            angular_cmd = (
                self.table_slide_orientation_gain * angular_cmd[0],
                self.table_slide_orientation_gain * angular_cmd[1],
                0.0,
            )
            angular_cmd = _clamp_vector(angular_cmd, self.max_angular_speed)

        return linear_cmd, angular_cmd

    def _control_step(self) -> None:
        self._drain_socket()

        if not self._packet_is_fresh() or self._latest_hand_xyz is None:
            self._publish_enabled(False)
            if self.publish_zero_when_stale:
                self._publish_zero()
            self._origin_hand_xyz = None
            self._warn_stale("waiting for fresh MediaPipe teleop packets")
            return

        self._publish_raw_pose(self._latest_hand_xyz)

        valid_tracking = self._latest_tracked and self._latest_confidence >= self.min_confidence
        clutch_active = self._latest_clutch or not self.require_clutch

        if not valid_tracking:
            self._publish_enabled(False)
            if self.publish_zero_when_stale:
                self._publish_zero()
            self._origin_hand_xyz = None
            self._warn_stale(
                f"teleop hand tracking below confidence threshold ({self._latest_confidence:.2f} < {self.min_confidence:.2f})"
            )
            return

        if not clutch_active:
            self._publish_enabled(False)
            if self.publish_zero_when_stale:
                self._publish_zero()
            if self.reset_origin_on_clutch_release:
                self._origin_hand_xyz = None
            return

        self._publish_enabled(True)

        if self._origin_hand_xyz is None:
            self._origin_hand_xyz = self._latest_hand_xyz
            self._publish_zero()
            return

        hand_delta = (
            self._latest_hand_xyz[0] - self._origin_hand_xyz[0],
            self._latest_hand_xyz[1] - self._origin_hand_xyz[1],
            self._latest_hand_xyz[2] - self._origin_hand_xyz[2],
        )
        raw_linear_cmd = (0.0, 0.0, 0.0)
        raw_angular_cmd = (0.0, 0.0, 0.0)
        if self.table_slide_mode:
            table_slide_cmd = self._compute_table_slide_cmd(hand_delta)
            if table_slide_cmd is None:
                self._publish_enabled(False)
                if self.publish_zero_when_stale:
                    self._publish_zero()
                return
            raw_linear_cmd, raw_angular_cmd = table_slide_cmd
        else:
            raw_linear_cmd = _clamp_vector(self._map_hand_delta_to_robot(hand_delta), self.max_linear_speed)

        self._publish_twist(self.raw_cmd_pub, raw_linear_cmd, raw_angular_cmd)

        self._smoothed_linear_cmd = _clamp_vector(
            _smooth_vector(self._smoothed_linear_cmd, raw_linear_cmd, self.smoothing_alpha),
            self.max_linear_speed,
        )
        self._smoothed_angular_cmd = _clamp_vector(
            _smooth_vector(self._smoothed_angular_cmd, raw_angular_cmd, self.smoothing_alpha),
            self.max_angular_speed,
        )
        self._publish_twist(self.twist_pub, self._smoothed_linear_cmd, self._smoothed_angular_cmd)

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_status_log_ns > self.warn_period_ns:
            self.get_logger().info(
                "teleop cmd active: "
                f"lin={_vector_norm(self._smoothed_linear_cmd):.3f}, "
                f"ang={_vector_norm(self._smoothed_angular_cmd):.3f}, "
                f"confidence={self._latest_confidence:.2f}, clutch={self._latest_clutch}"
            )
            self._last_status_log_ns = now_ns


def main() -> None:
    rclpy.init()
    node = MediaPipeTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
