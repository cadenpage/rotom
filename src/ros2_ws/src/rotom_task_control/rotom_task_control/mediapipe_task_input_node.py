import json
import math
import socket
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3
from rclpy.node import Node
from std_msgs.msg import Bool


def _clamp_vector(vec: tuple[float, float, float], max_magnitude: float) -> tuple[float, float, float]:
    magnitude = math.sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2])
    if max_magnitude <= 0.0 or magnitude <= max_magnitude or magnitude <= 1e-12:
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


class MediaPipeTaskInputNode(Node):
    def __init__(self) -> None:
        super().__init__("mediapipe_task_input")

        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("bind_port", 8765)
        self.declare_parameter("delta_topic", "/rotom_task/delta_cmd")
        self.declare_parameter("raw_pose_topic", "/rotom_task/raw_hand_pose")
        self.declare_parameter("enabled_topic", "/rotom_task/enabled")
        self.declare_parameter("frame_id", "teleop_hand")
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("packet_timeout_s", 0.25)
        self.declare_parameter("min_confidence", 0.5)
        self.declare_parameter("require_clutch", True)
        self.declare_parameter("reset_origin_on_clutch_release", True)
        self.declare_parameter("deadband", 0.015)
        self.declare_parameter("smoothing_alpha", 0.35)
        self.declare_parameter("max_delta_speed", 0.05)
        self.declare_parameter("axis_map", [0, 1, 2])
        self.declare_parameter("axis_signs", [-1.0, -1.0, 0.0])
        self.declare_parameter("linear_scale_xyz", [0.04, 0.04, 0.00])
        self.declare_parameter("publish_zero_when_stale", True)
        self.declare_parameter("warn_period_s", 2.0)

        self.bind_host = str(self.get_parameter("bind_host").value)
        self.bind_port = int(self.get_parameter("bind_port").value)
        self.delta_topic = str(self.get_parameter("delta_topic").value)
        self.raw_pose_topic = str(self.get_parameter("raw_pose_topic").value)
        self.enabled_topic = str(self.get_parameter("enabled_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.packet_timeout_s = float(self.get_parameter("packet_timeout_s").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.require_clutch = bool(self.get_parameter("require_clutch").value)
        self.reset_origin_on_clutch_release = bool(self.get_parameter("reset_origin_on_clutch_release").value)
        self.deadband = float(self.get_parameter("deadband").value)
        self.smoothing_alpha = float(self.get_parameter("smoothing_alpha").value)
        self.max_delta_speed = float(self.get_parameter("max_delta_speed").value)
        self.axis_map = tuple(int(x) for x in self.get_parameter("axis_map").value)
        self.axis_signs = tuple(float(x) for x in self.get_parameter("axis_signs").value)
        self.linear_scale_xyz = tuple(float(x) for x in self.get_parameter("linear_scale_xyz").value)
        self.publish_zero_when_stale = bool(self.get_parameter("publish_zero_when_stale").value)
        self.warn_period_ns = int(float(self.get_parameter("warn_period_s").value) * 1e9)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.bind_host, self.bind_port))
        self._sock.setblocking(False)

        self.delta_pub = self.create_publisher(Vector3, self.delta_topic, 10)
        self.raw_pose_pub = self.create_publisher(PoseStamped, self.raw_pose_topic, 10)
        self.enabled_pub = self.create_publisher(Bool, self.enabled_topic, 10)

        self._latest_seq = -1
        self._latest_packet_time_ns: Optional[int] = None
        self._latest_hand_xyz: Optional[tuple[float, float, float]] = None
        self._latest_clutch = False
        self._latest_confidence = 0.0
        self._latest_tracked = False
        self._origin_hand_xyz: Optional[tuple[float, float, float]] = None
        self._smoothed_delta = (0.0, 0.0, 0.0)
        self._last_warn_ns = 0

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self._control_step)
        self.get_logger().info(
            f"mediapipe_task_input active. udp={self.bind_host}:{self.bind_port}, delta_topic={self.delta_topic}"
        )

    def destroy_node(self) -> bool:
        self._sock.close()
        return super().destroy_node()

    def _warn(self, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_warn_ns > self.warn_period_ns:
            self.get_logger().warn(message)
            self._last_warn_ns = now_ns

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

    def _publish_delta(self, linear: tuple[float, float, float]) -> None:
        msg = Vector3()
        msg.x = linear[0]
        msg.y = linear[1]
        msg.z = linear[2]
        self.delta_pub.publish(msg)

    def _drain_socket(self) -> None:
        while True:
            try:
                payload, _ = self._sock.recvfrom(4096)
            except BlockingIOError:
                return

            try:
                packet = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue

            if not isinstance(packet, dict):
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

    def _packet_is_fresh(self) -> bool:
        if self._latest_packet_time_ns is None:
            return False
        age_s = (self.get_clock().now().nanoseconds - self._latest_packet_time_ns) * 1e-9
        return age_s <= self.packet_timeout_s

    def _map_hand_delta(self, hand_delta: tuple[float, float, float]) -> tuple[float, float, float]:
        mapped = []
        for axis in range(3):
            hand_axis = self.axis_map[axis]
            value = self.axis_signs[axis] * hand_delta[hand_axis]
            value = _apply_deadband(value, self.deadband)
            mapped.append(value * self.linear_scale_xyz[axis])
        return tuple(mapped)

    def _control_step(self) -> None:
        self._drain_socket()

        if not self._packet_is_fresh() or self._latest_hand_xyz is None:
            self._publish_enabled(False)
            self._origin_hand_xyz = None
            self._smoothed_delta = (0.0, 0.0, 0.0)
            if self.publish_zero_when_stale:
                self._publish_delta((0.0, 0.0, 0.0))
            self._warn("waiting for fresh MediaPipe task-input packets")
            return

        self._publish_raw_pose(self._latest_hand_xyz)
        valid_tracking = self._latest_tracked and self._latest_confidence >= self.min_confidence
        clutch_active = self._latest_clutch or not self.require_clutch

        if not valid_tracking:
            self._publish_enabled(False)
            self._origin_hand_xyz = None
            self._smoothed_delta = (0.0, 0.0, 0.0)
            if self.publish_zero_when_stale:
                self._publish_delta((0.0, 0.0, 0.0))
            if not self._latest_tracked:
                self._warn(
                    f"task-input packets report no tracked hand (confidence={self._latest_confidence:.2f}, clutch={self._latest_clutch})"
                )
            else:
                self._warn(
                    f"task-input tracking below confidence threshold ({self._latest_confidence:.2f} < {self.min_confidence:.2f})"
                )
            return

        if not clutch_active:
            self._publish_enabled(False)
            self._smoothed_delta = (0.0, 0.0, 0.0)
            if self.publish_zero_when_stale:
                self._publish_delta((0.0, 0.0, 0.0))
            if self.reset_origin_on_clutch_release:
                self._origin_hand_xyz = None
            return

        self._publish_enabled(True)

        if self._origin_hand_xyz is None:
            self._origin_hand_xyz = self._latest_hand_xyz
            self._smoothed_delta = (0.0, 0.0, 0.0)
            self._publish_delta((0.0, 0.0, 0.0))
            return

        hand_delta = (
            self._latest_hand_xyz[0] - self._origin_hand_xyz[0],
            self._latest_hand_xyz[1] - self._origin_hand_xyz[1],
            self._latest_hand_xyz[2] - self._origin_hand_xyz[2],
        )
        raw_delta = _clamp_vector(self._map_hand_delta(hand_delta), self.max_delta_speed)
        self._smoothed_delta = _clamp_vector(
            _smooth_vector(self._smoothed_delta, raw_delta, self.smoothing_alpha),
            self.max_delta_speed,
        )
        self._publish_delta(self._smoothed_delta)


def main() -> None:
    rclpy.init()
    node = MediaPipeTaskInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
