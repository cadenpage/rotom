import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from rclpy.node import Node


def _vector_norm(vec: Vector3) -> float:
    return math.sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)


def _clamp_vector_magnitude(vec: Vector3, max_magnitude: float) -> Vector3:
    if max_magnitude <= 0.0:
        return Vector3()

    magnitude = _vector_norm(vec)
    if magnitude <= max_magnitude or magnitude <= 1e-12:
        return vec

    scale = max_magnitude / magnitude
    clamped = Vector3()
    clamped.x = vec.x * scale
    clamped.y = vec.y * scale
    clamped.z = vec.z * scale
    return clamped


class TwistRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("twist_relay")

        self.declare_parameter("input_topic", "/rotom_servo/cartesian_cmd")
        self.declare_parameter("output_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("command_frame", "tool0")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("command_timeout_s", 0.20)
        self.declare_parameter("max_linear_speed", 0.22)
        self.declare_parameter("max_angular_speed", 0.60)
        self.declare_parameter("deadband", 1e-4)
        self.declare_parameter("publish_zero_when_stale", True)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.command_frame = str(self.get_parameter("command_frame").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.deadband = float(self.get_parameter("deadband").value)
        self.publish_zero_when_stale = bool(self.get_parameter("publish_zero_when_stale").value)

        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0.0")
        if self.command_timeout_s < 0.0:
            raise ValueError("command_timeout_s must be >= 0.0")
        if self.deadband < 0.0:
            raise ValueError("deadband must be >= 0.0")

        self._latest_cmd = Twist()
        self._latest_cmd_time_ns: Optional[int] = None
        self._published_zero_since_stale = False

        self.cmd_pub = self.create_publisher(TwistStamped, self.output_topic, 10)
        self.create_subscription(Twist, self.input_topic, self._twist_cb, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_step)

        self.get_logger().info(
            f"twist_relay active. input={self.input_topic}, output={self.output_topic}, frame={self.command_frame}"
        )

    def _twist_cb(self, msg: Twist) -> None:
        self._latest_cmd = msg
        self._latest_cmd_time_ns = self.get_clock().now().nanoseconds
        self._published_zero_since_stale = False

    def _clamp_twist(self, twist: Twist) -> Twist:
        clamped = Twist()
        clamped.linear = _clamp_vector_magnitude(twist.linear, self.max_linear_speed)
        clamped.angular = _clamp_vector_magnitude(twist.angular, self.max_angular_speed)

        if _vector_norm(clamped.linear) < self.deadband:
            clamped.linear = Vector3()
        if _vector_norm(clamped.angular) < self.deadband:
            clamped.angular = Vector3()
        return clamped

    def _publish_twist(self, twist: Twist) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame
        msg.twist = twist
        self.cmd_pub.publish(msg)

    def _publish_step(self) -> None:
        if self._latest_cmd_time_ns is None:
            return

        age_s = (self.get_clock().now().nanoseconds - self._latest_cmd_time_ns) * 1e-9
        if age_s <= self.command_timeout_s:
            self._publish_twist(self._clamp_twist(self._latest_cmd))
            return

        if self.publish_zero_when_stale and not self._published_zero_since_stale:
            self._publish_twist(Twist())
            self._published_zero_since_stale = True


def main() -> None:
    rclpy.init()
    node = TwistRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
