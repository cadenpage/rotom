from typing import Optional

from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .math_utils import (
    clamp_vector,
    pose_xyzrpy_to_matrix,
    rotation_matrix_to_rotvec,
    transform_msg_to_matrix,
)


class StaleTransformError(Exception):
    pass


class MarkerFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("marker_follower")

        self.declare_parameter("robot_marker_frame", "aruco_marker_1")
        self.declare_parameter("object_marker_frame", "aruco_marker_2")
        self.declare_parameter("robot_marker_to_ee_xyzrpy", [-0.004, 0.0, -0.02, 0.0, 0.0, 0.0])
        self.declare_parameter("object_marker_to_ee_target_xyzrpy", [0.02, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("twist_topic", "/rotom_servo/cartesian_cmd")
        self.declare_parameter("target_pose_topic", "/rotom_vision/ee_target_pose")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("linear_gain", 1.2)
        self.declare_parameter("angular_gain", 2.0)
        self.declare_parameter("max_linear_speed", 0.10)
        self.declare_parameter("max_angular_speed", 0.60)
        self.declare_parameter("position_tolerance_m", 0.005)
        self.declare_parameter("angle_tolerance_rad", 0.05)
        self.declare_parameter("command_smoothing_alpha", 1.0)
        self.declare_parameter("max_transform_age_s", 0.25)
        self.declare_parameter("publish_zero_when_lost", True)
        self.declare_parameter("enable_output", False)

        self.robot_marker_frame = str(self.get_parameter("robot_marker_frame").value)
        self.object_marker_frame = str(self.get_parameter("object_marker_frame").value)
        self.robot_marker_to_ee = pose_xyzrpy_to_matrix(self.get_parameter("robot_marker_to_ee_xyzrpy").value)
        self.object_marker_to_ee_target = pose_xyzrpy_to_matrix(
            self.get_parameter("object_marker_to_ee_target_xyzrpy").value
        )
        self.twist_topic = str(self.get_parameter("twist_topic").value)
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.position_tolerance_m = float(self.get_parameter("position_tolerance_m").value)
        self.angle_tolerance_rad = float(self.get_parameter("angle_tolerance_rad").value)
        self.command_smoothing_alpha = float(self.get_parameter("command_smoothing_alpha").value)
        self.max_transform_age_s = float(self.get_parameter("max_transform_age_s").value)
        self.publish_zero_when_lost = bool(self.get_parameter("publish_zero_when_lost").value)
        self.enable_output = bool(self.get_parameter("enable_output").value)

        if self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be > 0.0")
        if not 0.0 < self.command_smoothing_alpha <= 1.0:
            raise ValueError("command_smoothing_alpha must be in (0.0, 1.0]")
        if self.max_transform_age_s < 0.0:
            raise ValueError("max_transform_age_s must be >= 0.0")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.twist_pub = self.create_publisher(Twist, self.twist_topic, 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, self.target_pose_topic, 10)

        self._smoothed_linear_cmd = np.zeros(3, dtype=np.float64)
        self._smoothed_angular_cmd = np.zeros(3, dtype=np.float64)
        self._last_tf_warn_ns = 0
        self._last_command_log_ns = 0
        self._warn_period_ns = 2_000_000_000

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self._control_step)

        self.get_logger().info(
            f"marker_follower active. robot_marker={self.robot_marker_frame}, "
            f"object_marker={self.object_marker_frame}, output_enabled={self.enable_output}"
        )

    def _lookup_robot_to_object(self) -> np.ndarray:
        transform = self.tf_buffer.lookup_transform(
            self.robot_marker_frame,
            self.object_marker_frame,
            Time(),
        )
        if self.max_transform_age_s > 0.0:
            transform_time_ns = Time.from_msg(transform.header.stamp).nanoseconds
            age_s = (self.get_clock().now().nanoseconds - transform_time_ns) * 1e-9
            if age_s > self.max_transform_age_s:
                raise StaleTransformError(
                    f"marker transform is stale ({age_s:.3f}s old, limit {self.max_transform_age_s:.3f}s)"
                )
        return transform_msg_to_matrix(transform.transform)

    def _compute_command(self, transform_robot_to_object: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        transform_robot_to_ee_desired = transform_robot_to_object @ self.object_marker_to_ee_target
        transform_ee_to_ee_desired = np.linalg.inv(self.robot_marker_to_ee) @ transform_robot_to_ee_desired

        pos_error = transform_ee_to_ee_desired[:3, 3]
        rot_error = rotation_matrix_to_rotvec(transform_ee_to_ee_desired[:3, :3])

        linear_cmd = np.zeros(3, dtype=np.float64)
        angular_cmd = np.zeros(3, dtype=np.float64)

        if float(np.linalg.norm(pos_error)) > self.position_tolerance_m:
            linear_cmd = clamp_vector(self.linear_gain * pos_error, self.max_linear_speed)
        if float(np.linalg.norm(rot_error)) > self.angle_tolerance_rad:
            angular_cmd = clamp_vector(self.angular_gain * rot_error, self.max_angular_speed)

        return transform_robot_to_ee_desired[:3, 3], linear_cmd, angular_cmd

    def _smooth_command(self, linear_cmd: np.ndarray, angular_cmd: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        alpha = self.command_smoothing_alpha
        if alpha >= 1.0:
            self._smoothed_linear_cmd = linear_cmd.copy()
            self._smoothed_angular_cmd = angular_cmd.copy()
        else:
            self._smoothed_linear_cmd = alpha * linear_cmd + (1.0 - alpha) * self._smoothed_linear_cmd
            self._smoothed_angular_cmd = alpha * angular_cmd + (1.0 - alpha) * self._smoothed_angular_cmd
        return self._smoothed_linear_cmd.copy(), self._smoothed_angular_cmd.copy()

    def _publish_target_pose(self, frame_id: str, translation: np.ndarray) -> None:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = float(translation[0])
        pose_msg.pose.position.y = float(translation[1])
        pose_msg.pose.position.z = float(translation[2])
        pose_msg.pose.orientation.w = 1.0
        self.target_pose_pub.publish(pose_msg)

    def _publish_twist(self, linear: np.ndarray, angular: np.ndarray) -> None:
        if not self.enable_output:
            return

        twist_msg = Twist()
        twist_msg.linear.x = float(linear[0])
        twist_msg.linear.y = float(linear[1])
        twist_msg.linear.z = float(linear[2])
        twist_msg.angular.x = float(angular[0])
        twist_msg.angular.y = float(angular[1])
        twist_msg.angular.z = float(angular[2])
        self.twist_pub.publish(twist_msg)

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_command_log_ns > self._warn_period_ns:
            lin_norm = float(np.linalg.norm(linear))
            ang_norm = float(np.linalg.norm(angular))
            if lin_norm > 1e-4 or ang_norm > 1e-4:
                self.get_logger().info(f"publishing cartesian cmd: lin={lin_norm:.3f}, ang={ang_norm:.3f}")
                self._last_command_log_ns = now_ns

    def _publish_zero(self) -> None:
        self._smoothed_linear_cmd.fill(0.0)
        self._smoothed_angular_cmd.fill(0.0)
        self._publish_twist(np.zeros(3), np.zeros(3))

    def _control_step(self) -> None:
        try:
            transform_robot_to_object = self._lookup_robot_to_object()
        except (TransformException, StaleTransformError) as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns > self._warn_period_ns:
                self.get_logger().warn(f"waiting for marker transforms: {exc}")
                self._last_tf_warn_ns = now_ns
            if self.publish_zero_when_lost:
                self._publish_zero()
            return

        target_translation, linear_cmd, angular_cmd = self._compute_command(transform_robot_to_object)
        linear_cmd, angular_cmd = self._smooth_command(linear_cmd, angular_cmd)
        self._publish_target_pose(self.robot_marker_frame, target_translation)
        self._publish_twist(linear_cmd, angular_cmd)


def main() -> None:
    rclpy.init()
    node = MarkerFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
