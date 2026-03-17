from typing import Optional

from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from .math_utils import (
    clamp_vector,
    matrix_to_quaternion,
    pose_xyzrpy_to_matrix,
    rotation_matrix_to_rotvec,
    transform_msg_to_matrix,
)


class MarkerFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("marker_follower")

        self.declare_parameter("robot_marker_frame", "aruco_marker_1")
        self.declare_parameter("object_marker_frame", "aruco_marker_2")
        self.declare_parameter("robot_marker_to_ee_xyzrpy", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("object_marker_to_ee_target_xyzrpy", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("ee_command_frame", "tool0")
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("target_pose_topic", "/rotom_vision/ee_target_pose")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("linear_gain", 1.2)
        self.declare_parameter("angular_gain", 2.0)
        self.declare_parameter("max_linear_speed", 0.15)
        self.declare_parameter("far_field_distance_m", 0.05)
        self.declare_parameter("far_linear_gain_multiplier", 1.0)
        self.declare_parameter("far_max_linear_speed", 0.15)
        self.declare_parameter("max_angular_speed", 0.8)
        self.declare_parameter("position_tolerance_m", 0.005)
        self.declare_parameter("angle_tolerance_rad", 0.05)
        self.declare_parameter("command_smoothing_alpha", 1.0)
        self.declare_parameter("max_transform_age_s", 0.25)
        self.declare_parameter("max_robot_marker_age_s", -1.0)
        self.declare_parameter("max_object_marker_age_s", -1.0)
        self.declare_parameter("allow_cached_object_target", True)
        self.declare_parameter("tracking_parent_frame", "")
        self.declare_parameter("publish_zero_when_lost", True)
        self.declare_parameter("enable_twist_output", False)

        self.robot_marker_frame = str(self.get_parameter("robot_marker_frame").value)
        self.object_marker_frame = str(self.get_parameter("object_marker_frame").value)
        self.robot_marker_to_ee = pose_xyzrpy_to_matrix(self.get_parameter("robot_marker_to_ee_xyzrpy").value)
        self.object_marker_to_ee_target = pose_xyzrpy_to_matrix(
            self.get_parameter("object_marker_to_ee_target_xyzrpy").value
        )
        self.ee_command_frame = str(self.get_parameter("ee_command_frame").value)
        self.twist_topic = str(self.get_parameter("twist_topic").value)
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.far_field_distance_m = float(self.get_parameter("far_field_distance_m").value)
        self.far_linear_gain_multiplier = float(self.get_parameter("far_linear_gain_multiplier").value)
        self.far_max_linear_speed = float(self.get_parameter("far_max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.position_tolerance_m = float(self.get_parameter("position_tolerance_m").value)
        self.angle_tolerance_rad = float(self.get_parameter("angle_tolerance_rad").value)
        self.command_smoothing_alpha = float(self.get_parameter("command_smoothing_alpha").value)
        self.max_transform_age_s = float(self.get_parameter("max_transform_age_s").value)
        self.max_robot_marker_age_s = float(self.get_parameter("max_robot_marker_age_s").value)
        self.max_object_marker_age_s = float(self.get_parameter("max_object_marker_age_s").value)
        self.allow_cached_object_target = bool(self.get_parameter("allow_cached_object_target").value)
        self.tracking_parent_frame = str(self.get_parameter("tracking_parent_frame").value)
        self.publish_zero_when_lost = bool(self.get_parameter("publish_zero_when_lost").value)
        self.enable_twist_output = bool(self.get_parameter("enable_twist_output").value)

        if self.control_rate_hz <= 0.0:
            raise ValueError(f"control_rate_hz must be > 0.0. Got {self.control_rate_hz}")
        if not 0.0 < self.command_smoothing_alpha <= 1.0:
            raise ValueError(
                f"command_smoothing_alpha must be in (0.0, 1.0]. Got {self.command_smoothing_alpha}"
            )
        if self.far_field_distance_m < self.position_tolerance_m:
            raise ValueError(
                "far_field_distance_m must be >= position_tolerance_m. "
                f"Got far_field_distance_m={self.far_field_distance_m}, "
                f"position_tolerance_m={self.position_tolerance_m}"
            )
        if self.far_linear_gain_multiplier < 1.0:
            raise ValueError(
                f"far_linear_gain_multiplier must be >= 1.0. Got {self.far_linear_gain_multiplier}"
            )
        if self.far_max_linear_speed < self.max_linear_speed:
            raise ValueError(
                f"far_max_linear_speed must be >= max_linear_speed. Got {self.far_max_linear_speed}"
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.twist_pub = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, self.target_pose_topic, 10)

        self._last_tf_warn_ns = 0
        self._last_stale_tf_warn_ns = 0
        self._warn_period_ns = 2_000_000_000
        self._last_twist_enabled_warn_ns = 0
        self._twist_warn_period_ns = 5_000_000_000
        self._resolved_tracking_parent_frame: Optional[str] = self.tracking_parent_frame or None
        self._cached_parent_to_object: Optional[np.ndarray] = None
        self._cached_object_stamp_ns = 0
        self._last_cached_target_warn_ns = 0
        self._last_command_log_ns = 0
        self._command_log_period_ns = 2_000_000_000
        self._object_target_source = "unavailable"
        self._smoothed_linear_cmd = np.zeros(3, dtype=np.float64)
        self._smoothed_angular_cmd = np.zeros(3, dtype=np.float64)

        period = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(period, self._control_step)

        self.get_logger().info(
            f"Marker follower active. robot_marker={self.robot_marker_frame}, object_marker={self.object_marker_frame}, "
            f"twist_output_enabled={self.enable_twist_output}"
        )
        if not self.enable_twist_output:
            self.get_logger().warn("Twist output is disabled. Set enable_twist_output:=true when ready to move.")

    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _transform_age_s(self, transform: TransformStamped, now_ns: int) -> float:
        stamp_ns = self._stamp_to_ns(transform.header.stamp)
        if stamp_ns <= 0:
            return 0.0
        return max(0.0, (now_ns - stamp_ns) * 1e-9)

    def _resolve_tracking_parent_frame(self) -> Optional[str]:
        if self._resolved_tracking_parent_frame:
            return self._resolved_tracking_parent_frame

        try:
            frame_graph = yaml.safe_load(self.tf_buffer.all_frames_as_yaml()) or {}
        except Exception:
            return None

        robot_entry = frame_graph.get(self.robot_marker_frame)
        object_entry = frame_graph.get(self.object_marker_frame)
        if not isinstance(robot_entry, dict) or not isinstance(object_entry, dict):
            return None

        robot_parent = str(robot_entry.get("parent", "")).strip()
        object_parent = str(object_entry.get("parent", "")).strip()
        if not robot_parent or robot_parent != object_parent:
            return None

        self._resolved_tracking_parent_frame = robot_parent
        self.get_logger().info(
            f"Resolved tracking parent frame '{robot_parent}' for marker freshness checks."
        )
        return robot_parent

    def _age_limit(self, specific_limit_s: float) -> float:
        if specific_limit_s > 0.0:
            return specific_limit_s
        return self.max_transform_age_s

    def _cache_object_transform(self, transform: TransformStamped) -> None:
        self._cached_parent_to_object = transform_msg_to_matrix(transform.transform)
        self._cached_object_stamp_ns = self._stamp_to_ns(transform.header.stamp)

    def _cached_object_age_s(self, now_ns: int) -> float:
        if self._cached_object_stamp_ns <= 0:
            return 0.0
        return max(0.0, (now_ns - self._cached_object_stamp_ns) * 1e-9)

    def _lookup_marker_pair_transform(self) -> np.ndarray:
        now_ns = self.get_clock().now().nanoseconds
        parent_frame = self._resolve_tracking_parent_frame()

        if parent_frame:
            tf_parent_to_robot = self.tf_buffer.lookup_transform(parent_frame, self.robot_marker_frame, Time())
            robot_age_s = self._transform_age_s(tf_parent_to_robot, now_ns)
            max_robot_age_s = self._age_limit(self.max_robot_marker_age_s)
            if robot_age_s > max_robot_age_s:
                raise TransformException(
                    "stale marker pair: "
                    f"{self.robot_marker_frame}={robot_age_s:.3f}s, "
                    f"{self.object_marker_frame}=unavailable "
                    f"(limits: robot={max_robot_age_s:.3f}s, object={self._age_limit(self.max_object_marker_age_s):.3f}s, parent={parent_frame})"
                )

            max_object_age_s = self._age_limit(self.max_object_marker_age_s)
            object_age_s = None
            transform_parent_to_object = None
            object_lookup_error = None
            try:
                tf_parent_to_object = self.tf_buffer.lookup_transform(parent_frame, self.object_marker_frame, Time())
                object_age_s = self._transform_age_s(tf_parent_to_object, now_ns)
                if object_age_s <= max_object_age_s:
                    self._cache_object_transform(tf_parent_to_object)
                    transform_parent_to_object = self._cached_parent_to_object
                    self._object_target_source = "live"
                else:
                    object_lookup_error = (
                        f"stale marker pair: {self.robot_marker_frame}={robot_age_s:.3f}s, "
                        f"{self.object_marker_frame}={object_age_s:.3f}s "
                        f"(limits: robot={max_robot_age_s:.3f}s, object={max_object_age_s:.3f}s, parent={parent_frame})"
                    )
            except TransformException as exc:
                object_lookup_error = str(exc)

            if transform_parent_to_object is None:
                cached_object_age_s = self._cached_object_age_s(now_ns)
                if (
                    self.allow_cached_object_target
                    and self._cached_parent_to_object is not None
                    and cached_object_age_s <= max_object_age_s
                ):
                    self._object_target_source = "cached"
                    if now_ns - self._last_cached_target_warn_ns > self._warn_period_ns:
                        self.get_logger().warn(
                            "Using cached object marker pose because the live target marker is unavailable. "
                            f"cached_age={cached_object_age_s:.3f}s, parent={parent_frame}"
                        )
                        self._last_cached_target_warn_ns = now_ns
                    transform_parent_to_object = self._cached_parent_to_object
                else:
                    if object_lookup_error is None:
                        object_lookup_error = (
                            f"stale marker pair: {self.robot_marker_frame}={robot_age_s:.3f}s, "
                            f"{self.object_marker_frame}=unavailable "
                            f"(limits: robot={max_robot_age_s:.3f}s, object={max_object_age_s:.3f}s, parent={parent_frame})"
                        )
                    self._object_target_source = "unavailable"
                    raise TransformException(object_lookup_error)

            transform_parent_to_robot = transform_msg_to_matrix(tf_parent_to_robot.transform)
            return np.linalg.inv(transform_parent_to_robot) @ transform_parent_to_object

        tf_robot_to_object = self.tf_buffer.lookup_transform(
            self.robot_marker_frame,
            self.object_marker_frame,
            Time(),
        )
        if self.max_transform_age_s > 0.0:
            tf_age_s = self._transform_age_s(tf_robot_to_object, now_ns)
            if tf_age_s > self.max_transform_age_s:
                self._object_target_source = "unavailable"
                raise TransformException(
                    f"stale marker transform ({tf_age_s:.3f}s old) while waiting to infer a common parent frame"
                )
        self._object_target_source = "live"
        return transform_msg_to_matrix(tf_robot_to_object.transform)

    def _control_step(self) -> None:
        try:
            transform_robot_to_object = self._lookup_marker_pair_transform()
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if "stale marker" in str(exc):
                if now_ns - self._last_stale_tf_warn_ns > self._warn_period_ns:
                    self.get_logger().warn(
                        f"Skipping stale marker transform: {exc}. "
                        "Holding zero twist until a fresh marker pair is visible."
                    )
                    self._last_stale_tf_warn_ns = now_ns
            else:
                if now_ns - self._last_tf_warn_ns > self._warn_period_ns:
                    self.get_logger().warn(f"Waiting for marker transforms: {exc}")
                    self._last_tf_warn_ns = now_ns
            if self.publish_zero_when_lost:
                self._reset_command_filter()
                self._publish_twist(np.zeros(3), np.zeros(3))
            return

        # Desired EE pose is built from object marker pose and your configured offset.
        transform_robot_to_ee_desired = transform_robot_to_object @ self.object_marker_to_ee_target

        # Current EE pose is inferred from the marker attached to the robot and known fixed marker->EE transform.
        transform_ee_to_ee_desired = np.linalg.inv(self.robot_marker_to_ee) @ transform_robot_to_ee_desired

        pos_error = transform_ee_to_ee_desired[:3, 3]
        rot_error = rotation_matrix_to_rotvec(transform_ee_to_ee_desired[:3, :3])

        angle_error = float(np.linalg.norm(rot_error))
        dist_error = float(np.linalg.norm(pos_error))

        linear_cmd = np.zeros(3, dtype=np.float64)
        angular_cmd = np.zeros(3, dtype=np.float64)
        if dist_error > self.position_tolerance_m:
            linear_cmd = self._compute_linear_command(pos_error, dist_error)
        if angle_error > self.angle_tolerance_rad:
            angular_cmd = clamp_vector(self.angular_gain * rot_error, self.max_angular_speed)

        if float(np.linalg.norm(linear_cmd)) > 0.0 or float(np.linalg.norm(angular_cmd)) > 0.0:
            linear_cmd, angular_cmd = self._smooth_command(linear_cmd, angular_cmd)
        else:
            self._reset_command_filter()

        self._publish_target_pose(transform_robot_to_ee_desired)
        self._publish_twist(linear_cmd, angular_cmd)

    def _reset_command_filter(self) -> None:
        self._smoothed_linear_cmd.fill(0.0)
        self._smoothed_angular_cmd.fill(0.0)

    def _compute_linear_command(self, pos_error: np.ndarray, dist_error: float) -> np.ndarray:
        if self.far_field_distance_m <= self.position_tolerance_m:
            distance_scale = 1.0
        else:
            distance_scale = float(
                np.clip(
                    (dist_error - self.position_tolerance_m)
                    / (self.far_field_distance_m - self.position_tolerance_m),
                    0.0,
                    1.0,
                )
            )
        effective_linear_gain = self.linear_gain * (
            1.0 + (self.far_linear_gain_multiplier - 1.0) * distance_scale
        )
        effective_max_linear_speed = self.max_linear_speed + (
            self.far_max_linear_speed - self.max_linear_speed
        ) * distance_scale
        return clamp_vector(effective_linear_gain * pos_error, effective_max_linear_speed)

    def _smooth_command(self, linear_cmd: np.ndarray, angular_cmd: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        alpha = self.command_smoothing_alpha
        if alpha >= 1.0:
            self._smoothed_linear_cmd = linear_cmd.copy()
            self._smoothed_angular_cmd = angular_cmd.copy()
        else:
            self._smoothed_linear_cmd = alpha * linear_cmd + (1.0 - alpha) * self._smoothed_linear_cmd
            self._smoothed_angular_cmd = alpha * angular_cmd + (1.0 - alpha) * self._smoothed_angular_cmd
        return self._smoothed_linear_cmd.copy(), self._smoothed_angular_cmd.copy()

    def _publish_target_pose(self, transform_robot_to_ee_desired: np.ndarray) -> None:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.robot_marker_frame

        translation = transform_robot_to_ee_desired[:3, 3]
        quat = matrix_to_quaternion(transform_robot_to_ee_desired[:3, :3])

        pose_msg.pose.position.x = float(translation[0])
        pose_msg.pose.position.y = float(translation[1])
        pose_msg.pose.position.z = float(translation[2])
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])

        self.target_pose_pub.publish(pose_msg)

    def _publish_twist(self, linear: np.ndarray, angular: np.ndarray) -> None:
        if not self.enable_twist_output:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_twist_enabled_warn_ns > self._twist_warn_period_ns:
                lin_norm = float(np.linalg.norm(linear))
                ang_norm = float(np.linalg.norm(angular))
                if lin_norm > 1e-4 or ang_norm > 1e-4:
                    self.get_logger().warn(
                        f"Twist output disabled while command is non-zero (lin={lin_norm:.3f}, ang={ang_norm:.3f})."
                    )
                    self._last_twist_enabled_warn_ns = now_ns
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

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_command_log_ns > self._command_log_period_ns:
            lin_norm = float(np.linalg.norm(linear))
            ang_norm = float(np.linalg.norm(angular))
            if lin_norm > 1e-4 or ang_norm > 1e-4:
                self.get_logger().info(
                    f"Publishing twist (source={self._object_target_source}, lin={lin_norm:.3f}, ang={ang_norm:.3f})."
                )
                self._last_command_log_ns = now_ns


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


if __name__ == "__main__":
    main()
