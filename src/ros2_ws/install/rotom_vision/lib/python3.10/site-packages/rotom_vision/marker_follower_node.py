from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

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
        self.declare_parameter("max_angular_speed", 0.8)
        self.declare_parameter("position_tolerance_m", 0.005)
        self.declare_parameter("angle_tolerance_rad", 0.05)
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
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.position_tolerance_m = float(self.get_parameter("position_tolerance_m").value)
        self.angle_tolerance_rad = float(self.get_parameter("angle_tolerance_rad").value)
        self.publish_zero_when_lost = bool(self.get_parameter("publish_zero_when_lost").value)
        self.enable_twist_output = bool(self.get_parameter("enable_twist_output").value)

        if self.control_rate_hz <= 0.0:
            raise ValueError(f"control_rate_hz must be > 0.0. Got {self.control_rate_hz}")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.twist_pub = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, self.target_pose_topic, 10)

        self._last_tf_warn_ns = 0
        self._warn_period_ns = 2_000_000_000
        self._last_twist_enabled_warn_ns = 0
        self._twist_warn_period_ns = 5_000_000_000

        period = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(period, self._control_step)

        self.get_logger().info(
            f"Marker follower active. robot_marker={self.robot_marker_frame}, object_marker={self.object_marker_frame}, "
            f"twist_output_enabled={self.enable_twist_output}"
        )
        if not self.enable_twist_output:
            self.get_logger().warn("Twist output is disabled. Set enable_twist_output:=true when ready to move.")

    def _control_step(self) -> None:
        try:
            tf_robot_to_object = self.tf_buffer.lookup_transform(
                self.robot_marker_frame,
                self.object_marker_frame,
                Time(),
            )
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns > self._warn_period_ns:
                self.get_logger().warn(f"Waiting for marker transforms: {exc}")
                self._last_tf_warn_ns = now_ns
            if self.publish_zero_when_lost:
                self._publish_twist(np.zeros(3), np.zeros(3))
            return

        transform_robot_to_object = transform_msg_to_matrix(tf_robot_to_object.transform)

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
            linear_cmd = clamp_vector(self.linear_gain * pos_error, self.max_linear_speed)
        if angle_error > self.angle_tolerance_rad:
            angular_cmd = clamp_vector(self.angular_gain * rot_error, self.max_angular_speed)

        self._publish_target_pose(transform_robot_to_ee_desired)
        self._publish_twist(linear_cmd, angular_cmd)

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
