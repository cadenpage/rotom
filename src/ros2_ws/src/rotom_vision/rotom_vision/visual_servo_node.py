"""Lightweight Jacobian-based visual servo node for Rotom.

Subscribes to:
  /servo_node/delta_twist_cmds  (geometry_msgs/TwistStamped, expressed in tool0 frame)
  /joint_states                 (sensor_msgs/JointState)

Publishes:
  /joint_command                (sensor_msgs/JointState)

On each incoming TwistStamped the node:
  1. Reads the current joint positions from /joint_states.
  2. Computes forward kinematics (world → tool0) and the geometric Jacobian
     using the Rotom URDF kinematics hard-coded below.
  3. Transforms the EE-frame twist into the world frame using the FK rotation.
  4. Solves the damped least-squares pseudo-inverse  dq = J^+ * twist_world.
  5. Scales dq to respect max_joint_velocity_rad_s.
  6. Publishes  q_new = q_current + dq * dt  to /joint_command.

Safety:
  - All joint positions are clamped to the configured joint limits before publishing.
  - The node does nothing until the first /joint_states message is received.
  - If the magnitude of the incoming twist is below a threshold the command is skipped.
"""

import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState


# ---------------------------------------------------------------------------
# Robot kinematics extracted from rotom.urdf.xacro
# ---------------------------------------------------------------------------

# Revolute joints in chain order (world → tool0).
# Each entry: (origin_xyz, origin_rpy, axis_xyz)
_JOINT_PARAMS: List[Tuple[List[float], List[float], List[float]]] = [
    ([0.0,     -0.03,    0.04025],  [1.570796, -1.570796, 0.0], [1.0, 0.0, 0.0]),  # base_to_shoulder
    ([0.0166,  -0.01825, 0.02265],  [0.0, 0.0, 0.0],            [0.0, 1.0, 0.0]),  # shoulder_to_elbow
    ([0.0868,   0.0,     0.0],      [0.0, 0.0, 0.0],            [0.0, 1.0, 0.0]),  # elbow_to_wrist
    ([0.06025,  0.01825, -0.0285],  [0.0, 0.0, 0.0],            [1.0, 0.0, 0.0]),  # wrist_to_flick
    ([0.0206,  -0.01825, 0.02265],  [0.0, 0.0, 0.0],            [0.0, 1.0, 0.0]),  # flick_to_claw
    ([0.0668,   0.0365,  0.0],      [0.0, 0.0, 0.0],            [0.0, 1.0, 0.0]),  # flick_to_ee
]

# Fixed transform from ee → tool0
_TOOL0_XYZ = [0.055, -0.01825, 0.0]

_DEFAULT_JOINT_NAMES = [
    "base_to_shoulder",
    "shoulder_to_elbow",
    "elbow_to_wrist",
    "wrist_to_flick",
    "flick_to_claw",
    "flick_to_ee",
]

# Joint limits [lower, upper] matching joint order above
_JOINT_LIMITS = [
    (-3.141593,  3.141593),
    (-0.322215,  0.446497),
    (-2.192594,  1.410073),
    (-3.141593,  3.141593),
    (-1.480653,  0.348299),
    (-2.023815,  2.178785),
]


# ---------------------------------------------------------------------------
# Pure-numpy kinematics helpers
# ---------------------------------------------------------------------------

def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr],
        [sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr],
        [-sp,      cp * sr,                  cp * cr               ],
    ], dtype=np.float64)


def _axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    ux, uy, uz = float(axis[0]), float(axis[1]), float(axis[2])
    return np.array([
        [c + ux*ux*(1-c),       ux*uy*(1-c) - uz*s,  ux*uz*(1-c) + uy*s],
        [uy*ux*(1-c) + uz*s,    c + uy*uy*(1-c),      uy*uz*(1-c) - ux*s],
        [uz*ux*(1-c) - uy*s,    uz*uy*(1-c) + ux*s,   c + uz*uz*(1-c)   ],
    ], dtype=np.float64)


def _fk_and_jacobian(q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Compute FK (world→tool0) and the 6×n geometric Jacobian.

    Returns
    -------
    T_tool0 : (4, 4) homogeneous transform, world frame to tool0.
    J       : (6, n) Jacobian – upper 3 rows are linear velocity contribution,
              lower 3 rows are angular velocity contribution.
    """
    n = len(q)
    joint_pos_w: List[np.ndarray] = []
    joint_axis_w: List[np.ndarray] = []

    T = np.eye(4, dtype=np.float64)

    for i in range(n):
        xyz, rpy, axis_local = _JOINT_PARAMS[i]
        axis_np = np.array(axis_local, dtype=np.float64)

        # Apply the joint's origin (fixed translation + rotation)
        R_origin = _rpy_to_matrix(*rpy)
        T_origin = np.eye(4, dtype=np.float64)
        T_origin[:3, :3] = R_origin
        T_origin[:3, 3] = xyz
        T = T @ T_origin

        # Record joint axis and origin in world frame (before joint rotation)
        joint_axis_w.append(T[:3, :3] @ axis_np)
        joint_pos_w.append(T[:3, 3].copy())

        # Apply joint rotation about local axis by q[i]
        R_q = _axis_angle_to_matrix(axis_np, q[i])
        T_q = np.eye(4, dtype=np.float64)
        T_q[:3, :3] = R_q
        T = T @ T_q

    # Apply fixed tool0 offset
    T_tool0_local = np.eye(4, dtype=np.float64)
    T_tool0_local[:3, 3] = _TOOL0_XYZ
    T = T @ T_tool0_local

    p_ee = T[:3, 3]

    # Geometric Jacobian
    J = np.zeros((6, n), dtype=np.float64)
    for i in range(n):
        z_i = joint_axis_w[i]
        p_i = joint_pos_w[i]
        J[:3, i] = np.cross(z_i, p_ee - p_i)  # linear
        J[3:, i] = z_i                          # angular

    return T, J


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class VisualServoNode(Node):
    def __init__(self) -> None:
        super().__init__("visual_servo")

        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_command_topic", "/joint_command")
        self.declare_parameter("joint_names", _DEFAULT_JOINT_NAMES)
        self.declare_parameter("max_joint_velocity_rad_s", 0.5)
        self.declare_parameter("damping", 0.05)
        self.declare_parameter("twist_deadband", 1e-4)
        self.declare_parameter("default_dt_s", 0.05)

        self.twist_topic = str(self.get_parameter("twist_topic").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.joint_command_topic = str(self.get_parameter("joint_command_topic").value)
        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.max_joint_vel = float(self.get_parameter("max_joint_velocity_rad_s").value)
        self.damping = float(self.get_parameter("damping").value)
        self.twist_deadband = float(self.get_parameter("twist_deadband").value)
        self.default_dt_s = float(self.get_parameter("default_dt_s").value)

        self._current_q: Optional[Dict[str, float]] = None
        self._last_twist_time: Optional[rclpy.time.Time] = None

        self._cmd_pub = self.create_publisher(JointState, self.joint_command_topic, 10)
        self.create_subscription(JointState, self.joint_state_topic, self._joint_state_cb, 10)
        self.create_subscription(TwistStamped, self.twist_topic, self._twist_cb, 10)

        self.get_logger().info(
            f"Visual servo node active. "
            f"twist_in={self.twist_topic}, cmd_out={self.joint_command_topic}"
        )

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _joint_state_cb(self, msg: JointState) -> None:
        q: Dict[str, float] = {}
        for name, pos in zip(msg.name, msg.position):
            q[str(name)] = float(pos)
        self._current_q = q

    def _twist_cb(self, msg: TwistStamped) -> None:
        now = self.get_clock().now()
        if self._last_twist_time is not None:
            dt = (now - self._last_twist_time).nanoseconds * 1e-9
            # Clamp to a reasonable range to avoid huge jumps after a pause
            dt = float(np.clip(dt, 0.005, 0.2))
        else:
            dt = self.default_dt_s
        self._last_twist_time = now

        self._apply_twist(msg, dt)

    # ------------------------------------------------------------------
    # Core control step
    # ------------------------------------------------------------------

    def _apply_twist(self, twist_msg: TwistStamped, dt: float) -> None:
        if self._current_q is None:
            return

        # Build ordered joint angle vector
        try:
            q = np.array([self._current_q[name] for name in self.joint_names], dtype=np.float64)
        except KeyError as exc:
            self.get_logger().warn(f"Missing joint in /joint_states: {exc}", throttle_duration_sec=2.0)
            return

        # Extract 6D twist [vx, vy, vz, wx, wy, wz] in the EE (tool0) frame
        t = twist_msg.twist
        twist_ee = np.array([
            t.linear.x,  t.linear.y,  t.linear.z,
            t.angular.x, t.angular.y, t.angular.z,
        ], dtype=np.float64)

        if float(np.linalg.norm(twist_ee)) < self.twist_deadband:
            return

        # Compute FK and Jacobian at current configuration
        T_ee, J = _fk_and_jacobian(q)
        R_ee = T_ee[:3, :3]

        # Convert EE-frame twist to world frame for use with the space Jacobian.
        # marker_follower publishes the twist in tool0/ee frame.
        frame_id = twist_msg.header.frame_id.strip()
        if frame_id in ("tool0", "ee", "flick_to_ee", ""):
            twist_world = np.concatenate([
                R_ee @ twist_ee[:3],
                R_ee @ twist_ee[3:],
            ])
        else:
            # If published in world frame already, use as-is
            twist_world = twist_ee

        # Damped least-squares pseudo-inverse: J^+ = J^T (J J^T + λ² I)^-1
        lam2 = self.damping ** 2
        J_pinv = J.T @ np.linalg.inv(J @ J.T + lam2 * np.eye(6, dtype=np.float64))

        dq = J_pinv @ twist_world

        # Scale so the fastest joint moves at most max_joint_velocity_rad_s
        max_dq = float(np.max(np.abs(dq)))
        if max_dq > self.max_joint_vel:
            dq = dq * (self.max_joint_vel / max_dq)

        q_new = q + dq * dt

        # Clamp to joint limits
        for i, (lo, hi) in enumerate(_JOINT_LIMITS):
            q_new[i] = float(np.clip(q_new[i], lo, hi))

        # Publish
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = list(self.joint_names)
        cmd.position = q_new.tolist()
        self._cmd_pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = VisualServoNode()
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
