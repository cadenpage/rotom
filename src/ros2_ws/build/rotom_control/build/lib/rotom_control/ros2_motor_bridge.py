import math
import os
import sys
import threading
import time
from pathlib import Path
from typing import Dict, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

def _maybe_add_motors_path() -> None:
    candidates = []
    env_root = os.environ.get("ROTOM_ROOT")
    if env_root:
        candidates.append(Path(env_root))
    cwd = Path.cwd()
    candidates.append(cwd)
    candidates.extend(list(cwd.parents))

    for base in candidates:
        src_root = base / "src"
        if (src_root / "motors").is_dir():
            sys.path.insert(0, str(src_root))
            return
        if (base / "motors").is_dir():
            sys.path.insert(0, str(base))
            return


try:
    from motors import FeetechMotorsBus, Motor, MotorNormMode
except ModuleNotFoundError:
    _maybe_add_motors_path()
    from motors import FeetechMotorsBus, Motor, MotorNormMode


DEFAULT_JOINT_NAMES = [
    "base_to_shoulder",
    "shoulder_to_elbow",
    "elbow_to_wrist",
    "wrist_to_flick",
    "flick_to_claw",
    "flick_to_ee",
]
DEFAULT_MOTOR_NAMES = ["Base", "Shoulder", "Elbow", "Wrist", "Flick", "EE"]
DEFAULT_MOTOR_IDS = [6, 5, 4, 3, 2, 1]
DEFAULT_MOTOR_MODELS = ["sts3215"] * 6
DEFAULT_LOWER_LIMITS = [-3.141593, -0.322215, -2.192594, -3.141593, -1.480653, -2.023815]
DEFAULT_UPPER_LIMITS = [3.141593, 0.446497, 1.410073, 3.141593, 0.348299, 2.178785]
DEFAULT_JOINT_SIGNS = [-1, -1, -1, -1, -1, 1]


def _wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class RotomMotorBridge(Node):
    def __init__(self) -> None:
        super().__init__("rotom_motor_bridge")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("protocol_version", 0)
        self.declare_parameter("handshake", False)
        self.declare_parameter("enable_torque", True)
        self.declare_parameter("configure_motors_on_start", True)
        self.declare_parameter("maximum_acceleration", 254)
        self.declare_parameter("acceleration", 254)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("command_topic", "/joint_command")
        self.declare_parameter("state_topic", "/joint_states")
        self.declare_parameter("trajectory_action", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter("enable_trajectory_action", True)
        self.declare_parameter("trajectory_goal_tolerance", 0.08)
        self.declare_parameter("trajectory_goal_timeout_sec", 4.0)
        self.declare_parameter("sync_read_retries", 2)
        self.declare_parameter("sync_write_retries", 1)
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("motor_names", DEFAULT_MOTOR_NAMES)
        self.declare_parameter("motor_ids", DEFAULT_MOTOR_IDS)
        self.declare_parameter("motor_models", DEFAULT_MOTOR_MODELS)
        self.declare_parameter("lower_limits", DEFAULT_LOWER_LIMITS)
        self.declare_parameter("upper_limits", DEFAULT_UPPER_LIMITS)
        self.declare_parameter("joint_signs", DEFAULT_JOINT_SIGNS)

        self.port = self.get_parameter("port").value
        self.protocol_version = int(self.get_parameter("protocol_version").value)
        self.handshake = bool(self.get_parameter("handshake").value)
        self.enable_torque = bool(self.get_parameter("enable_torque").value)
        self.configure_motors_on_start = bool(self.get_parameter("configure_motors_on_start").value)
        self.maximum_acceleration = int(self.get_parameter("maximum_acceleration").value)
        self.acceleration = int(self.get_parameter("acceleration").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.command_topic = self.get_parameter("command_topic").value
        self.state_topic = self.get_parameter("state_topic").value
        self.trajectory_action = self.get_parameter("trajectory_action").value
        self.enable_trajectory_action = bool(self.get_parameter("enable_trajectory_action").value)
        self.trajectory_goal_tolerance = float(self.get_parameter("trajectory_goal_tolerance").value)
        self.trajectory_goal_timeout_sec = float(self.get_parameter("trajectory_goal_timeout_sec").value)
        self.sync_read_retries = int(self.get_parameter("sync_read_retries").value)
        self.sync_write_retries = int(self.get_parameter("sync_write_retries").value)

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.motor_names = list(self.get_parameter("motor_names").value)
        self.motor_ids = list(self.get_parameter("motor_ids").value)
        self.motor_models = list(self.get_parameter("motor_models").value)
        self.lower_limits = list(self.get_parameter("lower_limits").value)
        self.upper_limits = list(self.get_parameter("upper_limits").value)
        self.joint_signs = list(self.get_parameter("joint_signs").value)

        self._validate_config()

        self.joint_to_motor = dict(zip(self.joint_names, self.motor_names))
        self.joint_sign = dict(zip(self.joint_names, self.joint_signs))
        self.joint_limits_rad = {
            name: (lo, hi)
            for name, lo, hi in zip(self.joint_names, self.lower_limits, self.upper_limits)
        }
        self._bus_lock = threading.RLock()
        self._last_read_error_log_ns = 0
        self._read_error_log_period_ns = 500_000_000  # 0.5s

        self.bus = self._connect_bus()

        self.cmd_sub = self.create_subscription(
            JointState, self.command_topic, self._handle_command, 10
        )
        self.state_pub = self.create_publisher(JointState, self.state_topic, 10)

        self._action_cb_group = ReentrantCallbackGroup()
        self._action_server = None
        if self.enable_trajectory_action:
            self._action_server = ActionServer(
                self,
                FollowJointTrajectory,
                self.trajectory_action,
                execute_callback=self._execute_trajectory,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
                callback_group=self._action_cb_group,
            )

        timer_period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self._publish_state)

    def _validate_config(self) -> None:
        lengths = {
            "joint_names": len(self.joint_names),
            "motor_names": len(self.motor_names),
            "motor_ids": len(self.motor_ids),
            "motor_models": len(self.motor_models),
            "lower_limits": len(self.lower_limits),
            "upper_limits": len(self.upper_limits),
            "joint_signs": len(self.joint_signs),
        }
        if len(set(lengths.values())) != 1:
            raise ValueError(f"Config lists must be the same length: {lengths}")

    def _connect_bus(self) -> FeetechMotorsBus:
        motors = {
            name: Motor(id=m_id, model=m_model, norm_mode=MotorNormMode.DEGREES)
            for name, m_id, m_model in zip(self.motor_names, self.motor_ids, self.motor_models)
        }
        bus = FeetechMotorsBus(
            port=self.port,
            motors=motors,
            protocol_version=self.protocol_version,
        )
        bus.connect(handshake=self.handshake)
        if self.configure_motors_on_start:
            bus.configure_motors(
                return_delay_time=0,
                maximum_acceleration=self.maximum_acceleration,
                acceleration=self.acceleration,
            )
        bus.calibration = bus.read_calibration()
        if self.enable_torque:
            bus.enable_torque()
        return bus

    def _handle_command(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return

        self._write_joint_positions(dict(zip(msg.name, msg.position)))

    def _raw_to_rad(self, joint_name: str, motor_name: str, raw_tick: int) -> float:
        model = self.bus.motors[motor_name].model
        max_res = self.bus.model_resolution_table[model] - 1
        center_tick = int(max_res / 2)
        motor_rad = (raw_tick - center_tick) * (2.0 * math.pi) / max_res
        return self.joint_sign[joint_name] * motor_rad

    def _project_to_joint_limits(self, joint_name: str, rad: float) -> float:
        lo, hi = self.joint_limits_rad[joint_name]
        span = hi - lo
        if span >= (2.0 * math.pi * 0.95):
            return _wrap_to_pi(rad)
        while rad < lo:
            rad += 2.0 * math.pi
        while rad > hi:
            rad -= 2.0 * math.pi
        if rad < lo:
            return lo
        if rad > hi:
            return hi
        return rad

    def _rad_to_raw(self, joint_name: str, motor_name: str, rad: float) -> int:
        cal = self.bus.calibration[motor_name]
        model = self.bus.motors[motor_name].model
        max_res = self.bus.model_resolution_table[model] - 1
        center_tick = int(max_res / 2)
        motor_rad = self.joint_sign[joint_name] * rad
        raw = int(round(center_tick + (motor_rad * max_res / (2.0 * math.pi))))
        return max(cal.range_min, min(cal.range_max, raw))

    def _write_joint_positions(self, name_to_pos: Dict[str, float]) -> bool:
        values: Dict[str, int] = {}
        for joint_name, motor_name in self.joint_to_motor.items():
            if joint_name not in name_to_pos:
                continue
            pos_rad = name_to_pos[joint_name]
            lo_rad, hi_rad = self.joint_limits_rad[joint_name]
            clamped_rad = max(lo_rad, min(hi_rad, pos_rad))
            values[motor_name] = self._rad_to_raw(joint_name, motor_name, clamped_rad)

        if not values:
            return False

        try:
            with self._bus_lock:
                self.bus.sync_write(
                    "Goal_Position",
                    values,
                    normalize=False,
                    num_retry=max(self.sync_write_retries, 0),
                )
        except Exception as exc:
            self.get_logger().error(f"Failed to write motor goals: {exc}")
            return False

        return True

    def _read_joint_positions(self) -> Optional[Dict[str, float]]:
        try:
            with self._bus_lock:
                positions_raw = self.bus.sync_read(
                    "Present_Position",
                    self.motor_names,
                    normalize=False,
                    num_retry=max(self.sync_read_retries, 0),
                )
        except Exception as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_read_error_log_ns >= self._read_error_log_period_ns:
                self.get_logger().error(f"Failed to read motor positions: {exc}")
                self._last_read_error_log_ns = now_ns
            return None

        current: Dict[str, float] = {}
        for joint_name in self.joint_names:
            motor_name = self.joint_to_motor[joint_name]
            raw = int(positions_raw[motor_name])
            rad = self._raw_to_rad(joint_name, motor_name, raw)
            current[joint_name] = self._project_to_joint_limits(joint_name, rad)
        return current

    def _goal_reached(
        self, desired: Dict[str, float], current: Dict[str, float], per_joint_tol: Dict[str, float]
    ) -> bool:
        for joint_name, target in desired.items():
            actual = current.get(joint_name)
            if actual is None:
                return False
            tol = per_joint_tol.get(joint_name, self.trajectory_goal_tolerance)
            if abs(actual - target) > tol:
                return False
        return True

    def _goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        joints = goal_request.trajectory.joint_names
        if not joints:
            return GoalResponse.REJECT
        if set(joints) != set(self.joint_names):
            self.get_logger().error(
                f"Rejected trajectory: joint names mismatch. Expected {self.joint_names}, got {joints}"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_trajectory(self, goal_handle):
        traj = goal_handle.request.trajectory
        if not traj.points:
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Empty trajectory"
            goal_handle.abort()
            return result

        start_time = self.get_clock().now()
        start_ns = start_time.nanoseconds
        point_targets_ns = []
        for point in traj.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            target_ns = start_ns + point.time_from_start.sec * 1_000_000_000 + point.time_from_start.nanosec
            point_targets_ns.append(target_ns)
            while self.get_clock().now().nanoseconds < target_ns:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result()
                time.sleep(0.005)

            name_to_pos = dict(zip(traj.joint_names, point.positions))
            if not self._write_joint_positions(name_to_pos):
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = "Failed to write one or more trajectory points"
                goal_handle.abort()
                return result

            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = list(traj.joint_names)
            desired = JointTrajectoryPoint()
            desired.positions = list(point.positions)
            feedback.desired = desired
            goal_handle.publish_feedback(feedback)

        desired_final = dict(zip(traj.joint_names, traj.points[-1].positions))
        per_joint_tol: Dict[str, float] = {
            t.name: t.position
            for t in goal_handle.request.goal_tolerance
            if t.position > 0.0 and t.name
        }

        goal_time_tol_ns = (
            goal_handle.request.goal_time_tolerance.sec * 1_000_000_000
            + goal_handle.request.goal_time_tolerance.nanosec
        )
        settle_ns = max(int(self.trajectory_goal_timeout_sec * 1_000_000_000), 0)
        final_target_ns = point_targets_ns[-1]
        deadline_ns = final_target_ns + settle_ns + goal_time_tol_ns

        while self.get_clock().now().nanoseconds <= deadline_ns:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
            current = self._read_joint_positions()
            if current is not None and self._goal_reached(desired_final, current, per_joint_tol):
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                goal_handle.succeed()
                return result
            time.sleep(0.03)

        current = self._read_joint_positions() or {}
        worst_joint = None
        worst_error = 0.0
        for joint_name, target in desired_final.items():
            actual = current.get(joint_name)
            if actual is None:
                continue
            err = abs(actual - target)
            if err > worst_error:
                worst_error = err
                worst_joint = joint_name

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
        if worst_joint is None:
            result.error_string = "Goal tolerance not met before timeout"
        else:
            tol = per_joint_tol.get(worst_joint, self.trajectory_goal_tolerance)
            result.error_string = (
                f"Goal tolerance violated on '{worst_joint}': error={worst_error:.5f}, tol={tol:.5f}"
            )
        goal_handle.abort()
        return result

    def _publish_state(self) -> None:
        current = self._read_joint_positions()
        if current is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [current[joint_name] for joint_name in self.joint_names]
        self.state_pub.publish(msg)

    def destroy_node(self) -> bool:
        try:
            if self.enable_torque:
                self.bus.disable_torque()
            self.bus.disconnect()
        except Exception as exc:
            self.get_logger().warn(f"Shutdown error: {exc}")
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RotomMotorBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        executor.add_node(node)
        node.get_logger().info("rotom_motor_bridge running; waiting for commands/actions.")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
