import math
import sys
import threading
import time
from pathlib import Path
from typing import Dict, Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _maybe_add_motors_path() -> None:
    candidates = [Path.cwd(), *Path.cwd().parents]
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

DEFAULT_JOINT_NAMES = DEFAULT_MOTOR_NAMES = ["O", "A", "B", "C"]
DEFAULT_MOTOR_IDS = [6, 5, 4, 3]
DEFAULT_MOTOR_MODELS = ["sts3215"] * 4
DEFAULT_LOWER_LIMITS = [-1.787524, -3.141593, -3.141593, -3.141593]
DEFAULT_UPPER_LIMITS = [1.787524, 0.405070, 3.141593, 3.141593]

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
        self.declare_parameter("sync_read_retries", 2)
        self.declare_parameter("state_topic", "/joint_states")
        self.declare_parameter("sync_write_retries", 1)
        self.declare_parameter("command_topic", "/joint_commands")
        self.declare_parameter("trajectory_command_topic", "/joint_trajectory")
        self.declare_parameter("trajectory_action", "/army_controller/follow_joint_trajectory")
        self.declare_parameter("trajectory_sample_rate_hz", 30.0)

        

        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("motor_names", DEFAULT_MOTOR_NAMES)
        self.declare_parameter("motor_ids", DEFAULT_MOTOR_IDS)
        self.declare_parameter("motor_models", DEFAULT_MOTOR_MODELS)
        self.declare_parameter("lower_limits", DEFAULT_LOWER_LIMITS)
        self.declare_parameter("upper_limits", DEFAULT_UPPER_LIMITS)

        self.port = str(self.get_parameter("port").value)
        self.protocol_version = int(self.get_parameter("protocol_version").value)
        self.handshake = bool(self.get_parameter("handshake").value)
        self.enable_torque = bool(self.get_parameter("enable_torque").value)
        self.configure_motors_on_start = bool(self.get_parameter("configure_motors_on_start").value)
        self.maximum_acceleration = int(self.get_parameter("maximum_acceleration").value)
        self.acceleration = int(self.get_parameter("acceleration").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.sync_read_retries = int(self.get_parameter("sync_read_retries").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.sync_write_retries = int(self.get_parameter("sync_write_retries").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.trajectory_command_topic = str(self.get_parameter("trajectory_command_topic").value)
        self.trajectory_action = str(self.get_parameter("trajectory_action").value)
        self.trajectory_sample_rate_hz = float(self.get_parameter("trajectory_sample_rate_hz").value)

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.motor_names = list(self.get_parameter("motor_names").value)
        self.motor_ids = list(self.get_parameter("motor_ids").value)
        self.motor_models = list(self.get_parameter("motor_models").value)
        self.lower_limits = list(self.get_parameter("lower_limits").value)
        self.upper_limits = list(self.get_parameter("upper_limits").value)

        if not (
            len(self.joint_names)
            == len(self.motor_names)
            == len(self.motor_ids)
            == len(self.motor_models)
            == len(self.lower_limits)
            == len(self.upper_limits)
        ):
            raise ValueError("All parameter lists must have the same length")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0.0")
        if self.trajectory_sample_rate_hz <= 0.0:
            raise ValueError("trajectory_sample_rate_hz must be > 0.0")

        self.joint_to_motor = dict(zip(self.joint_names, self.motor_names))
        self.joint_limits_rad = {
            name: (lo, hi)
            for name, lo, hi in zip(self.joint_names, self.lower_limits, self.upper_limits)
        }

        self.bus = self._connect_bus()
        self._bus_lock = threading.RLock()
        self._last_joint_positions: Optional[Dict[str, float]] = None
        self._action_cb_group = ReentrantCallbackGroup()

        self.state_pub = self.create_publisher(JointState, self.state_topic, 10)
        self.command_sub = self.create_subscription(JointState, self.command_topic, self._handle_command, 10)
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            self.trajectory_command_topic,
            self._handle_trajectory,
            10,
        )
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.trajectory_action,
            execute_callback=self._execute_trajectory,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_cb_group,
        )

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_state)
        self.get_logger().info(
            f"rotom_motor_bridge publishing {self.state_topic}, listening on {self.command_topic}, "
            f"listening on {self.trajectory_command_topic}, and serving {self.trajectory_action} "
            f"for joints {self.joint_names}"
        )

    # connect to the motor bus and configure motors as needed
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

    # PUBLISH FUNCTIONS

    def _publish_state(self) -> None:
        current = self._read_joint_positions()
        if current is None:
            return
        self._last_joint_positions = current

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [current[joint_name] for joint_name in self.joint_names]
        self.state_pub.publish(msg)

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
            self.get_logger().error(f"Failed to read motor positions: {exc}")
            return None

        current: Dict[str, float] = {}
        for joint_name in self.joint_names:
            motor_name = self.joint_to_motor[joint_name]
            raw = int(positions_raw[motor_name])
            rad = self._raw_to_rad(joint_name, motor_name, raw)
            current[joint_name] = self._project_to_joint_limits(joint_name, rad)
        return current

    def _raw_to_rad(self, joint_name: str, motor_name: str, raw_tick: int) -> float:
        del joint_name
        model = self.bus.motors[motor_name].model
        max_res = self.bus.model_resolution_table[model] - 1
        center_tick = int(max_res / 2)
        motor_rad = (raw_tick - center_tick) * (2.0 * math.pi) / max_res
        return motor_rad

    def _project_to_joint_limits(self, joint_name: str, rad: float) -> float:
        lo, hi = self.joint_limits_rad[joint_name]
        if rad < lo:
            return lo
        if rad > hi:
            return hi
        return rad

    # COMMAND FUNCTIONS

    def _rad_to_raw(self, joint_name: str, motor_name: str, rad: float) -> int:
        cal = self.bus.calibration[motor_name]
        model = self.bus.motors[motor_name].model
        max_res = self.bus.model_resolution_table[model] - 1
        center_tick = int(max_res / 2)
        raw = int(round(center_tick + (rad * max_res / (2.0 * math.pi))))
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

    def _handle_command(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        if len(msg.name) != len(msg.position):
            self.get_logger().warn(
                "Ignoring JointState command because name and position lengths differ."
            )
            return

        self._write_joint_positions(dict(zip(msg.name, msg.position)))

    def _handle_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.joint_names or not msg.points:
            return

        point = msg.points[-1]
        if not point.positions:
            return
        if len(msg.joint_names) != len(point.positions):
            self.get_logger().warn(
                "Ignoring JointTrajectory command because joint_names and positions lengths differ."
            )
            return

        self._write_joint_positions(dict(zip(msg.joint_names, point.positions)))

    def _goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        joints = list(goal_request.trajectory.joint_names)
        if not joints:
            return GoalResponse.REJECT
        if set(joints) != set(self.joint_names):
            self.get_logger().error(
                f"Rejected FollowJointTrajectory goal: expected joints {self.joint_names}, got {joints}"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_trajectory(self, goal_handle) -> FollowJointTrajectory.Result:
        traj = goal_handle.request.trajectory
        result = FollowJointTrajectory.Result()

        if not traj.points:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Empty trajectory"
            goal_handle.abort()
            return result

        current_positions = self._last_joint_positions or self._read_joint_positions()
        if current_positions is None:
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = "Unable to read current joint positions before executing trajectory"
            goal_handle.abort()
            return result

        prev_positions = {
            joint_name: current_positions[joint_name]
            for joint_name in traj.joint_names
            if joint_name in current_positions
        }
        if len(prev_positions) != len(traj.joint_names):
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = "Current joint state does not cover all trajectory joints"
            goal_handle.abort()
            return result

        start_ns = self.get_clock().now().nanoseconds
        prev_time_ns = 0
        sample_period_s = 1.0 / self.trajectory_sample_rate_hz

        for point in traj.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            if len(point.positions) != len(traj.joint_names):
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "Trajectory point has mismatched joint_names and positions"
                goal_handle.abort()
                return result

            target_positions = dict(zip(traj.joint_names, point.positions))
            point_time_ns = point.time_from_start.sec * 1_000_000_000 + point.time_from_start.nanosec
            segment_duration_ns = max(point_time_ns - prev_time_ns, 0)
            segment_duration_s = segment_duration_ns / 1_000_000_000.0

            if segment_duration_s <= 0.0:
                if not self._write_joint_positions(target_positions):
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = "Failed to write one or more trajectory points"
                    goal_handle.abort()
                    return result
            else:
                num_samples = max(int(math.ceil(segment_duration_s / sample_period_s)), 1)
                for sample_idx in range(1, num_samples + 1):
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        return FollowJointTrajectory.Result()

                    alpha = sample_idx / num_samples
                    interpolated_positions = {
                        joint_name: (
                            (1.0 - alpha) * prev_positions[joint_name]
                            + alpha * target_positions[joint_name]
                        )
                        for joint_name in traj.joint_names
                    }
                    target_ns = start_ns + prev_time_ns + int(round(alpha * segment_duration_ns))

                    while self.get_clock().now().nanoseconds < target_ns:
                        if goal_handle.is_cancel_requested:
                            goal_handle.canceled()
                            return FollowJointTrajectory.Result()
                        time.sleep(min(sample_period_s / 2.0, 0.005))

                    if not self._write_joint_positions(interpolated_positions):
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
            prev_positions = target_positions
            prev_time_ns = point_time_ns

        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        return result



    # CLEANUP
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
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
