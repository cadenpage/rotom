import math
import os
import sys
from pathlib import Path
from typing import Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

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
DEFAULT_LOWER_LIMITS = [-3.14159, -2.2, -2.2, -2.0, -1.57, -1.57]
DEFAULT_UPPER_LIMITS = [3.14159, 2.2, 2.2, 2.0, 1.57, 1.57]


def _deg(rad: float) -> float:
    return math.degrees(rad)


def _rad(deg: float) -> float:
    return math.radians(deg)


class RotomMotorBridge(Node):
    def __init__(self) -> None:
        super().__init__("rotom_motor_bridge")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("protocol_version", 0)
        self.declare_parameter("handshake", False)
        self.declare_parameter("enable_torque", True)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("command_topic", "/joint_command")
        self.declare_parameter("state_topic", "/joint_states")
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("motor_names", DEFAULT_MOTOR_NAMES)
        self.declare_parameter("motor_ids", DEFAULT_MOTOR_IDS)
        self.declare_parameter("motor_models", DEFAULT_MOTOR_MODELS)
        self.declare_parameter("lower_limits", DEFAULT_LOWER_LIMITS)
        self.declare_parameter("upper_limits", DEFAULT_UPPER_LIMITS)

        self.port = self.get_parameter("port").value
        self.protocol_version = int(self.get_parameter("protocol_version").value)
        self.handshake = bool(self.get_parameter("handshake").value)
        self.enable_torque = bool(self.get_parameter("enable_torque").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.command_topic = self.get_parameter("command_topic").value
        self.state_topic = self.get_parameter("state_topic").value

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.motor_names = list(self.get_parameter("motor_names").value)
        self.motor_ids = list(self.get_parameter("motor_ids").value)
        self.motor_models = list(self.get_parameter("motor_models").value)
        self.lower_limits = list(self.get_parameter("lower_limits").value)
        self.upper_limits = list(self.get_parameter("upper_limits").value)

        self._validate_config()

        self.joint_to_motor = dict(zip(self.joint_names, self.motor_names))
        self.joint_limits_deg = {
            name: (_deg(lo), _deg(hi))
            for name, lo, hi in zip(self.joint_names, self.lower_limits, self.upper_limits)
        }

        self.bus = self._connect_bus()

        self.cmd_sub = self.create_subscription(
            JointState, self.command_topic, self._handle_command, 10
        )
        self.state_pub = self.create_publisher(JointState, self.state_topic, 10)

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
        bus.calibration = bus.read_calibration()
        if self.enable_torque:
            bus.enable_torque()
        return bus

    def _handle_command(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return

        name_to_pos = dict(zip(msg.name, msg.position))
        values: Dict[str, float] = {}

        for joint_name, motor_name in self.joint_to_motor.items():
            if joint_name not in name_to_pos:
                continue
            pos_rad = name_to_pos[joint_name]
            lo_deg, hi_deg = self.joint_limits_deg[joint_name]
            pos_deg = max(lo_deg, min(hi_deg, _deg(pos_rad)))
            values[motor_name] = pos_deg

        if not values:
            return

        try:
            self.bus.sync_write("Goal_Position", values, normalize=True)
        except Exception as exc:
            self.get_logger().error(f"Failed to write motor goals: {exc}")

    def _publish_state(self) -> None:
        try:
            positions_deg = self.bus.sync_read("Present_Position", self.motor_names, normalize=True)
        except Exception as exc:
            self.get_logger().error(f"Failed to read motor positions: {exc}")
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [_rad(positions_deg[self.joint_to_motor[j]]) for j in self.joint_names]
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
