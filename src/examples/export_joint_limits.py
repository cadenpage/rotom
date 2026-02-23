import math
import sys
from pathlib import Path

try:
    from motors import FeetechMotorsBus, Motor, MotorNormMode
    from motors.tables import MODEL_RESOLUTION
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from motors import FeetechMotorsBus, Motor, MotorNormMode
    from motors.tables import MODEL_RESOLUTION


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


def ticks_to_rad(raw: int, center_tick: int, max_res: int) -> float:
    return (raw - center_tick) * (2.0 * math.pi) / max_res


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def print_urdf_limits(joint_limits):
    for name, lower, upper in joint_limits:
        print(f'  <joint name="{name}" type="revolute">')
        print(f'    <limit lower="{lower:.6f}" upper="{upper:.6f}" effort="5.0" velocity="3.0"/>')
        print("  </joint>")


def print_yaml_limits(joint_limits):
    print("lower_limits:")
    for _, lower, _ in joint_limits:
        print(f"  - {lower:.6f}")
    print("upper_limits:")
    for _, _, upper in joint_limits:
        print(f"  - {upper:.6f}")


def main() -> None:
    motors = {
        name: Motor(id=m_id, model=m_model, norm_mode=MotorNormMode.DEGREES)
        for name, m_id, m_model in zip(DEFAULT_MOTOR_NAMES, DEFAULT_MOTOR_IDS, DEFAULT_MOTOR_MODELS)
    }
    bus = FeetechMotorsBus(
        port="/dev/ttyACM0",
        motors=motors,
        protocol_version=0,
    )
    bus.connect(handshake=False)
    calibration = bus.read_calibration()

    joint_limits = []
    for joint_name, motor_name, model in zip(
        DEFAULT_JOINT_NAMES, DEFAULT_MOTOR_NAMES, DEFAULT_MOTOR_MODELS
    ):
        cal = calibration[motor_name]
        max_res = MODEL_RESOLUTION[model] - 1
        center_tick = int(max_res / 2)
        print(
            f"{motor_name}: min={cal.range_min} max={cal.range_max} homing_offset={cal.homing_offset}"
        )
        lower_raw = ticks_to_rad(cal.range_min, center_tick, max_res)
        upper_raw = ticks_to_rad(cal.range_max, center_tick, max_res)
        if lower_raw > upper_raw:
            lower_raw, upper_raw = upper_raw, lower_raw
        span = upper_raw - lower_raw
        if span <= 0:
            raise ValueError(f"Invalid range for {motor_name}: min={cal.range_min} max={cal.range_max}")

        if span >= (2.0 * math.pi * 0.95):
            lower_rad, upper_rad = -math.pi, math.pi
        else:
            # Keep the same span but fold the interval near the principal branch for URDF/MoveIt.
            lower_rad = wrap_to_pi(lower_raw)
            upper_rad = lower_rad + span
            if upper_rad > math.pi:
                lower_rad -= 2.0 * math.pi
                upper_rad -= 2.0 * math.pi

        joint_limits.append((joint_name, lower_rad, upper_rad))

    bus.disconnect()

    print_urdf_limits(joint_limits)


if __name__ == "__main__":
    main()
