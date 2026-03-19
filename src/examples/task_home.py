"""Move joints O/A/B/C to a configurable raw-position home pose.

This is similar to ``src/examples/home.py``, but instead of sending every motor
to the raw encoder midpoint, it sends the four main arm joints to an explicit,
editable home pose in raw 0..4095 position units. That makes it easy to copy
values directly from the calibration GUI.
"""

from __future__ import annotations

import time

from motors import FeetechMotorsBus, Motor, OperatingMode


PORT = "/dev/ttyACM0"

# Arm definition: name -> (id, model)
MOTORS = {
    "O": (6, "sts3215"),
    "A": (5, "sts3215"),
    "B": (4, "sts3215"),
    "C": (3, "sts3215"),
}

# Main planar arm only.
ARM = ["O", "A", "B", "C"]

# Edit these values to tune the desired task-home pose.
# The units are raw 0..4095 motor positions from the calibration GUI.
HOME_POSITION = {
    "O": 2064,
    "A": 940,
    "B": 2271,
    "C": 836,
}

SETTLE_TIME_S = 3.0
KEEP_TORQUE_ENABLED = True


def main() -> None:
    if any(name not in MOTORS for name in ARM):
        missing = [name for name in ARM if name not in MOTORS]
        raise RuntimeError(f"ARM contains unknown motor names: {missing}")

    if any(name not in HOME_POSITION for name in ARM):
        missing = [name for name in ARM if name not in HOME_POSITION]
        raise RuntimeError(f"HOME_POSITION is missing targets for: {missing}")

    bus = FeetechMotorsBus(
        port=PORT,
        motors={
            name: Motor(id=m_id, model=m_model, norm_mode="position")
            for name, (m_id, m_model) in MOTORS.items()
        },
        protocol_version=0,
    )

    bus.connect(handshake=False)
    try:
        for name in ARM:
            bus.write("Operating_Mode", name, OperatingMode.POSITION.value, normalize=False)

        current_position = {name: bus.read("Present_Position", name, normalize=False) for name in ARM}
        print("Current arm pose (raw position):")
        for name in ARM:
            print(f"  {name}: {current_position[name]:4d}")

        print("Target task-home pose (raw position):")
        for name in ARM:
            print(f"  {name}: {HOME_POSITION[name]:4d}")

        bus.sync_write("Goal_Position", {name: HOME_POSITION[name] for name in ARM}, normalize=False)
        time.sleep(SETTLE_TIME_S)
        print("Robot moved to task-home pose.")
    finally:
        bus.disconnect(disable_torque=not KEEP_TORQUE_ENABLED)
        if KEEP_TORQUE_ENABLED:
            print("Torque left enabled to hold task-home pose.")
        else:
            print("Torque disabled on exit.")


if __name__ == "__main__":
    main()
