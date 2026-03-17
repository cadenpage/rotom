
#.  python -m examples.change_torque
"""
Typical workflow:
1) Set MODE = "disable" and run. Move the arm by hand.
2) Set MODE = "hold" and run. It will set goals to current positions, then enable torque.
"""

from motors import FeetechMotorsBus, Motor, OperatingMode

PORT = "/dev/ttyACM0"

# Arm definition: name -> (id, model)
MOTORS = {
    "Base": (6, "sts3215"),
    "Shoulder": (5, "sts3215"),
    "Elbow": (4, "sts3215"),
    "Wrist": (3, "sts3215"),
    "Flick": (2, "sts3215"),
}

# Which motors to act on.
ARM = ["Base", "Shoulder", "Elbow", "Wrist", "Flick"]

# One of: "disable" | "enable" | "hold"
# - disable: torque off so you can move by hand
# - enable: torque on (doesn't change goal position)
# - hold: reads current positions, writes them as Goal_Position, then enables torque
MODE = "disable"


def main() -> None:
    if MODE not in {"disable", "enable", "hold"}:
        raise RuntimeError('MODE must be one of: "disable", "enable", "hold"')

    if any(name not in MOTORS for name in ARM):
        missing = [name for name in ARM if name not in MOTORS]
        raise RuntimeError(f"ARM contains unknown motor names: {missing}")

    bus = FeetechMotorsBus(
        port=PORT,
        motors={name: Motor(id=m_id, model=m_model, norm_mode="position") for name, (m_id, m_model) in MOTORS.items()},
        protocol_version=0,
    )

    bus.connect(handshake=False)

    try:
        # Goal_Position control expects position mode.
        for name in ARM:
            bus.write("Operating_Mode", name, OperatingMode.POSITION.value, normalize=False)

        if MODE == "disable":
            bus.disable_torque(ARM)
            print(f"Torque disabled for: {ARM}")
            return

        if MODE == "enable":
            bus.enable_torque(ARM)
            print(f"Torque enabled for: {ARM}")
            return

        # MODE == "hold": safest way to re-enable after manual posing.
        # Read current positions while torque is off, set them as goals, then enable torque.
        positions = {name: bus.read("Present_Position", name, normalize=False) for name in ARM}
        for name, cur in positions.items():
            min_lim = bus.read("Min_Position_Limit", name, normalize=False)
            max_lim = bus.read("Max_Position_Limit", name, normalize=False)
            goal = max(min_lim, min(cur, max_lim))
            bus.write("Goal_Position", name, goal, normalize=False)

        bus.enable_torque(ARM)
        print(f"Holding current pose for: {ARM}")
    finally:
        # NOTE: MotorsBus.disconnect() defaults to disabling torque.
        # For MODE "enable" and "hold" we want torque to remain on after the script exits,
        # so we explicitly keep it enabled on disconnect.
        bus.disconnect(disable_torque=(MODE == "disable"))


if __name__ == "__main__":
    main()