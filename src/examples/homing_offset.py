"""
#.  python -m examples.homing_offset

Calibrate Homing_Offset for an arm (no CLI / no argument parsing).

This uses the built-in helper:
		MotorsBus.set_half_turn_homings(...)

NOTE: set_half_turn_homings() calls reset_calibration() first, which resets:
	- Homing_Offset to 0
	- Min_Position_Limit to 0
	- Max_Position_Limit to full range

Typical workflow:
1) MODE = "show" (prints current values)
2) Disable torque (separately) and move arm by hand to your desired mid pose
3) MODE = "set_half_turn" (writes calibration so current pose reads as midpoint)
"""

from motors import FeetechMotorsBus, Motor


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

# One of: "show" | "set_half_turn"
MODE = "set_half_turn"


def main() -> None:
	if MODE not in {"show", "set_half_turn"}:
		raise RuntimeError('MODE must be one of: "show", "set_half_turn"')

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
		if MODE == "show":
			for name in ARM:
				model = bus.motors[name].model
				res = bus.model_resolution_table[model]
				present = bus.read("Present_Position", name, normalize=False)
				offset = bus.read("Homing_Offset", name, normalize=False)
				print(f"{name}: model={model} res={res} present={present} homing_offset={offset}")
			return

		# MODE == "set_half_turn"
		# Homing/limit writes are safest with torque disabled.
		bus.disable_torque(ARM)
		offsets = bus.set_half_turn_homings(ARM)
		for name in ARM:
			present = bus.read("Present_Position", name, normalize=False)
			print(f"{name}: wrote_homing_offset={offsets[name]} now_present={present}")

		print("Half-turn homings written. Torque remains disabled.")
	finally:
		# Keep torque disabled on exit.
		bus.disconnect(disable_torque=True)


if __name__ == "__main__":
	main()

