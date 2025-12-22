"""
#.  python -m examples.change_id

Minimal helper to change a motor's ID (no CLI / no argument parsing).

SAFETY:
- Disconnect all other motors from the bus first.
- Set the correct MODEL for the motor you're changing.
- If you know the current ID/baudrate, set CURRENT_ID / CURRENT_BAUDRATE to avoid scanning.

This uses MotorsBus.setup_motor(), which:
- disables torque,
- writes the new ID,
- writes the bus default baudrate,
- and returns the bus to its default baudrate.
"""

from motors import FeetechMotorsBus, Motor


# --- EDIT THESE ---
PORT = "/dev/ttyACM0"
MODEL = "sts3215"

# The ID you want the motor to have after running this.
TARGET_ID = 6

# If you know the motor's current settings, fill these in to avoid scanning.
# Otherwise leave as None and the code will scan IDs at supported baudrates.
CURRENT_ID = None  # e.g. 1
CURRENT_BAUDRATE = None  # e.g. 1_000_000


def main() -> None:
    # The bus needs to know the *desired* ID and model for the named motor.
    bus = FeetechMotorsBus(
        port=PORT,
        motors={"m": Motor(id=TARGET_ID, model=MODEL, norm_mode="position")},
        protocol_version=0,
    )

    try:
        bus.setup_motor("m", initial_baudrate=CURRENT_BAUDRATE, initial_id=CURRENT_ID)
        print(f"Changed motor ID to {TARGET_ID} on {PORT}")
    finally:
        # setup_motor may connect implicitly
        if bus.is_connected:
            bus.disconnect(disable_torque=True)


if __name__ == "__main__":
    main()
