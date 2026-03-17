
from motors import FeetechMotorsBus
from motors import Motor
from motors import SCAN_BAUDRATES, MODEL_NUMBER_TABLE
import time

def main():
    # Adjust these to your hardware
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

    bus = FeetechMotorsBus(
        port=PORT,
        motors={name: Motor(id=m_id, model=m_model, norm_mode="position") for name, (m_id, m_model) in MOTORS.items()},
        protocol_version=0,
    )

    # Open the port without handshake so we can scan reliably
    bus.connect(handshake=False)

    position = bus.read("Present_Position", "Base", normalize=False)
    torque = bus.read("Torque_Enable", "Base", normalize=False)
    mode = bus.read("Operating_Mode", "Base", normalize=False)
    print(f"Mode={mode}, Torque={torque}, Position={position}")
    few_steps = -500
    goal = position + few_steps

    # Keep goal within limits
    min_lim = bus.read("Min_Position_Limit", "my_motor", normalize=False)
    max_lim = bus.read("Max_Position_Limit", "my_motor", normalize=False)
    print(f"Position limits: {min_lim} to {max_lim}")
    goal = max(min_lim, min(goal, max_lim))

    print(f"Min={min_lim}, Max={max_lim}, Goal={goal}")
    bus.write("Goal_Position", "my_motor", goal, normalize=False)

    # Observe motion for a short period
    import time
    for _ in range(20):
        pos = bus.read("Present_Position", "my_motor", normalize=False)
        print(f"Present_Position={pos}")
        time.sleep(0.1)


    bus.disconnect()

if __name__ == "__main__":
    main()
