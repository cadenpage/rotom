
from motors import FeetechMotorsBus
from motors import Motor
from motors import SCAN_BAUDRATES, MODEL_NUMBER_TABLE
import time

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
bus.connect(handshake=False)

goal_position = 2047
for name in ARM:
    model = bus.motors[name].model
    bus.write("Goal_Position", name, goal_position, normalize=False)
    time.sleep(1)

    print("robot homed to center position")

bus.disconnect()