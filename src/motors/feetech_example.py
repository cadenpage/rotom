
# Copyright 2025 cadenpage.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from motors.feetech.feetech import FeetechMotorsBus
from motors.motors_bus import Motor
from motors.feetech.tables import SCAN_BAUDRATES, MODEL_NUMBER_TABLE


def main():
    # Adjust these to your hardware
    port = "/dev/ttyACM0"
    motor_id = 4
    motor_model = "sts3215"

    bus = FeetechMotorsBus(
        port=port,
        motors={"my_motor": Motor(id=motor_id, model=motor_model, norm_mode="position")},
        protocol_version=0,
    )

    # Open the port without handshake so we can scan reliably
    bus.connect(handshake=False)
        # Ensure correct mode and torque before moving
        # Some units ignore commands if torque is disabled or mode mismatches.
        # Avoid locking config while debugging.
    # bus.write("Operating_Mode", "my_motor", 0)  # POSITION mode
    bus.write("Torque_Enable", "my_motor", 0)
    # bus.write("Lock", "my_motor", 0)

        # Proceed with normal operations
    position = bus.read("Present_Position", "my_motor", normalize=False)
    torque = bus.read("Torque_Enable", "my_motor", normalize=False)
    mode = bus.read("Operating_Mode", "my_motor", normalize=False)
    print(f"Mode={mode}, Torque={torque}, Position={position}")
    few_steps = 500
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
