# MediaPipe Teleop Phase One

Phase one uses a Mac webcam and MediaPipe to generate a simple hand-teleop stream.

The Mac sends UDP packets with:

- `hand_xyz`: normalized palm-center position in the webcam frame
- `clutch`: pinch-to-enable state
- `tracked`: whether a hand is currently detected
- `confidence`: simple tracking confidence placeholder

On the Jetson, `rotom_teleop/mediapipe_teleop` receives those packets and
publishes `geometry_msgs/Twist` on `/rotom_servo/cartesian_cmd`, which reuses
the existing:

`twist_relay -> MoveIt Servo -> joint trajectory -> rotom_motor_bridge`

## Mac Setup

Create a small venv on the Mac:

```bash
just mac-teleop-venv
just mac-teleop-install
```

Get the current machine IP on either machine:

```bash
just ip-local
```

Run `just ip-local` on the Jetson to get the target host value for the Mac sender.
Run `just ip-local` on the Mac if you want to confirm which interface it is using locally.

Run the sender from the Mac clone of the repo:

```bash
just mac-teleop <jetson-ip>
```

- Press `q` to quit.
- Pinch thumb and index finger together to enable motion.
- Releasing the pinch sends `clutch=false`, so the Jetson teleop node zeros the command.

## Jetson Bringup

Build once after adding `rotom_teleop`:

```bash
just build
```

Run the teleop stack:

```bash
just teleop-hw
```

Or, if you want only the teleop node while the rest of the stack is already up:

```bash
just teleop-node
```

## Notes

- Phase one is translation-only.
- The robot-side node owns the actual speed caps, smoothing, deadband, and timeout.
- The safest tuning lives on the Jetson, not the Mac sender.
