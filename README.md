# Rotom
Rotom is a custom 4-DoF robot arm project that combines mechanical design, low-level Feetech servo control, ROS 2 bringup, perception, custom kinematics, and real-time control on one stack. The platform runs on a Jetson Orin Nano and has now progressed well beyond a CAD + RViz prototype: it can both follow ArUco markers on the real robot and perform smooth MediaPipe teleoperation through a reduced task-space controller that I wrote for this mechanism.

<div style="display: flex; justify-content: center; gap: 20px;">
  <img src="imgs/rotomv2.png" alt="Rotom in real life" height="400"/>
  <img src="imgs/rotomv2_cad.png" alt="Rotom CAD model" height="400"/>
</div>

## Flagship Milestone: Smooth MediaPipe Task-Space Teleoperation
This is the most important result in the project right now. I can teleoperate the real robot from my Mac webcam using MediaPipe hand tracking, but instead of sending generic 3D or 6D twists through MoveIt Servo, I now command a reduced action space that matches the actual robot and the tabletop tasks I care about.

The live teleoperation stack is:

`Mac webcam -> MediaPipe -> UDP -> mediapipe_task_input -> /rotom_task/delta_cmd -> planar_task_controller -> /joint_commands -> rotom_motor_bridge -> motors`

What this proves:

- I now have a human teleoperation interface on the physical robot, not just autonomous tracking.
- The project has moved from generic ROS bringup into custom controller design.
- I have a task/action interface that is much better suited for collecting demonstrations for imitation learning.
- The same action space can later be reused for policies, rather than being only a hand-built demo path.

### Demo Placeholder
<div align="center">
  <img src="imgs/teleop.gif" alt="Smooth Teleoperation Demo" height="800"/>
</div>

## Why MoveIt Servo Was Not The Right Teleop Interface For This Arm
MoveIt Servo was very useful earlier in the project. It helped me validate the URDF, joint state bridge, motion execution, and a full ArUco-following perception-to-servo pipeline on real hardware. That work was important and still matters.

But for teleoperation and later learning, generic Cartesian twist control was the wrong abstraction for this specific robot:

- the arm only has 4 DoF, so full Cartesian twist behavior is overconstrained,
- world-frame and tool-frame twists were often unintuitive for tabletop manipulation,
- as joints approached limits, the realized motion would deviate from the requested cardinal direction,
- trying to control too much state at once made the motion noisy and hard to tune,
- and the controller path became harder to debug because perception, Servo behavior, Jacobian conditioning, and hardware dynamics were all mixed together.

In other words, MoveIt Servo proved that the system worked, but it was not the best control interface for the initial task I actually want: sliding and manipulating objects on a table, then collecting data for learning.

## Why The New Action Space Is Better
The new teleop path uses a lower-dimensional actionable space instead of generic twists:

- planar `x/y` motion on the table,
- fixed `z` for the current working plane,
- desired wrist pitch in the world frame,
- and a dependent wrist joint instead of treating every joint as independently commanded

That is a better fit for this arm because the mechanism is effectively:

- one base yaw joint `O`,
- a mostly planar `A/B/C` chain,
- and a tool whose useful behavior for my tasks is mostly defined by table-plane motion plus pitch

This makes the interface:

- easier for a human to drive,
- easier to stabilize,
- easier to reason about,
- and much better aligned with future imitation-learning action spaces

## How The New Controller Works
The current teleop controller is implemented in [planar_task_controller_node.py](/home/caden/Documents/rotom/src/ros2_ws/src/rotom_task_control/rotom_task_control/planar_task_controller_node.py) and [kinematics.py](/home/caden/Documents/rotom/src/ros2_ws/src/rotom_task_control/rotom_task_control/kinematics.py).

The robot-side flow is:

- `mediapipe_task_input`
  receives UDP packets from the Mac and publishes reduced task-space delta commands on `/rotom_task/delta_cmd`
- `planar_task_controller`
  maintains a desired task target and a smoothed commanded task target, solves reduced kinematics, and publishes `/joint_commands`
- `rotom_motor_bridge`
  converts joint commands into Feetech motor goals and republishes `/joint_states`

### Math Behind The Reduced Controller
I am no longer trying to command a full twist and hope a generic solver figures out the rest. Instead, I explicitly encode the task and the robot structure.

The task state is approximately:

- `s = [x_table, y_table, pitch_world]`

with:

- `z = z_fixed`

The controller maintains:

- a desired task target from teleop input,
- a commanded task target that moves toward it with velocity and acceleration limits,
- and a reduced joint solve over `O/A/B`

The reduced solve is essentially trying to minimize a cost of the form:

- `||p(q) - p_des||^2 + lambda_smooth ||Delta q||^2 + lambda_center ||q - q_mid||^2`

where:

- `p(q)` is the end-effector position from forward kinematics,
- `p_des` is the commanded planar task target,
- `Delta q` penalizes abrupt joint jumps,
- and `q_mid` biases the solution away from joint extremes

### Driven Wrist Pitch
One of the important controller changes was getting rid of the old per-cycle numeric wrist search.

For this robot, the world-frame tool pitch reduces to a simple analytic relationship:

- `pitch_world ~= A + B + C - pi/2`

So instead of treating `C` as another free solve variable, I now compute it directly:

- `C = pitch_desired - (A + B - pi/2)`

with wrapping and joint-limit projection.

That was a big improvement because it removed unnecessary search, reduced branch jitter, and matched the actual mechanism better.

## Tuning And Smoothing Challenges I Had To Solve
Getting the task-space teleop to feel good was not just a matter of increasing gains. I had to solve several different kinds of choppiness and instability.

### 1. Raw target chasing was too abrupt
If the controller directly chased every new desired target, step changes in task space looked jerky on the physical arm.

Fix:

- I added a velocity- and acceleration-limited task-space trajectory generator between the desired target and the commanded target.

### 2. Reacting directly to raw joint feedback caused choppy motion
Using the latest measured joint state as the full solver state every cycle made some directions, especially `x`, look stair-stepped.

Fix:

- I changed the controller to solve around a blended state between the last commanded joint configuration and the latest measured one.

### 3. Wrist pitch closure was overcomplicated
The older approach numerically searched for a valid `C` every cycle.

Fix:

- I replaced that search with the analytic driven-joint relation described above.

### 4. MediaPipe teleop needed its own tuning
The teleop feel depended heavily on:

- hand-to-task scaling,
- deadband,
- smoothing,
- clutch behavior,
- confidence thresholds,
- and the send/receive rates between the Mac and Jetson

Fix:

- I iteratively tuned the MediaPipe sender and the ROS task-input node until the hand motion was responsive without being unusably twitchy.

### 5. Speed and smoothness were different knobs
One of the more important lessons was that "gain" and "scale" are not the same thing anymore:

- MediaPipe scale changes how much hand motion turns into task-space motion
- controller gain changes how aggressively the arm chases the commanded target

Separating those two was a major improvement in usability.

## What This Unlocks For Learning
This new reduced action space is the reason I am comfortable moving toward learning-based control.

Instead of collecting demonstrations in an awkward twist space, I can now collect demonstrations in an action space that is:

- lower dimensional,
- physically meaningful,
- easier to teleoperate,
- and much closer to the structure I would actually want a policy to output

The near-term direction is:

- MediaPipe teleoperation for human demonstration collection
- imitation learning on the real robot, initially with diffusion policies
- possibly later reinforcement learning on top of the same reduced controller interface
- and eventually a more custom resolved-rate or optimization-based servo controller built around this same task-space formulation

## Major Prior Milestone: Real ArUco Marker Following
Before the new task-space teleoperation controller, the project's biggest result was real ArUco marker following on the physical robot using a full perception-to-servo-to-hardware pipeline:

`camera -> stereo_splitter -> aruco_tracker -> TF -> marker_follower -> /rotom_servo/cartesian_cmd -> twist_relay -> /servo_node/delta_twist_cmds -> MoveIt Servo -> /servo_node/joint_trajectory -> rotom_motor_bridge -> motors`

That milestone still matters because it proved:

- a real perception loop on live camera data,
- a functioning Cartesian command interface into MoveIt Servo,
- and a robot design that was mechanically stable enough for closed-loop servo behavior

### ArUco Demo Gallery
<table>
  <tr>
    <td align="center" width="33%">
      <strong>1. Initial high-gain behavior</strong><br/>
      <img src="imgs/aruco_oscillating.gif" alt="Initial oscillatory ArUco following" width="100%"/><br/>
      <sub>Fast, but oscillatory and choppy when marker tracking flickered.</sub>
    </td>
    <td align="center" width="33%">
      <strong>2. Retuned controller</strong><br/>
      <img src="imgs/aruco_retuned.gif" alt="Retuned ArUco following" width="100%"/><br/>
      <sub>Lower gains, lower caps, and a more realistic response for the physical arm.</sub>
    </td>
    <td align="center" width="33%">
      <strong>3. Final stable behavior</strong><br/>
      <img src="imgs/aruco_final.gif" alt="Final ArUco following result" width="100%"/><br/>
      <sub>Improved timeout handling and smoother closed-loop tracking on real hardware.</sub>
    </td>
  </tr>
</table>

### Engineering Challenges Overcome In The ArUco System
The hardest part was getting the closed-loop robot behavior to be trustworthy on the real arm. The infrastructure issues mattered, but the more important engineering work was solving the control and mechanism problems that showed up once the loop was actually running.

#### Control And Robot-Behavior Problems
- Oscillation and choppy motion:
  the first real marker-following runs were too aggressive and unstable. I had to retune gains, caps, and smoothing so the arm would still respond quickly without hunting around the target.
- Stale-target behavior:
  when the markers disappeared, the follower would keep reusing the last cached transform and continue sending the old motion command. I fixed that by enforcing transform-age checks and zeroing behavior when detections went stale.
- 10 fps perception constraint:
  the stable camera mode only ran at about 10 fps, which meant the stale-transform timeout and controller tuning had to match the real sensor rate. That was a major part of reducing the stop-go feel when detections flickered.
- Legacy singularity and geometry problems:
  the older version of the robot and control path would often end up near singular configurations, so the arm would either stall or behave unpredictably. The current redesign and newer control path removed those deeper geometric blockers.
- Legacy mechanical instability:
  the old lower-joint structure had oscillatory behavior that could not be solved just by adjusting motor PID values. The redesign fixed the hardware-side instability enough that controller tuning became meaningful.

#### Perception And Systems Problems
- Camera dropout and streaming instability:
  the stereo USB camera would periodically stall on `/image_raw`, which made the follower look broken even though the real fault was upstream in the image path. I had to diagnose the camera mode/encoding tradeoffs and stabilize the stream in a mode that the Jetson and ROS image stack could actually sustain.
- DDS participant-limit failure:
  the ROS graph would fail once the system grew beyond roughly nine active CycloneDDS participants. I traced that back to CycloneDDS participant indexing and fixed it with a clean ROS environment plus a custom CycloneDDS configuration.
- Middleware/interface conflict debugging:
  after fixing the DDS limit, I also hit loopback-interface configuration issues and had to sort out exactly how localhost-only ROS and CycloneDDS interface selection interacted.
- Debug-viewer vs live-stack conflicts:
  running debug tooling alongside the live stack originally started duplicate camera/tracker pipelines. I split standalone viewers from the active control pipeline so I could visualize the system without breaking it.
- Launch/environment problems on the Jetson:
  the stack originally had environment and launch-fragility problems, especially when mixing ROS, DDS configuration, and earlier environment-manager workflows. I reworked the bringup into cleaner reproducible commands.

## Foundational Milestones That Made These Results Possible
The current controller and teleop results sit on top of several earlier milestones that were important in their own right:

- low-level Feetech bus communication and safe motor bringup,
- joint-state readback and a ROS 2 motor bridge on the real arm,
- MoveIt 2 integration and RViz-based motion validation,
- remote visualization and development from macOS into the Jetson-hosted stack,
- camera calibration and perception bringup,
- and a mechanical redesign that fixed instability in the lower chain

These milestones are still important because they were the path from "robot model exists" to "the physical robot can actually do useful closed-loop behavior."

<table>
  <tr>
    <td align="center" width="33%">
      <strong>Interactive robot control</strong><br/>
      <img src="imgs/interactive_control.gif" alt="Interactive control demo" width="100%"/><br/>
      <sub>Early proof that the hardware and software stack could intentionally command the physical arm.</sub>
    </td>
    <td align="center" width="33%">
      <strong>MoveIt inverse kinematics</strong><br/>
      <img src="imgs/moveit_ik.gif" alt="MoveIt inverse kinematics demo" width="100%"/><br/>
      <sub>Proof that the robot description, planning configuration, and kinematic model were usable.</sub>
    </td>
    <td align="center" width="33%">
      <strong>MoveIt motion execution</strong><br/>
      <img src="imgs/moveit2.gif" alt="MoveIt motion execution demo" width="100%"/><br/>
      <sub>Proof that the real robot could execute higher-level motion before the project moved into custom control.</sub>
    </td>
  </tr>
</table>

## Major Challenges Overcome
### In The New Task-Space Teleop Path
- Moved from a generic MoveIt Servo teleop path to a custom reduced task-space controller that better matches the mechanism.
- Replaced a numeric driven-wrist search with an analytic dependent-joint closure.
- Added a task-space trajectory generator so teleop commands no longer become raw setpoint jumps.
- Separated teleop scale from controller aggressiveness so speed and stability could be tuned independently.
- Solved choppy controller behavior caused by raw feedback and updated the solver to blend measured and commanded state.
- Tuned MediaPipe clutch, confidence, deadband, smoothing, and scaling across a Mac-to-Jetson teleop link.

### In The Earlier ArUco/Vision Path
- Retuned a real closed-loop marker follower from oscillatory high-gain motion into stable usable behavior on the physical arm.
- Fixed stale-TF behavior so the arm no longer reused old marker poses after markers disappeared.
- Tuned the controller around a real 10 fps perception limit instead of assuming faster vision updates.
- Diagnosed camera dropouts and stabilized the camera path in a mode the full Jetson ROS stack could sustain.
- Solved legacy singularity and mechanism-stability problems that made earlier versions of the system hard to control.
- Diagnosed a CycloneDDS participant-limit issue that caused the ROS graph to fail once the stack grew beyond roughly nine active participants.
- Reworked launch and environment handling so the Jetson could run the full vision and control stack consistently.
- Split debug viewers from the active control stack so visualization could happen without duplicating the pipeline.

## Skills Demonstrated In This Project
- Mechanical redesign for stability, not just appearance
- Low-level smart-servo bus integration
- Custom robot kinematics and reduced-order controller design
- ROS 2 node/package design and launch composition
- Action-space design for teleoperation and learning
- Real-time perception and frame-graph reasoning
- Camera calibration, ArUco tracking, and visual servoing
- MoveIt 2 and MoveIt Servo integration on physical hardware
- Cross-machine teleoperation from macOS to Jetson
- Controller tuning across perception, middleware, kinematics, and actuation layers
- DDS and middleware debugging
- End-to-end systems debugging across hardware, drivers, ROS, and control software
- Demonstration-data tooling and dataset conversion for imitation learning

## Current Bringup Paths
The most relevant commands in the repository right now are:

```bash
just build
just reset
just task-home
just task-core
just task-teleop-hw
just vision-follow
just view-camera
just view-aruco
```

On the Mac teleop side:

```bash
just mac-teleop Berra
```

## Demonstration Collection
I now have a dedicated demonstration recorder path for collecting LeRobot-ready real-robot episodes. The important design choice is that I do **not** slow the controller itself down to 20 Hz. The task controller and teleop path can stay faster for smoother robot behavior, while the recorder samples the latest camera, action, and robot state at **20 Hz** for the dataset.

That gives me:
- a training-friendly fixed-rate dataset,
- small RGB images center-cropped to a square and resized to **112x112**,
- and a raw episode format that can still be re-converted later if I change the training stack.

### What Gets Recorded
At each 20 Hz sample, the recorder stores:
- camera frame from `/camera/selected/image_raw`
- camera intrinsics from `/camera/selected/camera_info`
- `/joint_states`
- `/joint_commands`
- `/rotom_task/delta_cmd`
- `/rotom_task/raw_hand_pose`
- `/rotom_task/teleop_enabled`
- `/rotom_task/controller_enabled`
- `/rotom_task/current_pose`, `/rotom_task/target_pose`, `/rotom_task/command_pose`
- `/rotom_task/current_pitch`, `/rotom_task/target_pitch`, `/rotom_task/command_pitch`
- episode-level success / failure labels and notes

Each raw episode is written under `data/demos/` as:
- `meta.json`
- `camera_info.json`
- `steps.jsonl`
- `images/frame_000000.jpg`, ...

The raw episodes are the source of truth. I then convert those into a **local LeRobot dataset** for training and inference.

### Record A Test Demo Today
Jetson, terminal 1:

```bash
just build
just reset
just task-home
just record-demo-hw
```

Jetson, terminal 2:

```bash
just record-start
just record-stop-success
just record-stop-failure
just record-discard
```

Mac:

```bash
just mac-teleop Berra
```

Recommended first test:
1. Start `record-demo-hw` on the Jetson.
2. If you want to see the live camera stream, open another Jetson terminal and run `just view-camera`.
3. Start `mac-teleop Berra` on the Mac.
4. Call `just record-start` on the Jetson.
5. Teleoperate one short block-slide or reach demo.
6. End with `just record-stop-success` or `just record-stop-failure`.

### Seeing What The Camera Sees
While the camera is running, the quickest full-resolution live viewer is:

```bash
just view-camera
```

If you want to see the exact processed `112x112` image that will be saved and converted for learning, run:

```bash
just view-dataset-camera
```

If you only want the camera without the full teleop/record stack, run:

```bash
just camera-stream
```

and in another terminal:

```bash
just view-camera
```

If you want to inspect the exact saved dataset frames, open the latest episode folder under `data/demos/<episode_id>/images/`. Those are the center-cropped 112x112 frames that will be converted into the LeRobot dataset.

### Convert To LeRobot Dataset
On the machine you want to train on, install the extra ML dependency layer:

```bash
just lerobot-install
```

Then convert the raw demos:

```bash
just convert-lerobot-dataset
```

By default that writes a local LeRobot dataset under:

```text
data/lerobot/local/rotom_task_teleop
```

with a deliberately minimal policy-facing interface:
- `observation.images.front`
- `observation.state`
- `action`

The default action exported for learning is the same reduced control space I am teleoperating with now:
- `action = [dx, dy]`

The default state exported is intentionally just the 4 measured arm joints:
- `state = [O, A, B, C]`

That keeps the policy aligned with the real controller interface while avoiding redundant state channels when I only have a small number of demos.

### Mac Training Workflow
I do **not** want large datasets in git. The clean workflow is:
- keep code in git,
- keep raw demos and LeRobot datasets out of git,
- sync `data/demos/` from the Jetson to the Mac,
- convert locally on the Mac,
- and train / infer there.

Example sync from the Mac:

```bash
rsync -av --progress Berra:~/Documents/rotom/data/demos/ ~/Documents/rotom/data/demos/
```

Then on the Mac:

```bash
just lerobot-install
just convert-lerobot-dataset
```

If the camera is still only producing about 10 fps in the stable mode, the recorder will still sample at 20 Hz, but some consecutive dataset frames will reuse the latest camera image while the robot/action state continues updating. That is acceptable for getting started today, and the raw episode metadata preserves the original image timestamps.

The current repo is now set up around one primary learning-data path:
- raw episodes in `data/demos/`
- local LeRobot dataset in `data/lerobot/`
- no replay-zarr conversion in the active workflow

For the best chance of useful same-day inference from only 10-20 demos, I should keep the data collection very consistent:
- one task only
- same camera viewpoint
- same planar home / reset pose
- same object and table layout
- mostly success-labeled demos

## Repo Map
- `src/motors`: low-level Feetech bus helpers and register access
- `src/examples`: homing, calibration, and one-off task-space helpers
- `src/ros2_ws/src/rotom_control`: ROS 2 bridge between ROS topics/actions and the physical motor bus
- `src/ros2_ws/src/rotom_data`: episode recorder and demonstration-collection launch files
- `src/ros2_ws/src/rotom_task_control`: reduced task-space controller, task-space teleop input, and kinematics
- `src/ros2_ws/src/rotom_vision`: camera split, ArUco tracking, marker following, and visual-servo logic
- `src/ros2_ws/src/rotom_servo`: twist relay interface into MoveIt Servo for the older Servo-based path
- `src/ros2_ws/src/rotom_moveit_config`: MoveIt configuration and bringup
- `src/ros2_ws/src/rotom_description`: URDF/Xacro, meshes, launch files, and RViz configs
- `scripts`: ROS environment isolation, DDS configuration, and helper wrappers for the current bringup flow
- `teleop`: Mac-side MediaPipe sender and teleop setup notes

## Hardware
- Compute: NVIDIA Jetson Orin Nano
- Actuation: Feetech STS3215 smart servos on a daisy-chained serial bus
- Vision: stereo USB camera
- Development host: macOS machine used for teleop, RViz, and remote workflows

## Software Stack
- ROS 2 Humble
- MoveIt 2 + MoveIt Servo
- Custom reduced task-space controller in `rotom_task_control`
- Python motor stack in `src/motors`
- OpenCV ArUco / ChArUco tooling
- MediaPipe hand tracking for teleoperation
- Repo-local `.venv` for non-ROS helper scripts, with system Python fallback
- Just for command shortcuts
- CycloneDDS for the stable multi-node ROS graph
- Zenoh + Tailscale for remote ROS visualization across machines

## Additional Documentation
- [Creating a Custom ROS 2 Package](docs/ros2_createpackage.md)
- [Installing ROS 2 on Mac](docs/ros2_macROS2.md)
- [MoveIt 2 Setup](docs/ros2_moveit.md)
- [Remote Visualization on macOS](docs/ros2_remoteVis.md)
- [Setting Up a ROS 2 Workspace](docs/ros2_workspace.md)
- [Jetson Headless Screen Sharing to macOS](docs/Nano_ScreenShare.md)
