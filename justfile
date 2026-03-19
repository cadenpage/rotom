set shell := ["bash", "-lc"]
repo_root := justfile_directory()

default:
  @just --list

venv-create:
  python3 -m venv {{repo_root}}/.venv

venv-install:
  {{repo_root}}/.venv/bin/pip install --upgrade pip
  {{repo_root}}/.venv/bin/pip install -r {{repo_root}}/requirements.txt

mac-teleop-venv:
  $(brew --prefix python@3.10)/bin/python3.10 -m venv {{repo_root}}/.venv

mac-teleop-install:
  {{repo_root}}/.venv/bin/python -m pip install --upgrade pip
  {{repo_root}}/.venv/bin/python -m pip install --no-compile -r {{repo_root}}/teleop/requirements-mac.txt

mac-teleop host port="8765" camera="0":
  {{repo_root}}/.venv/bin/python {{repo_root}}/teleop/mac_mediapipe_sender.py --host {{host}} --port {{port}} --camera {{camera}}

ip-local:
  python3 -c 'import socket; s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(("8.8.8.8", 80)); print(s.getsockname()[0]); s.close()'

# Core robot utilities (system Python + local source tree)
calibration-gui:
  /home/caden/Documents/rotom/scripts/repo_python_env.sh -m motors.calibration_gui

homing-offset:
  /home/caden/Documents/rotom/scripts/repo_python_env.sh /home/caden/Documents/rotom/src/examples/homing_offset.py

home:
  /home/caden/Documents/rotom/scripts/repo_python_env.sh /home/caden/Documents/rotom/src/examples/home.py

task-home:
  /home/caden/Documents/rotom/scripts/repo_python_env.sh /home/caden/Documents/rotom/src/examples/task_home.py

export-joint-limits:
  /home/caden/Documents/rotom/scripts/repo_python_env.sh /home/caden/Documents/rotom/src/examples/export_joint_limits.py

charuco-calibrate:
  DISPLAY="${DISPLAY:-:0}" QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}" /home/caden/Documents/rotom/scripts/ros2_clean_env.sh python /home/caden/Documents/rotom/src/examples/charuco_calibrate.py

calibrate-camera: charuco-calibrate

motor-bridge:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 run rotom_control rotom_motor_bridge

build:
  source /opt/ros/humble/setup.bash && cd /home/caden/Documents/rotom/src/ros2_ws && colcon build --symlink-install

reset:
  pkill -f '[r]os2 launch rotom_' 2>/dev/null || true
  pkill -f '[r]otom_motor_bridge|[t]wist_relay|[s]tereo_splitter|[a]ruco_tracker|[m]arker_follower|[s]ervo_node_main|[v]4l2_camera_node|[m]ove_group|[r]obot_state_publisher|[m]ediapipe_teleop|[p]lanar_task_controller|[m]ediapipe_task_input' 2>/dev/null || true
  sleep 1
  echo "ROS graph reset complete."

moveit-real:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_moveit_config real_hw.launch.py

moveit: moveit-real

servo-real:
  DISPLAY="${DISPLAY:-:0}" QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}" /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_moveit_config servo_hw.launch.py

servo-real-headless:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_moveit_config servo_hw.launch.py use_rviz:=false

twist-relay:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_servo twist_relay.launch.py

twist-relay-ground:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_servo twist_relay.launch.py command_frame:=ground

test-cartesian x="0.0" y="0.0" z="0.0" rate="10":
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 topic pub /rotom_servo/cartesian_cmd geometry_msgs/msg/Twist "{linear: {x: {{x}}, y: {{y}}, z: {{z}}}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r {{rate}}

pulse-cartesian x="0.0" y="0.0" z="0.0" duration="1.5" rate="10":
  bash -lc 'timeout {{duration}} just test-cartesian {{x}} {{y}} {{z}} {{rate}} >/dev/null 2>&1; status=$?; if [ "$status" -ne 0 ] && [ "$status" -ne 124 ]; then exit "$status"; fi'

test-x value="0.03" rate="10":
  just test-cartesian {{value}} 0.0 0.0 {{rate}}

test-y value="0.03" rate="10":
  just test-cartesian 0.0 {{value}} 0.0 {{rate}}

test-z value="0.03" rate="10":
  just test-cartesian 0.0 0.0 {{value}} {{rate}}

pulse-x value="0.01" duration="1.5" rate="10":
  just pulse-cartesian {{value}} 0.0 0.0 {{duration}} {{rate}}

pulse-y value="0.01" duration="1.5" rate="10":
  just pulse-cartesian 0.0 {{value}} 0.0 {{duration}} {{rate}}

pulse-z value="0.01" duration="1.5" rate="10":
  just pulse-cartesian 0.0 0.0 {{value}} {{duration}} {{rate}}

# Canonical vision entry points
vision:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_vision vision_pipeline.launch.py

vision-debug: vision

vision-camera: vision

vision-follow:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_vision vision_pipeline.launch.py enable_follower_output:=true

aruco-follow:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_moveit_config aruco_follow_hw.launch.py

teleop-node:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_teleop mediapipe_teleop.launch.py

teleop-hw:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_moveit_config teleop_hw.launch.py use_rviz:=false

task-core:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_task_control task_control_hw.launch.py

task-teleop-hw:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_task_control task_teleop_hw.launch.py

task-delta x="0.0" y="0.0" z="0.0" rate="10":
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 topic pub /rotom_task/delta_cmd geometry_msgs/msg/Vector3 "{x: {{x}}, y: {{y}}, z: {{z}}}" -r {{rate}}

task-set-target x y z:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 topic pub --once /rotom_task/target geometry_msgs/msg/PointStamped "{header: {frame_id: ground}, point: {x: {{x}}, y: {{y}}, z: {{z}}}}"

task-step dx dy dz="0.0":
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh python /home/caden/Documents/rotom/src/examples/task_step.py {{dx}} {{dy}} {{dz}}

task-step-x value="0.02":
  just task-step {{value}} 0.0

task-step-y value="0.02":
  just task-step 0.0 {{value}}

task-set-pitch value:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 topic pub --once /rotom_task/pitch_target std_msgs/msg/Float64 "{data: {{value}}}"

task-step-pitch value="0.10":
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh python /home/caden/Documents/rotom/src/examples/task_step_pitch.py {{value}}

table-slide-hw:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_moveit_config teleop_hw.launch.py use_rviz:=false table_slide_mode:=true

table-slide-core:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_moveit_config teleop_hw.launch.py use_rviz:=false table_slide_mode:=true include_teleop:=false

view-camera:
  DISPLAY="${DISPLAY:-:0}" QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}" /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_vision view_camera.launch.py

# Compatibility aliases
marker-follow:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_vision marker_follow.launch.py

aruco-tracker:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_vision aruco_tracker.launch.py

vision-pipeline: vision

view-selected: view-camera

view-aruco:
  DISPLAY="${DISPLAY:-:0}" QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}" /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_vision view_aruco.launch.py

rqt-images:
  DISPLAY="${DISPLAY:-:0}" QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}" /home/caden/Documents/rotom/scripts/ros2_clean_env.sh rqt_image_view

moveit-rviz:
  DISPLAY="${DISPLAY:-:0}" QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}" /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_moveit_config moveit_rviz.launch.py

view-robot:
  DISPLAY="${DISPLAY:-:0}" QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}" /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_description view_robot.launch.py
