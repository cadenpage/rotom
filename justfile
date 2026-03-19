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
  python3 -m venv {{repo_root}}/.venv

mac-teleop-install:
  {{repo_root}}/.venv/bin/python -m pip install --upgrade pip
  {{repo_root}}/.venv/bin/python -m pip install -r {{repo_root}}/teleop/requirements-mac.txt

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
  pkill -f 'ros2 launch rotom_' 2>/dev/null || true
  pkill -f 'rotom_motor_bridge|twist_relay|stereo_splitter|aruco_tracker|marker_follower|servo_node_main|v4l2_camera_node|move_group|robot_state_publisher' 2>/dev/null || true
  sleep 1
  echo "ROS graph reset complete."

moveit-real:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_moveit_config real_hw.launch.py

servo-real:
  DISPLAY="${DISPLAY:-:0}" QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}" /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_moveit_config servo_hw.launch.py

servo-real-headless:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_moveit_config servo_hw.launch.py use_rviz:=false

twist-relay:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_servo twist_relay.launch.py

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
