set shell := ["bash", "-lc"]

default:
  @just --list

venv-create:
  /usr/bin/python3 -m venv /home/caden/Documents/rotom/.venv

venv-install:
  /home/caden/Documents/rotom/.venv/bin/pip install --upgrade pip
  /home/caden/Documents/rotom/.venv/bin/pip install -r /home/caden/Documents/rotom/requirements.txt

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
  /home/caden/Documents/rotom/scripts/ros2_gui_env.sh python /home/caden/Documents/rotom/src/examples/charuco_calibrate.py

calibrate-camera:
  /home/caden/Documents/rotom/scripts/ros2_gui_env.sh python /home/caden/Documents/rotom/src/examples/charuco_calibrate.py

ros2-motor-bridge:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 run rotom_control rotom_motor_bridge

ros2-build:
  source /opt/ros/humble/setup.bash && cd /home/caden/Documents/rotom/src/ros2_ws && colcon build --symlink-install

ros2-reset:
  /home/caden/Documents/rotom/scripts/ros2_reset.sh

ros2-moveit-real:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_moveit_config real_hw.launch.py

ros2-servo-real:
  /home/caden/Documents/rotom/scripts/ros2_gui_env.sh ros2 launch rotom_moveit_config servo_hw.launch.py

ros2-servo-real-headless:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_moveit_config servo_hw.launch.py use_rviz:=false

ros2-twist-relay:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_servo twist_relay.launch.py

# Canonical vision entry points
ros2-vision:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_vision vision_pipeline.launch.py

ros2-vision-debug:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_vision vision_pipeline.launch.py

ros2-vision-camera:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_vision vision_pipeline.launch.py

ros2-vision-follow:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_vision vision_pipeline.launch.py enable_follower_output:=true

ros2-aruco-follow:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_moveit_config aruco_follow_hw.launch.py

ros2-view-camera:
  /home/caden/Documents/rotom/scripts/ros2_gui_env.sh ros2 launch rotom_vision view_camera.launch.py

# Compatibility aliases
ros2-marker-follow:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_vision marker_follow.launch.py

ros2-aruco-tracker:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_vision aruco_tracker.launch.py

ros2-vision-pipeline:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_vision vision_pipeline.launch.py

ros2-view-selected:
  /home/caden/Documents/rotom/scripts/ros2_gui_env.sh ros2 launch rotom_vision view_camera.launch.py

ros2-view-aruco:
  /home/caden/Documents/rotom/scripts/ros2_gui_env.sh ros2 launch rotom_vision view_aruco.launch.py

ros2-rqt-images:
  /home/caden/Documents/rotom/scripts/ros2_gui_env.sh rqt_image_view

ros2-moveit-rviz:
  /home/caden/Documents/rotom/scripts/ros2_gui_env.sh ros2 launch rotom_moveit_config moveit_rviz.launch.py

ros2-view-robot:
  /home/caden/Documents/rotom/scripts/ros2_gui_env.sh ros2 launch rotom_description view_robot.launch.py
