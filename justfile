set shell := ["bash", "-lc"]
repo_root := justfile_directory()

default:
  @just --list

venv-create:
  python3 -m venv {{repo_root}}/.venv

venv-install:
  {{repo_root}}/.venv/bin/pip install --upgrade pip
  {{repo_root}}/.venv/bin/pip install -r {{repo_root}}/requirements.txt

lerobot-install:
  {{repo_root}}/.venv/bin/python -m pip install --upgrade pip
  {{repo_root}}/.venv/bin/python -m pip install -r {{repo_root}}/requirements-lerobot.txt

lerobot-install-jetson-cuda:
  {{repo_root}}/.venv/bin/python -m pip install --upgrade pip
  {{repo_root}}/.venv/bin/python -m pip install -r {{repo_root}}/requirements-lerobot.txt
  {{repo_root}}/.venv/bin/python -m pip uninstall -y torch torchvision torchaudio || true
  {{repo_root}}/.venv/bin/python -m pip install 'numpy<2'
  {{repo_root}}/.venv/bin/python -m pip install --no-deps --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 torch==2.8.0 torchvision==0.23.0

jetson-torch-check:
  {{repo_root}}/.venv/bin/python -c "import torch; print('torch', torch.__version__); print('cuda_available', torch.cuda.is_available()); print('torch_cuda', torch.version.cuda); print('device_count', torch.cuda.device_count()); print('device_name', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'n/a')"

hf-login:
  {{repo_root}}/.venv/bin/python -m huggingface_hub.commands.huggingface_cli login

hf-whoami:
  {{repo_root}}/.venv/bin/python -m huggingface_hub.commands.huggingface_cli whoami

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
  pkill -f '[r]otom_motor_bridge|[t]wist_relay|[s]tereo_splitter|[a]ruco_tracker|[m]arker_follower|[s]ervo_node_main|[v]4l2_camera_node|[m]ove_group|[r]obot_state_publisher|[m]ediapipe_teleop|[p]lanar_task_controller|[m]ediapipe_task_input|[p]olicy_inference|[e]pisode_recorder' 2>/dev/null || true
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

camera-stream:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_vision camera_stream.launch.py

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

policy-node policy_repo_id="caden-ut/rotom_task_teleop_diffusion" local_policy_path="/home/caden/Documents/rotom/policies/rotom_task_teleop_diffusion/pretrained_model":
  /home/caden/Documents/rotom/scripts/ros2_repo_python_env.sh -m rotom_task_control.policy_inference_node --ros-args --params-file /home/caden/Documents/rotom/src/ros2_ws/src/rotom_task_control/config/policy_inference.yaml -p policy_repo_id:={{policy_repo_id}} -p local_policy_path:={{local_policy_path}}

policy-hw policy_repo_id="caden-ut/rotom_task_teleop_diffusion" local_policy_path="/home/caden/Documents/rotom/policies/rotom_task_teleop_diffusion/pretrained_model" start_enabled="false" camera_pixel_format="YUYV" camera_image_size="[1280,480]" camera_output_encoding="mono8":
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_task_control task_policy_hw.launch.py policy_repo_id:={{policy_repo_id}} local_policy_path:={{local_policy_path}} start_enabled:={{start_enabled}} camera_pixel_format:={{camera_pixel_format}} camera_image_size:="{{camera_image_size}}" camera_output_encoding:={{camera_output_encoding}}

policy-start:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 service call /rotom_policy/start std_srvs/srv/Trigger "{}"

policy-stop:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 service call /rotom_policy/stop std_srvs/srv/Trigger "{}"

record-node:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_data episode_recorder.launch.py

record-demo-core:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_data demo_record_core.launch.py

record-demo-hw:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 launch rotom_data demo_record_hw.launch.py

record-start:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 service call /rotom_dataset/start_episode std_srvs/srv/Trigger "{}"

record-stop:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 service call /rotom_dataset/stop_episode std_srvs/srv/Trigger "{}"

record-stop-success:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 service call /rotom_dataset/stop_episode_success std_srvs/srv/Trigger "{}"

record-stop-failure:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 service call /rotom_dataset/stop_episode_failure std_srvs/srv/Trigger "{}"

record-discard:
  /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 service call /rotom_dataset/discard_episode std_srvs/srv/Trigger "{}"

annotate-episode episode success="" task_id="" failure_code="" notes="":
  bash -lc 'args=(); if [[ -n "{{success}}" ]]; then args+=(--success "{{success}}"); fi; if [[ -n "{{task_id}}" ]]; then args+=(--task-id "{{task_id}}"); fi; if [[ -n "{{failure_code}}" ]]; then args+=(--failure-code "{{failure_code}}"); fi; if [[ -n "{{notes}}" ]]; then args+=(--notes "{{notes}}"); fi; /home/caden/Documents/rotom/scripts/repo_python_env.sh /home/caden/Documents/rotom/src/examples/annotate_episode.py "{{episode}}" "${args[@]}"'

convert-lerobot-dataset raw_root="data/demos" repo_id="local/rotom_task_teleop" root="data/lerobot" success_only="true" task="table_teleop" action_mode="xy" force="false":
  {{repo_root}}/scripts/repo_python_env.sh {{repo_root}}/src/examples/convert_to_lerobot_dataset.py --raw-root {{raw_root}} --repo-id {{repo_id}} --root {{root}} --task {{task}} --action-mode {{action_mode}} {{ if success_only == "true" { "--success-only" } else { "" } }} {{ if force == "true" { "--force" } else { "" } }}

convert-lerobot-hub repo_id:
  {{repo_root}}/scripts/repo_python_env.sh {{repo_root}}/src/examples/convert_to_lerobot_dataset.py --raw-root data/demos --repo-id {{repo_id}} --root data/lerobot --task table_teleop --action-mode xy --push-to-hub --success-only

convert-lerobot-hub-force repo_id:
  {{repo_root}}/scripts/repo_python_env.sh {{repo_root}}/src/examples/convert_to_lerobot_dataset.py --raw-root data/demos --repo-id {{repo_id}} --root data/lerobot --task table_teleop --action-mode xy --push-to-hub --success-only --force

convert-lerobot-hub-private repo_id:
  {{repo_root}}/scripts/repo_python_env.sh {{repo_root}}/src/examples/convert_to_lerobot_dataset.py --raw-root data/demos --repo-id {{repo_id}} --root data/lerobot --task table_teleop --action-mode xy --push-to-hub --success-only --force --private

pull-lerobot-dataset repo_id:
  {{repo_root}}/scripts/repo_python_env.sh {{repo_root}}/src/examples/pull_lerobot_dataset.py --repo-id {{repo_id}} --root data/lerobot_hub

pull-lerobot-dataset-force repo_id:
  {{repo_root}}/scripts/repo_python_env.sh {{repo_root}}/src/examples/pull_lerobot_dataset.py --repo-id {{repo_id}} --root data/lerobot_hub --force

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

view-dataset-camera:
  DISPLAY="${DISPLAY:-:0}" QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}" /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 run rotom_vision image_monitor --ros-args -p image_topic:=/rotom_dataset/image_resized -p window_name:="Rotom Dataset Camera" -p max_width:=448

view-policy-camera:
  DISPLAY="${DISPLAY:-:0}" QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}" /home/caden/Documents/rotom/scripts/ros2_clean_env.sh ros2 run rotom_vision image_monitor --ros-args -p image_topic:=/rotom_policy/image_input -p window_name:="Rotom Policy Camera" -p max_width:=448

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
