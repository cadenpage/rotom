# ROS 2 Remote RViz over WAN using Zenoh (Jetson ↔ macOS)

This document records a **fully working, reproducible setup** for running a robot on a Jetson
and visualizing it with RViz on macOS over a VPN (Tailscale), using **Zenoh instead of DDS multicast discovery**.

DDS is confined to **localhost only** on each machine.
Zenoh is the **only cross-machine transport**.

This setup is robust across:
- WAN / VPN links
- NAT / campus networks
- Mixed architectures (Jetson ARM ↔ macOS)

---

## ARCHITECTURE

### Jetson (robot host)
- ROS 2 Humble
- `robot_state_publisher`
- `joint_state_publisher`
- `zenoh-bridge-ros2dds` (listener / router)
- DDS confined to localhost

### macOS (visualization host)
- ROS 2 Humble via RoboStack (conda)
- `zenoh-bridge-ros2dds` (client)
- RViz2
- DDS confined to localhost
```
Jetson DDS ──┐
│ (localhost only)
Zenoh TCP (7447 over Tailscale)
│
macOS DDS ───┘
```
---

## CORE RULES (DO NOT VIOLATE)

- `ROS_LOCALHOST_ONLY=1` on **both** machines
- **Different** `ROS_DOMAIN_ID` per machine
- Same `RMW_IMPLEMENTATION` everywhere
- No DDS peer XMLs or discovery servers
- ROS daemon disabled
- Zenoh is the **only** cross-machine transport

---

## JETSON SETUP (ROBOT HOST)

### Assumptions
- Miniforge3 installed
- Conda env: `rotom`
- Workspace: `~/Documents/rotom/src/ros2_ws`

---

### Jetson `~/.bashrc` (minimal, correct)

```bash
# === ROTOM / ROS2 BASE ENV (JETSON) ===

# Conda
if [ -f "$HOME/miniforge3/etc/profile.d/conda.sh" ]; then
  source "$HOME/miniforge3/etc/profile.d/conda.sh"
  conda activate rotom
fi

# ROS 2 Humble
source /opt/ros/humble/setup.bash

# Workspace overlay
if [ -f "$HOME/Documents/rotom/src/ros2_ws/install/setup.bash" ]; then
  source "$HOME/Documents/rotom/src/ros2_ws/install/setup.bash"
fi

# Zenoh-safe ROS defaults
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export ROS2_DISABLE_DAEMON=1

unset CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_DISCOVERY_SERVER
Jetson control helpers
rotom_up() {
  echo "[rotom_up] launching robot..."
  ros2 launch rotom_description view_robot.launch.py use_rviz:=false &

  echo "[rotom_up] starting zenoh listener on tcp/0.0.0.0:7447"
  zenoh-bridge-ros2dds -l tcp/0.0.0.0:7447 --no-multicast-scouting
}

rotom_down() {
  pkill -f "ros2 launch rotom_description" 2>/dev/null || true
  pkill -f zenoh-bridge-ros2dds 2>/dev/null || true
  pkill -f robot_state_publisher 2>/dev/null || true
  pkill -f joint_state_publisher 2>/dev/null || true
  echo "[rotom_down] stopped robot and zenoh"
}
```
### Jetson usage

```bash
source ~/.bashrc
rotom_up
rotom_down
```

### Rebuild description when URDF or meshes change:
```bash
cd ~/Documents/rotom/src/ros2_ws
colcon build --packages-select rotom_description
source install/setup.bash
```
# MACOS SETUP (RVIZ HOST)
### Assumptions
ROS 2 Humble via RoboStack (ros-humble conda env)
Workspace: ~/ros2_ws
Zenoh bridge installed via Homebrew
Tailscale hostname of Jetson: berra
Install Zenoh bridge
```bash
brew tap eclipse-zenoh/homebrew-zenoh
brew install zenoh-bridge-ros2dds
```
### Verify:
```bash
zenoh-bridge-ros2dds --version
Build description package (macOS)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <your_rotom_repo_url> rotom_description

cd ~/ros2_ws
conda activate ros-humble
colcon build --symlink-install
```
### macOS ~/.zshrc (working, zsh-correct)
Important: macOS uses setup.zsh, not setup.bash.

# === ROTOM / ROS2 BASE ENV (MAC) ===

```zsh
export ROTOM_JETSON_USER="caden"
export ROTOM_JETSON_HOST="berra"
export ROTOM_JETSON_DESC="/home/caden/Documents/rotom/src/ros2_ws/src/rotom_description"
export ROTOM_LOCAL_DESC="$HOME/ros2_ws/src/rotom_description"
export ROTOM_ZENOH_ENDPOINT="tcp/berra:7447"

zenoh_ros() {
  conda activate ros-humble

  # RoboStack ROS (zsh hooks)
  [ -f "$CONDA_PREFIX/setup.zsh" ] && source "$CONDA_PREFIX/setup.zsh"

  # Workspace overlay (zsh hooks)
  if [ -f "$HOME/ros2_ws/install/setup.zsh" ]; then
    source "$HOME/ros2_ws/install/setup.zsh"
  else
    echo "[zenoh_ros] ERROR: missing ros2_ws overlay"
    return 1
  fi

  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID=1
  export ROS_LOCALHOST_ONLY=1
  export ROS2_DISABLE_DAEMON=1

  unset CYCLONEDDS_URI
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  unset ROS_DISCOVERY_SERVER
}
```

### macOS control helpers
```zsh
rotom_up() {
  zenoh_ros || return 1

  echo "[rotom_up] syncing description package..."
  rsync -az --delete -e ssh \
    "${ROTOM_JETSON_USER}@${ROTOM_JETSON_HOST}:${ROTOM_JETSON_DESC}/" \
    "${ROTOM_LOCAL_DESC}/"

  echo "[rotom_up] rebuilding mac overlay..."
  (cd "$HOME/ros2_ws" && colcon build --packages-select rotom_description --symlink-install)
  source "$HOME/ros2_ws/install/setup.zsh"

  echo "[rotom_up] starting zenoh client..."
  pkill -f zenoh-bridge-ros2dds 2>/dev/null || true
  nohup zenoh-bridge-ros2dds -e "$ROTOM_ZENOH_ENDPOINT" --no-multicast-scouting client \
    > "$HOME/zenoh_mac_bridge.log" 2>&1 &

  echo "[rotom_up] starting RViz..."
  rviz2 -d "$(ros2 pkg prefix rotom_description --share)/rviz/view_robot.rviz"
}

rotom_down() {
  pkill -f zenoh-bridge-ros2dds 2>/dev/null || true
  pkill -f rviz2 2>/dev/null || true
}
```
# macOS usage
```zsh
source ~/.zshrc
rotom_up
rotom_down
```

# REQUIRED: Installing meshes & URDF correctly
To allow RViz to resolve:
package://rotom_description/meshes/...
Your rotom_description/CMakeLists.txt must include:

install(
  DIRECTORY
    meshes
    urdf
    rviz
    launch
  DESTINATION share/${PROJECT_NAME}
)

Always rebuild with:
```zsh
colcon build --packages-select rotom_description --symlink-install
```
### COMMON FAILURE MODES
TF visible but no robot: meshes not installed or overlay not sourced
Package not found: wrong shell (bash vs zsh) setup files
Zenoh client exits immediately: wrong CLI mode (client must be positional)
DDS UDP spam: ROS_LOCALHOST_ONLY not set
world frame missing: set RViz Fixed Frame to base_link
END STATE (VERIFIED)
Jetson publishes TF + joint states
macOS receives TF over Zenoh
RViz renders full robot with meshes
DDS confined to localhost
WAN-safe, reproducible, debuggable workflow