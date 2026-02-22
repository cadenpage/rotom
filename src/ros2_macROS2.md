# ROS 2 Humble + RViz on macOS (Native, No Docker) — **micromamba** (RoboStack)

This guide installs ROS 2 Humble on macOS using **micromamba + conda-forge (RoboStack)**  
and runs **RViz locally on the Mac** (no X11, no NoMachine, no Docker).

This setup is ideal when:
- Your robot runs ROS 2 on Ubuntu (Jetson, etc.)
- Your Mac is used for visualization, debugging, and development

---

## System Assumptions

- macOS (Sequoia or newer)
- zsh shell (default on macOS)
- Apple Silicon (M1/M2/M3) or Intel
- Robot runs ROS 2 Humble on Ubuntu 22.04

---

## 1. Install Apple Command Line Tools

```bash
xcode-select --install
```

Verify:
```bash
xcode-select -p
```

Expected:
```
/Library/Developer/CommandLineTools
```

Full Xcode.app is NOT required.

---

## 2. Install **micromamba** (no Anaconda/Miniforge needed)

Pick the correct architecture:

- Apple Silicon: `osx-arm64`
- Intel Macs: `osx-64`

### Install (manual, recommended)

```bash
mkdir -p "$HOME/bin"
cd "$HOME/bin"

# Apple Silicon (arm64):
curl -L -o micromamba 'https://micro.mamba.pm/api/micromamba/osx-arm64/latest'

# Intel (x86_64):
# curl -L -o micromamba 'https://micro.mamba.pm/api/micromamba/osx-64/latest'

chmod +x micromamba
```

Verify:
```bash
"$HOME/bin/micromamba" --version
```

---

## 3. Initialize micromamba for zsh

Choose where environments/packages live (recommended):
```bash
mkdir -p "$HOME/micromamba"
```

Add this near the TOP of `~/.zshrc`:
```bash
nano ~/.zshrc
```

Add:
```bash
# micromamba
export MAMBA_ROOT_PREFIX="$HOME/micromamba"
export PATH="$HOME/bin:$PATH"
eval "$("$HOME/bin/micromamba" shell hook -s zsh)"
```

Reload:
```bash
source ~/.zshrc
```

Sanity check:
```bash
micromamba info
```

---

## 4. Create a Dedicated ROS 2 Environment

```bash
micromamba create -n ros_humble -c conda-forge -c robostack-staging python=3.10
```

Activate:
```bash
micromamba activate ros_humble
```

Prompt should show something like:
```
(ros-humble)
```

---

## 5. Install ROS 2 Humble + RViz (RoboStack)

```bash
micromamba install -n ros-humble -c conda-forge -c robostack-staging ros-humble-desktop
```

Notes:
- Takes 10–30 minutes
- Large dependency list is normal
- Do not interrupt

---

## 6. Run RViz Locally on macOS

```bash
micromamba activate ros_humble
rviz2
```

Expected:
- RViz window opens natively
- Smooth GPU rendering
- No X11, no VNC, no Docker

---

## 7. Connect to Robot Over Network (Jetson, etc.)

On BOTH machines:
```bash
export ROS_DOMAIN_ID=42
```

(Optional: add to `~/.bashrc` on Linux and `~/.zshrc` on macOS.)

Verify discovery on macOS:
```bash
ros2 topic list
```

Robot topics should appear.

---

## 8. Daily Workflow

Motor control / non-ROS tools (example):
```bash
micromamba activate rotom
```

ROS visualization:
```bash
micromamba activate ros_humble
rviz2
```

ROS workspaces are just folders and are NOT tied to micromamba environments.

---

## 9. Optional: one-command startup (zsh)

Add this to `~/.zshrc`:
```bash
# Quick ROS + RViz launcher
rosviz() {
  micromamba activate ros-humble || return 1
  export ROS_DOMAIN_ID=42
  rviz2
}
```

Then run:
```bash
rosviz
```

---

## Key Idea

ROS is distributed.

The robot publishes data.  
The Mac visualizes it.