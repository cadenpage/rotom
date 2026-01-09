# ROS 2 Humble + RViz on macOS (Native, No Docker)

This guide installs ROS 2 Humble on macOS using **Miniforge + mamba (RoboStack)**  
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

## 2. Install Miniforge (conda-forge based)

Download:
```bash
curl -L -O https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-arm64.sh
```

Install:
```bash
bash Miniforge3-MacOSX-arm64.sh
```

During install:
- Accept the license
- Accept default install location
- Say **yes** to shell initialization

Close and reopen the terminal when finished.

---

## 3. Ensure You Are Using Miniforge (Not Anaconda)

Activate Miniforge base:
```bash
source ~/miniforge3/bin/activate
```

Verify:
```bash
conda info | sed -n '1,5p'
```

Expected:
```
active env location : /Users/<username>/miniforge3
```

Do NOT install ROS into `/opt/anaconda3`.

---

## 4. Fix PATH for Miniforge / mamba

Edit zsh config:
```bash
nano ~/.zshrc
```

Add near the TOP:
```bash
export PATH="$HOME/miniforge3/bin:$PATH"
```

Reload:
```bash
source ~/.zshrc
```

Verify:
```bash
mamba --version
```

---

## 5. Create a Dedicated ROS 2 Environment

```bash
mamba create -n ros-humble -c conda-forge python=3.10
```

Activate:
```bash
conda activate ros-humble
```

Prompt should show:
```
(ros-humble)
```

---

## 6. Install ROS 2 Humble + RViz (RoboStack)

```bash
mamba install -c conda-forge -c robostack-staging ros-humble-desktop
```

Notes:
- Takes 10–30 minutes
- Large dependency list is normal
- Do not interrupt

---

## 7. Run RViz Locally on macOS

```bash
conda activate ros-humble
rviz2
```

Expected:
- RViz window opens natively
- Smooth GPU rendering
- No X11, no VNC, no Docker

---

## 8. Connect to Robot Over Network (Jetson, etc.)

On BOTH machines:
```bash
export ROS_DOMAIN_ID=42
```

(Optional: add to ~/.bashrc on Linux and ~/.zshrc on macOS.)

Verify discovery on macOS:
```bash
ros2 topic list
```

Robot topics should appear.

---

## 9. Daily Workflow

Motor control / non-ROS tools:
```bash
conda activate rotom
```

ROS visualization:
```bash
conda activate ros-humble
rviz2
```

ROS workspaces are just folders and are NOT tied to conda environments.


--
To have an easy configuration set up so you just need to run a simple command in the mac terminal to get it going, edit ~/.zshrc and add this snippit to automatically run or have the option to run this command each reset

## Key Idea

ROS is distributed.

The robot publishes data.  
The Mac visualizes it.


