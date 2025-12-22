# ROS2 Humble Setup Guide

Assumes you already downloaded binary package of ROS2 humble on Ubuntu 22.04. I will be going off of video series from Robotics Back-End.

---

Update and upgrade any necessary packages

```bash
sudo apt update
sudo apt upgrate -y
```
---

Install the common add-ons for colcon build tool
```
sudo apt install python3-colcon-common-extensions
```
---
If you just use colcon, you will not be able to use autocompletion for the command line, so first we will see the files in the hook directory.
```
cd /usr/share/colcon_argcomplete/hook/
ls
```
Then edit the set up file
```
gedit ~/.bashrc
```
Ensure these two source lines are at the bottom of the set up file (We use the bash file you just viewed above as shown below).
```
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```
Create workspace folder in the src directory
```
cd ~/Documents/rotom/src
mkdir ros2_ws
cd ros2_ws
mkdir src
```
---
Build with colcon. Should show that 0 packages have finished. It should also create build, install, and log directories.
```
colcon build
```
---
In the install directory, there should be a file, setup.bash. Source this global filepath, then add the line to .bashrc as done above. You should have three lines at the bottom of the ~/.bashrc file after doing this.

```
source ~/Documents/rotom/src/ros2_ws/install/setup.bash
```
--- 
**We are now ready to start creating our application in ROS 2!**


