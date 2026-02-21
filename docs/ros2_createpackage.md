# I am going to cover how to make your own custom package in ROS 2 and write first node

Create Package with the python build type. The first package we want to create is rotom_description, so we can tell ROS the geometries and constraints of our robot. I will add the minimal dependency of rclpy for now, but more will be needed for URDF and xarco(later it will be something like rotom_control, rotom_moveit, rotom_bringup)
```
cd ~/Documents/rotom/src/ros2_ws/src/
ros2 pkg create rotom_description --build-type ament_python --dependencies rclpy
```
---
Now we need to create the launch file and urdf file in order to be able to view our robot in rviz. this takes a few steps

First, create this file structure in the rotom_description package you just created:

```
cd ~/Documents/rotom/src/ros2_ws/src/rotom_description
mkdir -p launch urdf meshes rviz
```
Then Create the files for each like so
```
touch urdf/rotom.urdf.xacro
touch launch/view_robot.launch.py
```

```
ros2_ws/
  src/
    rotom_description/
      package.xml
      setup.py
      resource/rotom_description
      rotom_description/        # python module (leave this alone mostly)
      launch/                  # launch files live here
        view_robot.launch.py
      urdf/                    # URDF / Xacro live here
        rotom.urdf.xacro       
      meshes/                  # STL/DAE/etc if you use them
        visuals/
          {all of your STL's}
      rviz/                    # optional: rviz config
        view_robot.rviz
```
---
In the launch file, we want to add a joint state publisher node and an rviz node.

The joint state publisher takes Xacro via the parameter and publishes the string to the /robot_description topic, and it publishes the tf on /tf and /tf_static

The RViz node just allows you to view your robot after you set up what topic and file you are gathering your data from

---
In the setup.py file, make sure to add files to the data file section that you want on the share install path such as

```
(os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
  (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
  (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
  (os.path.join('share', package_name, 'meshes', 'visual'), glob('meshes/visual/*')),
  (os.path.join('share', package_name, 'meshes', 'collision'), glob('meshes/collision/*')),
```
---
Finally, you will also need to create your xacro file. It is just like an URDF, with a few more details to add. Not really sure what they are called but it is better than urdf for modularity and makes it easy to scale complex robots where you are inputting your own meshes like I want to.

---

Go back to the workspace directory and build with colcon. If this throws a warning, it is likely that you need to downgrade your setuptools version (pip3 list | grep setuptools) to 58.2.0 (pip3 install setuptools==58.2.0)
```
cd ~/Documents/rotom/src/ros2_ws
rm -rf build install log
colcon build
source ~/.bashrc
```
Then launch the launch file
```
ros2 launch rotom_description view_robot.launch.py
```
Again, you will need to configure the Rviz setup to get your meshes to appear, by scanging the source description and source file I believe.