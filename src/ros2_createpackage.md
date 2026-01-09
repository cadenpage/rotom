# I am going to cover how to make your own custom package in ROS 2 and write first node

Create Package with the python build type. The first package we want to create is rotom_description, so we can tell ROS the geometries and constraints of our robot. I will add the minimal dependency of rclpy for now, but more will be needed for URDF and xarco(later it will be something like rotom_control, rotom_moveit, rotom_bringup)
```
cd ~/Documents/rotom/src/ros2_ws/src/
ros2 pkg create rotom_description --build-type ament_python --dependencies rclpy
```
# Now we need to create the launch file and urdf file in order to be able to view our robot in rviz. this takes a few steps

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



Go back to the workspace directory and build with colcon. If this throws a warning, it is likely that you need to downgrade your setuptools version (pip3 list | grep setuptools) to 58.2.0 (pip3 install setuptools==58.2.0)
```
cd ~/Documents/rotom/src/ros2_ws
rm -rf build install log
colcon build
```


```cd 
ros2 launch rotom_description view_robot.launch.py
```