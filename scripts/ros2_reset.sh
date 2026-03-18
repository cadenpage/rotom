#!/bin/bash
set -eo pipefail

pkill -f 'ros2 launch rotom_' 2>/dev/null || true
pkill -f 'rotom_motor_bridge|twist_relay|stereo_splitter|aruco_tracker|marker_follower|servo_node_main|v4l2_camera_node|move_group|robot_state_publisher' 2>/dev/null || true

sleep 1

echo "ROS graph reset complete."
