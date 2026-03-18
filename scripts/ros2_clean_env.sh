#!/bin/bash
set -eo pipefail

unset CONDA_PREFIX
unset LD_LIBRARY_PATH
unset PYTHONHOME
unset PYTHONPATH
unset ROS_IP
unset ROS_HOSTNAME
unset ROS_DISCOVERY_SERVER
unset ROS_AUTOMATIC_DISCOVERY_RANGE
unset CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset FASTDDS_DEFAULT_PROFILES_FILE
unset PIXI_ENVIRONMENT_NAME
unset PIXI_ENV_ROOT
unset PIXI_EXE
unset PIXI_IN_SHELL
unset PIXI_PROJECT_MANIFEST
unset PIXI_PROJECT_NAME
unset PIXI_PROJECT_ROOT

export PATH="/usr/bin:/bin"

if [[ -n "${DISPLAY:-}" ]]; then
  export DISPLAY
fi
if [[ -n "${QT_QPA_PLATFORM:-}" ]]; then
  export QT_QPA_PLATFORM
fi
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/roslogs}"
export PYTHONUNBUFFERED=1
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
unset FASTDDS_BUILTIN_TRANSPORTS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/caden/Documents/rotom/scripts/cyclonedds.xml

source /opt/ros/humble/setup.bash
source /home/caden/Documents/rotom/src/ros2_ws/install/setup.bash

export PYTHONPATH=/home/caden/Documents/rotom/src${PYTHONPATH:+:$PYTHONPATH}

exec "$@"
