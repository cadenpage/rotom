#!/bin/bash
set -eo pipefail

export DISPLAY="${DISPLAY:-:0}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

exec /home/caden/Documents/rotom/scripts/ros2_clean_env.sh "$@"
