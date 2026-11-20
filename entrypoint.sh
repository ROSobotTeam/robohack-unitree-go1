#!/bin/bash
set -e

if [ "${1:-}" = "ping" ]; then
    exec cat /opt/go1/ping.json
fi

if [ "${1:-}" = "T2" ]; then
    source /opt/ros/humble/setup.bash
    source /opt/unitree_go/share/unitree_go/local_setup.bash
    source /opt/go1_ws/install/setup.bash
    exec ros2 run safety_block safety_node --ros-args -p use_sim_time:=true
fi

printf '%s\n' 'Usage: /entrypoint.sh {ping|T2}' >&2
exit 2
