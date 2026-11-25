#!/bin/bash
set -e

if [ "${1:-}" = "ping" ]; then
    exec cat /opt/go1/ping.json
fi

if [ "${1:-}" = "T5" ]; then
    source /opt/ros/humble/setup.bash
    exec ros2 launch /opt/go1/src/t5_launch.py
fi

printf '%s\n' 'Usage: /entrypoint.sh {ping|T5}' >&2
exit 2
