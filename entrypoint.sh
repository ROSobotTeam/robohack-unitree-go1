#!/bin/bash
set -e

case "${1:-}" in
    ping)
        exec cat /opt/go1/ping.json
        ;;
    T2)
        shift
        source /opt/ros/humble/setup.bash
        source /opt/unitree_ws/install/local_setup.bash
        exec python3 /opt/go1/safety_block.py "$@"
        ;;
    *)
        printf '%s\n' 'Usage: /entrypoint.sh ping|T2' >&2
        exit 2
        ;;
esac
