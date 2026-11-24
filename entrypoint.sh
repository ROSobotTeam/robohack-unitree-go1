#!/bin/bash
set -e

case "${1:-}" in
    ping)
        exec cat /opt/go1/ping.json
        ;;
    T2|T3)
        task=$1
        shift
        source /opt/ros/humble/setup.bash
        source /opt/unitree_ws/install/local_setup.bash
        if [ "$task" = T2 ]; then
            exec python3 /opt/go1/safety_block.py "$@"
        fi
        # Keep the small policy matrices from competing with collision checking.
        export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
        python3 /opt/go1/t3_controller.py safety &
        safety_pid=$!
        python3 /opt/go1/t3_controller.py policy &
        policy_pid=$!

        stop_nodes() {
            kill "$safety_pid" "$policy_pid" 2>/dev/null || true
            wait "$safety_pid" "$policy_pid" 2>/dev/null || true
        }
        trap stop_nodes EXIT
        trap 'exit 130' INT
        trap 'exit 143' TERM
        wait -n "$safety_pid" "$policy_pid"
        ;;
    *)
        printf '%s\n' 'Usage: /entrypoint.sh ping|T2|T3' >&2
        exit 2
        ;;
esac
