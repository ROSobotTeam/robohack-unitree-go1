#!/bin/bash

if [ "${1:-}" = "ping" ]; then
    exec cat /opt/go1/ping.json
fi

printf '%s\n' 'Usage: /entrypoint.sh ping' >&2
exit 2
