#!/usr/bin/env bash
set -eo pipefail
source /opt/occupancy-joining/install/setup.bash
if [[ "${1:-}" == exec ]]; then shift; exec "$@"; fi
exec /bin/bash /opt/occupancy-joining/tools/occupancy-joining "$@"
