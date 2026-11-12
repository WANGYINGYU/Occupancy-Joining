#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd "$(dirname "$0")/.." && pwd)"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
cd "$project_dir"
colcon build --base-paths "$project_dir/src" --build-base "$project_dir/build" --install-base "$project_dir/install" --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
