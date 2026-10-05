#!/usr/bin/env bash
# ROS setup scripts read some optional variables before defining them, so
# nounset (`set -u`) cannot be active while they are sourced.
set -eo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ros_setup="${ROS_SETUP:-/opt/ros/rolling/setup.bash}"

if [[ ! -f "$ros_setup" ]]; then
    echo "ROS setup not found: $ros_setup" >&2
    exit 1
fi
if [[ ! -f "$workspace_root/install/setup.bash" ]]; then
    echo "Workspace is not built. Build swarm_flight_launcher first." >&2
    exit 1
fi

unset ROS_DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source "$ros_setup"
source "$workspace_root/install/setup.bash"
cd "$workspace_root"
exec ros2 run swarm_flight_launcher launcher

