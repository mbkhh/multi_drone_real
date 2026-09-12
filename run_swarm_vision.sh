#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
vision_environment="$workspace_root/.venv-swarm-vision"
ros_setup="${ROS_SETUP:-/opt/ros/rolling/setup.bash}"
node_name="${1:-flight}"

if [[ ! -f "$ros_setup" ]]; then
    echo "ROS setup not found: $ros_setup" >&2
    echo "Set ROS_SETUP to your ROS distribution setup.bash." >&2
    exit 1
fi
if [[ ! -x "$vision_environment/bin/python" ]]; then
    echo "Vision environment is missing: $vision_environment" >&2
    echo "Follow src/swarm_vision/README.md to create it." >&2
    exit 1
fi
if [[ ! -f "$workspace_root/install/setup.bash" ]]; then
    echo "Workspace is not built. Run: colcon build --packages-select swarm_vision" >&2
    exit 1
fi

case "$node_name" in
    flight)
        module='swarm_vision.flight_vision_node'
        ;;
    web)
        module='swarm_vision.vision_node'
        ;;
    *)
        echo "Usage: $0 {flight|web} [ROS arguments...]" >&2
        exit 2
        ;;
esac
shift || true

# PYTHONNOUSERSITE prevents packages in ~/.local from leaking into this node.
source "$ros_setup"
source "$workspace_root/install/setup.bash"
export PYTHONNOUSERSITE=1
export ORT_DISABLE_TELEMETRY=1
exec "$vision_environment/bin/python" -m "$module" "$@"
