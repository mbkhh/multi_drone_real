#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
    cat <<'EOF'
Usage: sudo ./deployment/systemd/install.sh [--user USER] [--workspace PATH]

Installs the swarm-drone systemd units and runner. It deliberately does not
enable or start the target. Review /etc/swarm-drone.conf and perform a safe
bench test first.
EOF
}

service_user="${SUDO_USER:-mbkh}"
workspace=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --user)
            [[ $# -ge 2 ]] || { usage >&2; exit 2; }
            service_user="$2"
            shift 2
            ;;
        --workspace)
            [[ $# -ge 2 ]] || { usage >&2; exit 2; }
            workspace="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
    echo "Run this installer with sudo." >&2
    exit 1
fi

if [[ ! "$service_user" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]]; then
    echo "Invalid service user: $service_user" >&2
    exit 2
fi
if ! getent passwd "$service_user" >/dev/null; then
    echo "User does not exist: $service_user" >&2
    exit 2
fi

if [[ -z "$workspace" ]]; then
    user_home="$(getent passwd "$service_user" | cut -d: -f6)"
    workspace="$user_home/multi_drone_real"
fi
if [[ "$workspace" != /* || "$workspace" == *$'\n'* ]]; then
    echo "Workspace must be an absolute path: $workspace" >&2
    exit 2
fi
if [[ ! -d "$workspace/deployment/systemd" ]]; then
    echo "Deployment files not found below workspace: $workspace" >&2
    exit 1
fi

readonly source_dir="$workspace/deployment/systemd"
readonly unit_dir=/etc/systemd/system
readonly runner_path=/usr/local/libexec/swarm-drone-run
readonly config_path=/etc/swarm-drone.conf

install -D -o root -g root -m 0755 \
    "$source_dir/swarm-drone-run" "$runner_path"

for unit in \
    swarm-xrce-agent.service \
    swarm-p900-bridge.service \
    swarm-control.service \
    swarm-vision.service
do
    temporary="$(mktemp)"
    trap 'rm -f "${temporary:-}"' EXIT
    sed \
        -e "s|@SWARM_USER@|$service_user|g" \
        -e "s|@SWARM_WORKSPACE@|$workspace|g" \
        "$source_dir/${unit}.in" > "$temporary"
    install -o root -g root -m 0644 "$temporary" "$unit_dir/$unit"
    rm -f "$temporary"
    trap - EXIT
done

install -o root -g root -m 0644 \
    "$source_dir/swarm-drone.target" "$unit_dir/swarm-drone.target"

if [[ ! -e "$config_path" ]]; then
    temporary="$(mktemp)"
    trap 'rm -f "${temporary:-}"' EXIT
    sed "s|@SWARM_WORKSPACE@|$workspace|g" \
        "$source_dir/swarm-drone.conf.example" > "$temporary"
    install -o root -g root -m 0644 "$temporary" "$config_path"
    rm -f "$temporary"
    trap - EXIT
    echo "Created $config_path; edit it before starting anything."
else
    echo "Preserved existing $config_path."
fi

systemctl daemon-reload

cat <<EOF

Installed swarm-drone systemd files for:
  user:      $service_user
  workspace: $workspace

Nothing was enabled or started.

Next:
  1. sudoedit $config_path
  2. sudo systemctl start swarm-drone.target
  3. inspect all four services and test with propellers removed
  4. sudo systemctl enable swarm-drone.target

See: $source_dir/README.md
EOF
