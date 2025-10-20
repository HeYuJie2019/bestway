#!/usr/bin/env bash
set -e -o pipefail

# Configurable via environment
ROS_DISTRO="${ROS_DISTRO:-humble}"
# Seconds to wait before start (e.g., for network/DDS)
START_DELAY="${START_DELAY:-2}"

# Resolve workspace root (this script lives in <ws>/scripts/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Source ROS 2 and workspace env
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "[start_livox] ROS 2 distro '${ROS_DISTRO}' not found under /opt/ros" >&2
  exit 1
fi

if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  source "${WS_ROOT}/install/setup.bash"
else
  echo "[start_livox] Warning: ${WS_ROOT}/install/setup.bash not found; proceeding with system ROS only" >&2
fi

sleep "$START_DELAY"

echo "[start_livox] Starting: ros2 launch livox_ros_driver2 msg_MID360_launch.py"
exec ros2 launch livox_ros_driver2 msg_MID360_launch.py
