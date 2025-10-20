#!/usr/bin/env bash
set -euo pipefail
# Helper to source files without nounset errors from external scripts
source_relaxed() {
  # Temporarily disable nounset to avoid unbound variable errors from sourced scripts
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}


# Configurations
# Arg1: node name to wait for (default: /yuntai_controller_node)
NODE_NAME="${1:-/yuntai_controller_node}"

# Environment overrides (can be set in systemd unit or shell)
# TIMEOUT_SEC=0 means wait indefinitely
TIMEOUT_SEC="${TIMEOUT_SEC:-300}"
INTERVAL_SEC="${INTERVAL_SEC:-2}"

# Resolve workspace dir based on this script location
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Source ROS 2 environment
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  # Prefer workspace overlay if available
  echo "[INFO] Sourcing workspace: ${WS_DIR}/install/setup.bash"
  source_relaxed "${WS_DIR}/install/setup.bash"
else
  # Fallback to common ROS 2 distros
  if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    # shellcheck disable=SC1091
    echo "[INFO] Sourcing ROS 2: /opt/ros/jazzy/setup.bash"
    source_relaxed "/opt/ros/jazzy/setup.bash"
  elif [ -f "/opt/ros/humble/setup.bash" ]; then
    # shellcheck disable=SC1091
    echo "[INFO] Sourcing ROS 2: /opt/ros/humble/setup.bash"
    source_relaxed "/opt/ros/humble/setup.bash"
  elif [ -f "/opt/ros/iron/setup.bash" ]; then
    # shellcheck disable=SC1091
    echo "[INFO] Sourcing ROS 2: /opt/ros/iron/setup.bash"
    source_relaxed "/opt/ros/iron/setup.bash"
  elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    # shellcheck disable=SC1091
    echo "[INFO] Sourcing ROS 2: /opt/ros/foxy/setup.bash"
    source_relaxed "/opt/ros/foxy/setup.bash"
  elif [ -f "/opt/ros/rolling/setup.bash" ]; then
    # shellcheck disable=SC1091
    echo "[INFO] Sourcing ROS 2: /opt/ros/rolling/setup.bash"
    source_relaxed "/opt/ros/rolling/setup.bash"
  else
    echo "[ERROR] Failed to locate ROS 2 environment (install/setup.bash or /opt/ros/*)." >&2
    exit 1
  fi
fi

echo "[INFO] Waiting for node ${NODE_NAME} to appear..."
start_ts=$(date +%s)
while true; do
  if ros2 node list 2>/dev/null | grep -qx "${NODE_NAME}"; then
    echo "[INFO] Detected ${NODE_NAME}."
    break
  fi

  if [ "${TIMEOUT_SEC}" != "0" ]; then
    now=$(date +%s)
    elapsed=$(( now - start_ts ))
    if [ "${elapsed}" -ge "${TIMEOUT_SEC}" ]; then
      echo "[WARN] Timeout ${TIMEOUT_SEC}s reached; starting temperature_publisher anyway."
      break
    fi
  fi

  sleep "${INTERVAL_SEC}"
done

echo "[INFO] Starting: ros2 run temperature_publisher temperature_publisher"
exec ros2 run temperature_publisher temperature_publisher
