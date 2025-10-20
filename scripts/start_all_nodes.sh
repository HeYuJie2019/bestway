#!/usr/bin/env bash
set -e -o pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
START_DELAY="${START_DELAY:-5}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SDK_LIB_PATH="/home/bestway/SV_NET_SDK_ARM_250424/demo/consoleDemo/lib/x64"

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "[start_all_nodes] ROS 2 distro '${ROS_DISTRO}' not found under /opt/ros" >&2
  exit 1
fi

if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  source "${WS_ROOT}/install/setup.bash"
else
  echo "[start_all_nodes] Warning: ${WS_ROOT}/install/setup.bash not found; proceeding with system ROS only" >&2
fi

# 补充运行所需的动态库路径（供 temperature_publisher 等依赖 SDK 的程序使用）
if [[ -d "$SDK_LIB_PATH" ]]; then
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$SDK_LIB_PATH"
  echo "[start_all_nodes] LD_LIBRARY_PATH+=${SDK_LIB_PATH}"
else
  echo "[start_all_nodes] Warning: SDK lib path not found: ${SDK_LIB_PATH}" >&2
fi

sleep "$START_DELAY"

echo "[start_all_nodes] Starting: ros2 launch launch/all_nodes.launch.py"
exec ros2 launch "${WS_ROOT}/launch/all_nodes.launch.py"
