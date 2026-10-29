#!/usr/bin/env bash
# Author: Steven (https://github.com/sng516157-web) — 2026-10-30
# Install ROS 2 Jazzy dependencies if needed, then launch the deterministic
# insertion simulation: Gazebo Harmonic + UR5e + AG95 + MoveIt + task node.
#
#   ./scripts/sim.sh                       # GUI, full task
#   ./scripts/sim.sh --cpu                 # software rendering fallback
#   ./scripts/sim.sh -- rviz:=true         # extra args go to the launch file
#
# The workspace must be built once per machine (see README):
#   cd ~/cable_ws && colcon build --symlink-install --parallel-workers 2
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CPU=false
LAUNCH_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cpu)
      CPU=true
      shift
      ;;
    --)
      shift
      LAUNCH_ARGS+=("$@")
      break
      ;;
    *)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ -f "$ROOT/config/arm.env" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT/config/arm.env"
fi
UR_TYPE="${UR_TYPE:-ur5e}"
export UR_TYPE

codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}")"
if [[ "$codename" != "noble" ]]; then
  echo "ROS 2 Jazzy needs Ubuntu 24.04. This system is ${codename:-unknown}." >&2
  exit 1
fi

PACKAGES=(
  ros-jazzy-desktop
  ros-jazzy-ros-gz
  ros-jazzy-ur-description
  ros-jazzy-xacro
  ros-jazzy-robot-state-publisher
  ros-jazzy-ros2-control
  ros-jazzy-ros2-controllers
  ros-jazzy-gz-ros2-control
  ros-jazzy-controller-manager
  ros-jazzy-joint-state-broadcaster
  ros-jazzy-joint-trajectory-controller
  ros-jazzy-moveit
  ros-jazzy-gz-msgs-vendor
  ros-jazzy-gz-transport-vendor
  ros-jazzy-trac-ik-kinematics-plugin
  python3-colcon-common-extensions
)

install_jazzy() {
  sudo apt-get update
  sudo apt-get install -y software-properties-common curl ca-certificates
  sudo add-apt-repository -y universe
  local ver
  ver="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | sed -n 's/.*"tag_name": *"\([^"]*\)".*/\1/p' | head -1)"
  if [[ -z "$ver" ]]; then
    echo "Could not find the ROS apt source release." >&2
    exit 1
  fi
  curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${codename}_all.deb"
  sudo apt-get install -y /tmp/ros2-apt-source.deb
  sudo apt-get update
  sudo apt-get install -y "${PACKAGES[@]}"
}

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "Installing ROS 2 Jazzy, Gazebo, MoveIt, and the UR arm description."
  install_jazzy
fi

missing=()
for pkg in "${PACKAGES[@]}"; do
  if ! dpkg -s "$pkg" >/dev/null 2>&1; then
    missing+=("$pkg")
  fi
done
if ((${#missing[@]})); then
  sudo apt-get update
  sudo apt-get install -y "${missing[@]}"
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

# Source the built workspace (default ~/cable_ws; override with CABLE_WS).
CABLE_WS="${CABLE_WS:-$HOME/cable_ws}"
if [[ -f "$CABLE_WS/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "$CABLE_WS/install/setup.bash"
  set -u
else
  echo "Workspace not built: $CABLE_WS/install/setup.bash is missing." >&2
  echo "Build it first:  cd $CABLE_WS && colcon build --symlink-install --parallel-workers 2" >&2
  exit 1
fi

export GZ_SIM_RESOURCE_PATH="$ROOT/sim/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export DISPLAY="${DISPLAY:-:0}"
# All nodes live in this one WSL instance; localhost-only DDS avoids flaky
# discovery through the NAT/VPN network path.
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
if [[ -d /mnt/wslg ]]; then
  export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
  export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"
  export PULSE_SERVER="${PULSE_SERVER:-unix:/mnt/wslg/PulseServer}"
fi
if [[ "$CPU" == true ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
fi

# Kill leftovers from previous runs first: gz/ROS topics are shared
# machine-wide, and a stale task node or bridge can weld the plug (or command
# the gripper) in a FRESH world. This bit us hard; never run two at once.
echo "Cleaning up any previous sim processes..."
pkill -f "gz sim" 2>/dev/null || true
pkill -f "move_group|robot_state_publisher|gripper_move|parameter_bridge|pose_bridge|static_transform|spawner|deterministic_insertion|ros2 launch|rviz2" 2>/dev/null || true
sleep 2

echo "Launching cable insertion simulation (arm: ${UR_TYPE})."
echo "Useful variants:"
echo "  ./scripts/sim.sh --cpu                       # software rendering"
echo "  ./scripts/sim.sh -- start_task:=false        # bring up the sim without the task"
echo "  ./scripts/sim.sh -- gui:=false rviz:=false   # headless"
exec ros2 launch cable_insertion insertion.launch.py "${LAUNCH_ARGS[@]}"
