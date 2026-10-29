#!/usr/bin/env bash
# Read-only Ubuntu/WSL preflight for the cable-insertion simulation.
#
#   bash scripts/preflight_wsl.sh --bootstrap
#   bash scripts/preflight_wsl.sh --ros
#   bash scripts/preflight_wsl.sh --ros --project-dir ~/cable_ws/src/cable-insertion-sim
#
# --bootstrap checks the base WSL environment before ROS installation.
# --ros repeats those checks and also validates ROS 2 Jazzy, Gazebo,
# MoveIt, controller packages, and Gazebo Python bindings.
#
# This script does not install packages, use sudo, edit files, or start ROS.

set -uo pipefail

MODE="bootstrap"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
FAILURES=0
WARNINGS=0

usage() {
  cat <<'EOF'
Usage: bash scripts/preflight_wsl.sh [--bootstrap|--ros] [--project-dir PATH]

  --bootstrap         Check Ubuntu, WSL, WSLg, networking, and resources (default).
  --ros               Run bootstrap checks plus ROS/Gazebo/MoveIt checks.
  --project-dir PATH  Project directory to validate (defaults to this repository).
  -h, --help          Show this help.

No changes are made to the system. Exit code 1 means at least one check failed.
Warnings do not change the exit code.
EOF
}

while (($#)); do
  case "$1" in
    --bootstrap)
      MODE="bootstrap"
      shift
      ;;
    --ros)
      MODE="ros"
      shift
      ;;
    --project-dir)
      if [[ $# -lt 2 ]]; then
        echo "Missing path after --project-dir" >&2
        exit 2
      fi
      PROJECT_DIR="$2"
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

pass() { printf '[PASS] %s\n' "$1"; }
info() { printf '[INFO] %s\n' "$1"; }
warn() {
  WARNINGS=$((WARNINGS + 1))
  printf '[WARN] %s\n' "$1"
}
fail() {
  FAILURES=$((FAILURES + 1))
  printf '[FAIL] %s\n' "$1"
}

check_command() {
  local command_name="$1"
  local install_hint="$2"
  if command -v "$command_name" >/dev/null 2>&1; then
    pass "Command '$command_name' is available."
  else
    fail "Command '$command_name' is missing. $install_hint"
  fi
}

check_ros_package() {
  local package_name="$1"
  if ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
    pass "ROS package '$package_name' is available."
  else
    fail "ROS package '$package_name' is missing or not visible in this environment."
  fi
}

printf 'Cable insertion: read-only WSL preflight (%s mode)\n' "$MODE"
printf 'Project directory: %s\n\n' "$PROJECT_DIR"

# Base operating-system checks.
if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  if [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "24.04" ]]; then
    pass "Operating system is Ubuntu 24.04."
  else
    fail "Expected Ubuntu 24.04; found ${PRETTY_NAME:-unknown operating system}."
  fi
else
  fail "/etc/os-release is unavailable; cannot identify the operating system."
fi

kernel_release="$(uname -r 2>/dev/null || true)"
kernel_lower="${kernel_release,,}"
if [[ "$kernel_lower" == *microsoft* || "$kernel_lower" == *wsl* || -n "${WSL_INTEROP:-}" ]]; then
  pass "The environment appears to be WSL."
else
  warn "WSL was not detected. This script is intended for the project's WSL environment."
fi

if command -v apt-get >/dev/null 2>&1; then
  pass "APT is available."
else
  fail "APT is unavailable; Ubuntu package installation cannot proceed."
fi

if command -v sudo >/dev/null 2>&1; then
  if sudo -n true >/dev/null 2>&1; then
    pass "sudo is available and currently authorized."
  else
    info "sudo is available but will prompt for the user's password when needed; this is normal."
  fi
else
  fail "sudo is unavailable. Complete the Ubuntu account and sudo setup before installing ROS."
fi

# WSLg/display checks. A user-approved alternative X server is allowed with a warning.
if [[ -d /mnt/wslg ]]; then
  pass "WSLg directory is present."
elif [[ -n "${DISPLAY:-}" ]]; then
  warn "WSLg is absent, but DISPLAY is set. Validate the alternative display server before launching Gazebo."
else
  fail "WSLg is absent and DISPLAY is unset. Repair/update WSLg or approve an alternative display setup."
fi

if [[ -S /tmp/.X11-unix/X0 ]]; then
  pass "X11 display socket X0 is present."
else
  warn "X11 socket X0 was not found. The existing launcher defaults DISPLAY to :0, so GUI rendering may fail."
fi

if [[ -n "${DISPLAY:-}" ]]; then
  info "DISPLAY=$DISPLAY"
else
  info "DISPLAY is unset; the supplied sim.sh will default it to :0."
fi

# Basic network and resource checks. These do not modify APT state.
if command -v getent >/dev/null 2>&1 && getent hosts archive.ubuntu.com >/dev/null 2>&1; then
  pass "Ubuntu archive DNS resolution works."
else
  warn "Could not resolve archive.ubuntu.com. Check WSL networking before package installation."
fi

if [[ -r /proc/meminfo ]]; then
  memory_kb="$(awk '/^MemTotal:/ {print $2; exit}' /proc/meminfo)"
  if [[ "$memory_kb" =~ ^[0-9]+$ ]]; then
    memory_gib=$((memory_kb / 1024 / 1024))
    info "Approximate WSL memory: ${memory_gib} GiB."
    if ((memory_kb < 8 * 1024 * 1024)); then
      warn "Less than 8 GiB of memory is visible. Use binary packages, optional RViz, and limited build parallelism."
    fi
  fi
fi

disk_kb="$(df -Pk "$HOME" 2>/dev/null | awk 'NR==2 {print $4}')"
if [[ "$disk_kb" =~ ^[0-9]+$ ]]; then
  disk_gib=$((disk_kb / 1024 / 1024))
  info "Approximate free disk space on the home filesystem: ${disk_gib} GiB."
  if ((disk_kb < 15 * 1024 * 1024)); then
    warn "Less than 15 GiB is free. ROS, Gazebo, MoveIt, and build artifacts may require more space."
  fi
fi

# Repository-specific checks.
if [[ -d "$PROJECT_DIR" ]]; then
  pass "Project directory exists."
else
  fail "Project directory does not exist: $PROJECT_DIR"
fi

case "$PROJECT_DIR" in
  /mnt/[a-z]/*|/mnt/[a-z])
    warn "The project is on a Windows-mounted filesystem. Copy it into the WSL Linux filesystem before building."
    ;;
  *)
    pass "Project is not on a /mnt/<drive> Windows-mounted path."
    ;;
esac

if [[ -e "$PROJECT_DIR/sim/dh_ag95/COLCON_IGNORE" ]]; then
  pass "Legacy catkin gripper package is excluded from colcon."
else
  warn "sim/dh_ag95/COLCON_IGNORE is missing. Add it before building to exclude the nested ROS 1 package."
fi

if [[ "$MODE" == "ros" ]]; then
  printf '\n--- ROS 2 / Gazebo checks ---\n'

  check_command colcon "Install python3-colcon-common-extensions."

  if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    fail "ROS 2 Jazzy is not installed at /opt/ros/jazzy."
  else
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    source_status=$?
    set -u

    if ((source_status != 0)); then
      fail "Sourcing /opt/ros/jazzy/setup.bash failed."
    else
      if [[ "${ROS_DISTRO:-}" == "jazzy" && "${ROS_VERSION:-}" == "2" ]]; then
        pass "ROS 2 Jazzy environment sourced successfully."
      else
        fail "Unexpected ROS environment: ROS_DISTRO=${ROS_DISTRO:-unset}, ROS_VERSION=${ROS_VERSION:-unset}."
      fi

      check_command ros2 "Reinstall or repair the ROS 2 Jazzy installation."
      check_command xacro "Install ros-jazzy-xacro."

      if [[ -x /usr/bin/python3 ]]; then
        pass "Ubuntu system Python is present."
        if /usr/bin/python3 -c 'import gz.msgs10.double_pb2; import gz.transport13' >/dev/null 2>&1; then
          pass "Gazebo Python bindings gz.msgs10 and gz.transport13 import successfully."
        else
          fail "Gazebo Python imports failed. Install python3-gz-msgs10 and python3-gz-transport13 and check Python environment conflicts."
        fi
      else
        fail "/usr/bin/python3 is missing or not executable."
      fi

      if command -v ros2 >/dev/null 2>&1; then
        for package_name in \
          ur_description \
          ros_gz_sim \
          ros_gz_bridge \
          gz_ros2_control \
          controller_manager \
          joint_state_broadcaster \
          joint_trajectory_controller \
          robot_state_publisher \
          moveit \
          moveit_ros_move_group; do
          check_ros_package "$package_name"
        done
      fi

      if command -v gz >/dev/null 2>&1; then
        gazebo_versions="$(gz sim --versions 2>&1 || true)"
        if [[ "$gazebo_versions" =~ (^|[^0-9])8\.[0-9]+ ]]; then
          pass "Gazebo Sim 8.x (Harmonic series) is available."
        else
          warn "Could not confirm Gazebo Harmonic from 'gz sim --versions': $gazebo_versions"
        fi
      else
        fail "The 'gz' command is missing. Verify the ROS Jazzy Gazebo Harmonic installation."
      fi
    fi
  fi
fi

printf '\n================ WSL preflight summary ================\n'
if ((FAILURES > 0)); then
  printf 'FAILED: %d failure(s), %d warning(s).\n' "$FAILURES" "$WARNINGS"
  printf 'Resolve the failures and rerun this script. See the environment section of AGENT_DETERMINISTIC_WAYPOINT_GUIDE.md.\n'
  exit 1
fi

if ((WARNINGS > 0)); then
  printf 'PASSED WITH WARNINGS: %d warning(s).\n' "$WARNINGS"
else
  printf 'PASSED: all requested checks completed successfully.\n'
fi

if [[ "$MODE" == "bootstrap" ]]; then
  printf 'Next: install the documented ROS dependencies, then run bash scripts/preflight_wsl.sh --ros.\n'
else
  printf 'Next: validate package discovery with colcon list, then build and run the control milestones.\n'
fi
exit 0
