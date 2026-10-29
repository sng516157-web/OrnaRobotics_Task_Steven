#!/usr/bin/env bash
# Debug helper: run the deterministic task node against an ALREADY RUNNING
# simulation started with:
#   ros2 launch cable_insertion insertion.launch.py start_task:=false
#
# Regenerates the robot description from the current sources, resets the plug
# to its configured pose, then runs the task node. Much faster than a full
# relaunch when tuning config/insertion_task.yaml.
#
#   ./scripts/run_task.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE=/tmp/cable_insertion_debug
mkdir -p "$CACHE"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

UR_TYPE="${UR_TYPE:-ur5e}"
TCP_POSE="${TCP_POSE:-0 0 0.12 0 0 0}"

desc="$(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro"
xacro "$desc" "ur_type:=${UR_TYPE}" name:=ur > "$CACHE/robot.urdf"
python3 "$ROOT/sim/dh_ag95/attach_gripper.py" \
  "$CACHE/robot.urdf" "$ROOT/sim/dh_ag95/dh_robotics_ag95.urdf" \
  "$ROOT/sim/dh_ag95/meshes" --ros2-control "$ROOT/config/controllers.yaml" \
  --tcp "$TCP_POSE"
cp "$ROOT/moveit_config/cable_insertion.srdf" "$CACHE/robot.srdf"

python3 - "$ROOT" "$CACHE" <<'PYEOF'
import pathlib
import sys
import yaml

root, cache = map(pathlib.Path, sys.argv[1:3])
params = yaml.safe_load(open(root / "config/insertion_task.yaml"))[
    "deterministic_insertion"]["ros__parameters"]
params["robot_description"] = (cache / "robot.urdf").read_text()
params["robot_description_semantic"] = (cache / "robot.srdf").read_text()
params["robot_description_planning"] = yaml.safe_load(
    open(root / "moveit_config/joint_limits.yaml"))
params["use_sim_time"] = True
yaml.safe_dump(
    {"deterministic_insertion": {"ros__parameters": params}},
    open(cache / "task_params.yaml", "w"))
PYEOF

# Reset the plug to its configured pose (best effort; requires the sim up).
plug_pose=$(python3 -c "
import yaml
p = yaml.safe_load(open('$ROOT/config/insertion_task.yaml'))['deterministic_insertion']['ros__parameters']['scene']['plug_pose']
print(f'name: \"plug\", position: {{x: {p[0]}, y: {p[1]}, z: {p[2]}}}')")
gz service -s /world/insertion/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 3000 --req "$plug_pose" >/dev/null 2>&1 || echo "warning: could not reset plug pose (is the sim running?)"
sleep 1

exec ros2 run cable_insertion deterministic_insertion \
  --ros-args --params-file "$CACHE/task_params.yaml"
