# Cable insertion sim

Gazebo Harmonic scene of a UR5e arm with a DH AG95 gripper that picks up an SC
plug and inserts it into an SC port. Deterministic waypoint baseline built on
ROS 2 Jazzy, `gz_ros2_control`, and MoveIt 2.

> **Current status:** the pipeline works end-to-end through grasp, lift, and
> transport-to-port with rigorous validation. The final aligned insertion is
> blocked by a measured physics limitation (the AG95's loose cage lets the
> plug settle dangling, which cannot align with the horizontal port without
> the fingers hitting the table). See "Limitations and known issues". This is
> exactly what the camera + force/torque extensions are meant to address.

## Supported environment

- Ubuntu 24.04 (WSL 2 with WSLg works; `--cpu` software rendering is the fallback)
- ROS 2 Jazzy (binary packages)
- Gazebo Harmonic via `ros_gz`

## Setup

From Windows PowerShell, the read-only preflights report environment problems:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\scripts\preflight_windows.ps1
```

```bash
# inside Ubuntu, before ROS installation
bash scripts/preflight_wsl.sh --bootstrap
# after ROS installation
bash scripts/preflight_wsl.sh --ros
```

Keep the project in the Linux filesystem — never build under `/mnt/c`:

```bash
mkdir -p ~/cable_ws/src
cp -r /mnt/c/<path-to>/cable-insertion-sim ~/cable_ws/src/
```

Install dependencies (also happens automatically on the first `./scripts/sim.sh` run):

```bash
sudo apt install \
  ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-ur-description ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller \
  ros-jazzy-moveit ros-jazzy-gz-msgs-vendor ros-jazzy-gz-transport-vendor \
  ros-jazzy-trac-ik-kinematics-plugin python3-colcon-common-extensions
```

(`ros-jazzy-gz-msgs-vendor` / `ros-jazzy-gz-transport-vendor` provide the
`gz.msgs10` / `gz.transport13` Python bindings used by the gripper bridge. The
guide's `python3-gz-msgs10` / `python3-gz-transport13` names only exist in the
OSRF Gazebo apt repo; with `ros-jazzy-ros-gz` the bindings come from the ROS
vendor packages.)

Build:

```bash
cd ~/cable_ws
colcon build --symlink-install --parallel-workers 2
```

`colcon list` should show only `cable_insertion` (`sim/dh_ag95` is a legacy
ROS 1 manifest kept for gripper assets and excluded via `COLCON_IGNORE`).

## Run

```bash
./scripts/sim.sh                              # GUI + full deterministic task
./scripts/sim.sh --cpu                        # software rendering (WSLg fallback)
./scripts/sim.sh -- start_task:=false         # simulation only
./scripts/sim.sh -- gui:=false rviz:=false    # headless
./scripts/sim.sh -- start_task:=false rviz:=true
```

`scripts/sim.sh` sources `~/cable_ws/install` (override with `CABLE_WS`) and
ends in `ros2 launch cable_insertion insertion.launch.py`. The launch file
starts Gazebo, spawns the combined UR5e + AG95 robot at `(-0.15, 0, 0.75)`,
starts `robot_state_publisher`, the `gazebo_world` → `world` static transform,
the gz bridges (`/clock`, pose info, port contact), the arm controllers, the
gripper bridge, `move_group`, and the task node.

Choose the arm in `config/arm.env` (`UR_TYPE=ur5e` by default).

For faster tuning iterations against an already-running sim
(`start_task:=false`), `scripts/run_task.sh` resets the plug and re-runs just
the task node:

```bash
./scripts/sim.sh -- gui:=true start_task:=false   # terminal 1
./scripts/run_task.sh                             # terminal 2
```

## Task sequence

```text
START -> OPEN_GRIPPER -> HOME -> PRE_GRASP -> GRASP_APPROACH -> CLOSE_GRIPPER
-> LIFT -> VALIDATE_GRASP -> PRE_INSERT -> PORT_ALIGN -> INSERT
-> VALIDATE_INSERTION -> RELEASE -> RETREAT -> HOME -> SUCCESS
```

- HOME, PRE_GRASP, PRE_INSERT: MoveIt pose planning (OMPL).
- Grasp approach, lift, port alignment, insertion, retreat: Cartesian paths
  (`cartesian_step_size`, minimum path fraction `cartesian_min_fraction: 0.95`).
- VALIDATE_GRASP: the plug's bridged Gazebo pose must rise by at least 2 cm
  AND the plug's pose in the gripper (tip relative to `gripper_tcp`) must match
  the calibrated `tip_in_tcp` constant within `grasp_pose_tolerance`. The plug
  settles lower in the gripper's cage than the idealized grasp, so it rises
  less than the commanded lift — the cage-pose check is the meaningful one.
- VALIDATE_INSERTION: the tip pose (plug model pose ∘ fixed `sc_tip_link`
  offset) must land within `insertion_pose_tolerance` of the expected inserted
  pose, or the `sc_port_sensor` contact must fire.

Success output ends with:

```text
================ TASK RESULT ================
Result:        SUCCESS
Gripper:       open
Arm at home:   yes
```

A failure reports the exact state, the MoveIt/controller error if any, the
measured validation error, and the gripper/arm end state; the node exits
nonzero.

## Frames

| Frame | Meaning |
|---|---|
| `gazebo_world` | Gazebo world frame; all scene poses in config use it |
| `world` | Robot URDF root / MoveIt planning frame; `gazebo_world` → `world` is the static spawn transform `(-0.15, 0, 0.75)` |
| `gripper_tcp` | Fixed planning tip between the finger pads (`tcp_pose` launch arg) |
| plug body | `sc_plug_link`, deterministic pose from config |
| `sc_tip_link` | Plug tip helper frame, fixed offset in the plug model |
| port body | `sc_port_link`, deterministic pose from config |
| `sc_port_base_link_entrance` | Port entrance helper frame, fixed offsets in the port model |

The task node requests every motion as a `PoseStamped` in `gazebo_world` and
transforms to the planning frame via TF2. Gazebo pose ground truth is used only
for validation.

## Configuration

`config/insertion_task.yaml` holds every task number: scene poses, the constant
SDF-derived transforms (`plug_tip_in_plug`, `port_base_in_port`,
`entrance_in_port_base`), `insertion_depth`, grasp offset, approach/lift/
retreat distances, velocity scalings, tolerances, and the ACM allow-list.
`config/controllers.yaml` holds the arm ros2_control controllers. The gripper
keeps its own Gazebo joint controllers and the `/gripper_cmd` topic
(`0.0` open, `0.62` closed — measured on the model: 0.0 spreads the pads
~9 cm, 0.62 wraps the fingers to ~2 cm; the legacy demo script's comments
have these backwards).

### Tuning

`grasp_in_plug`, `tcp_pose`, and `insertion_depth` are starting values. After
the first run, check the gripper mount orientation and pad placement in the
Gazebo GUI (`ros2 run tf2_ros tf2_echo world gripper_tcp` helps) and adjust the
YAML. If the plug slips during LIFT, the port/plug friction may need `<ode>`
mu values — DART ignores the `<bullet>` friction blocks in the port model.

**Calibrating `tip_in_tcp`:** the plug settles into the gripper's cage lower
than the idealized grasp. The task node logs the measured transform at
VALIDATE_GRASP (ground truth, evaluation only):

```text
MEASURED tip_in_tcp: xyz=[...] rpy=[...]
```

Run a few times, average the values, and put them in `tip_in_tcp` in
`config/insertion_task.yaml`. The insertion alignment uses that calibrated
constant, so the port targets stay correct despite the cage settling.

## Simulation simplifications and scene notes

- `attach_gripper.py` disables gravity on every link and adds joint damping to
  the arm so the unactuated-by-default setup stays stable; the arm is driven by
  `gz_ros2_control` trajectories.
- The planning scene contains a simplified port collision box (a thin base
  plate) and the plug stand; the attached plug is ACM-allowed against them.
- The plug rests on a narrow stand (`plug_stand`, 15×25×30 mm at
  `(0.22, -0.12)`) raising it 3 cm off the table. The AG95's finger links are
  10–16 cm long, so a 7 mm-tall plug lying flat on the table cannot be wrapped
  — the fingertips would have to pass below the table surface. The stand gives
  the fingers room to close around the housing. (Verified from the sim: the
  closed cage holds the plug reliably once it is inside.)
- The gripper command mapping on the real model is **`0.0` = open, `0.62` =
  closed** (the legacy `gripper_move.py` comment block says the opposite).
- Fixed initial plug/port poses; no vision, no force control.

## Limitations and known issues

- Deterministic poses only; no vision, no force feedback; simplified physics
  (gravity disabled on the robot).
- **Insertion is the current blocker.** The grasp+lift reliably capture the
  plug, but the AG95's loose wrap lets the 40 g plug settle dangling
  *vertically* in the cage (measured `tip_in_tcp` repeats to ~0.2 mm but the
  orientation is vertical and varies a few degrees run to run). The SC port's
  entrance faces −y (horizontal), so a dangling plug cannot be aligned with it
  without the long fingers colliding with the table. The task fails cleanly at
  PORT_ALIGN/INSERT with a full report. Making this work needs a firmer,
  orientation-stable hold — that is exactly what the extension roadmap's
  camera (measure the real grasp pose) and force/torque sensing (guided
  insertion) are for.

## Extension roadmap

1. Wrist RGB-D camera → replace `scene.plug_pose` / `scene.port_pose` (keep the
   deterministic poses as fallback).
2. Gripper-pad contact sensors → contact-based grasp validation.
3. Wrist force/torque sensor → force limits and guided final insertion.
4. BehaviorTree.CPP or MoveIt Task Constructor for recovery — only if the
   baseline demonstrates the need.

## Layout

```
config/arm.env              which UR arm to spawn
config/controllers.yaml     arm ros2_control controllers
config/insertion_task.yaml  task poses, distances, speeds, tolerances
launch/insertion.launch.py  complete startup (sim + MoveIt + task)
moveit_config/              SRDF, kinematics, joint limits, controllers
src/deterministic_insertion.cpp  task state machine
scripts/sim.sh              install deps, launch everything (--cpu fallback)
scripts/preflight_*.ps1/sh  read-only environment checks
sim/insertion.sdf           table, plug, port, contact system
sim/models/                 SC plug and SC port meshes
sim/dh_ag95/                DH AG95 meshes, wrist mount, gripper bridge
```

## License

Scripts in this repository are MIT ([LICENSE](LICENSE)). The SC meshes are Apache-2.0 ([sim/models/NOTICE](sim/models/NOTICE)). The DH gripper model is BSD ([sim/dh_ag95/NOTICE](sim/dh_ag95/NOTICE)). The Universal Robots description installed by the script keeps its own license.
