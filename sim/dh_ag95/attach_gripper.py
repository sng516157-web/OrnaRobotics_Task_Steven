#!/usr/bin/env python3
"""Mount the DH AG95 onto a UR arm URDF and make the fingers position-controlled.

Usage: attach_gripper.py ARM.urdf GRIPPER.urdf MESH_DIR [--ros2-control YAML]
       [--tcp "x y z roll pitch yaw"]

Without the optional flags this produces the original description: AG95 fingers
with per-joint Gazebo position controllers, damped unactuated arm joints, and
gravity disabled on every link (a documented simulation simplification).

--ros2-control adds a gz_ros2_control system plugin for the six UR arm joints,
using the given controller YAML, so the arm gets a standard ROS 2 trajectory
interface. The gripper joints stay on their direct Gazebo controllers.

--tcp adds a fixed gripper_tcp link between the finger pads so MoveIt has a
stable end-effector frame for the combined model.
"""

import argparse
import xml.etree.ElementTree as ET

SKIP_LINKS = {"world", "gripper_root_link", "ee_link"}
SKIP_JOINTS = {"world_fixed", "ee_fixed_joint", "gripper_base_joint"}

ACTUATED = (
    "gripper_finger1_joint",
    "gripper_finger2_joint",
    "gripper_finger1_inner_knuckle_joint",
    "gripper_finger2_inner_knuckle_joint",
    "gripper_finger1_finger_tip_joint",
    "gripper_finger2_finger_tip_joint",
    "gripper_finger1_finger_joint",
    "gripper_finger2_finger_joint",
)

ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

# Links that touch a grasped object. The vendor URDF ships per-link Gazebo
# friction blocks (mu 100000) which this merge drops; restore a strong but
# solver-stable version so small objects can actually be pinched.
CONTACT_LINKS = (
    "gripper_finger1_finger_link",
    "gripper_finger2_finger_link",
    "gripper_finger1_finger_tip_link",
    "gripper_finger2_finger_tip_link",
)
CONTACT_MU = "10000"
CONTACT_KP = "1000000"

# Gazebo scales URDF colors by 1.25, so these stay grey instead of clamping to white.
COLORS = {
    "base_link.STL": "0.44 0.45 0.47 1",
    "crank_Link.STL": "0.28 0.29 0.31 1",
    "proximal_phalanx_Link.STL": "0.50 0.51 0.53 1",
    "rod_Link.STL": "0.18 0.19 0.20 1",
    "distal_phalanx_Link.STL": "0.56 0.57 0.59 1",
}


def mesh_file(filename: str) -> str:
    return filename.replace("\\", "/").rsplit("/", 1)[-1]


def paint(visual: ET.Element, rgba: str, name: str) -> None:
    material = visual.find("material")
    if material is None:
        material = ET.SubElement(visual, "material", {"name": name})
    else:
        material.set("name", name)
    color = material.find("color")
    if color is None:
        color = ET.SubElement(material, "color")
    color.set("rgba", rgba)


def stabilize(link: ET.Element) -> None:
    inertial = link.find("inertial")
    if inertial is None:
        return
    mass = inertial.find("mass")
    if mass is not None and float(mass.get("value", "0")) < 0.05:
        mass.set("value", "0.05")
    inertia = inertial.find("inertia")
    if inertia is None:
        return
    for key in ("ixx", "iyy", "izz"):
        value = abs(float(inertia.get(key, "0")))
        inertia.set(key, f"{max(value, 0.002):.6g}")
    for key in ("ixy", "ixz", "iyz", "iyx", "izx", "izy"):
        if inertia.get(key) is not None:
            inertia.set(key, "0")


def hold_arm(joint: ET.Element) -> None:
    if joint.get("type") == "fixed" or joint.get("name") in ACTUATED:
        return
    axis = joint.find("axis")
    if axis is None:
        return
    dynamics = axis.find("dynamics")
    if dynamics is None:
        dynamics = ET.SubElement(axis, "dynamics")
    dynamics.set("damping", "8")
    dynamics.set("friction", "2")


def add_position_controller(root: ET.Element, joint_name: str) -> None:
    gazebo = ET.SubElement(root, "gazebo")
    plugin = ET.SubElement(
        gazebo,
        "plugin",
        {
            "filename": "gz-sim-joint-position-controller-system",
            "name": "gz::sim::systems::JointPositionController",
        },
    )
    ET.SubElement(plugin, "joint_name").text = joint_name
    ET.SubElement(plugin, "topic").text = f"/gripper/{joint_name}"
    ET.SubElement(plugin, "p_gain").text = "5"
    ET.SubElement(plugin, "i_gain").text = "0.2"
    ET.SubElement(plugin, "d_gain").text = "1.0"
    ET.SubElement(plugin, "cmd_max").text = "2"
    ET.SubElement(plugin, "cmd_min").text = "-2"


def ensure_world_link(root: ET.Element) -> None:
    """Guarantee a 'world' root link so the robot can be placed in TF relative
    to the documented gazebo_world frame."""
    if root.find("link[@name='world']") is not None:
        return
    if root.find("link[@name='base_link']") is None:
        raise SystemExit("The arm URDF has neither a world nor a base_link.")
    ET.SubElement(root, "link", {"name": "world"})
    joint = ET.SubElement(root, "joint", {"name": "world_joint", "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(joint, "parent", {"link": "world"})
    ET.SubElement(joint, "child", {"link": "base_link"})


def add_tcp(root: ET.Element, pose: str) -> None:
    """Add a fixed gripper_tcp link at 'x y z roll pitch yaw' relative to
    gripper_base_link, roughly midway between the finger pads."""
    values = pose.split()
    if len(values) != 6:
        raise SystemExit("--tcp expects 'x y z roll pitch yaw'.")
    ET.SubElement(root, "link", {"name": "gripper_tcp"})
    joint = ET.SubElement(root, "joint", {"name": "gripper_tcp_fixed", "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": " ".join(values[:3]), "rpy": " ".join(values[3:])})
    ET.SubElement(joint, "parent", {"link": "gripper_base_link"})
    ET.SubElement(joint, "child", {"link": "gripper_tcp"})


def add_ros2_control(root: ET.Element, controllers_yaml: str) -> None:
    present = {joint.get("name") for joint in root.findall("joint")}
    missing = [name for name in ARM_JOINTS if name not in present]
    if missing:
        raise SystemExit(f"The arm URDF is missing joints: {', '.join(missing)}")
    system = ET.SubElement(
        root, "ros2_control", {"name": "gz_ros2_control", "type": "system"}
    )
    hardware = ET.SubElement(system, "hardware")
    ET.SubElement(hardware, "plugin").text = "gz_ros2_control/GazeboSimSystem"
    for name in ARM_JOINTS:
        joint = ET.SubElement(system, "joint", {"name": name})
        ET.SubElement(joint, "command_interface", {"name": "position"})
        state = ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(state, "param", {"name": "initial_value"}).text = "0.0"
        ET.SubElement(joint, "state_interface", {"name": "velocity"})
    gazebo = ET.SubElement(root, "gazebo")
    plugin = ET.SubElement(
        gazebo,
        "plugin",
        {
            "filename": "gz_ros2_control-system",
            "name": "gz_ros2_control::GazeboSimROS2ControlPlugin",
        },
    )
    ET.SubElement(plugin, "parameters").text = controllers_yaml


def attach(
    arm_path: str,
    gripper_path: str,
    mesh_dir: str,
    ros2_control: str | None = None,
    tcp: str | None = None,
) -> None:
    arm = ET.parse(arm_path)
    grip = ET.parse(gripper_path)
    root = arm.getroot()
    grip_root = grip.getroot()

    parent = next(
        (
            name
            for name in ("tool0", "flange", "wrist_3_link")
            if root.find(f"link[@name='{name}']") is not None
        ),
        None,
    )
    if parent is None:
        raise SystemExit("The arm URDF has no tool0 frame to mount the gripper on.")

    ensure_world_link(root)

    for joint in root.findall("joint"):
        hold_arm(joint)

    for link in grip_root.findall("link"):
        if link.get("name") in SKIP_LINKS:
            continue
        stabilize(link)
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            filename = mesh_file(mesh.get("filename", ""))
            mesh.set("filename", f"file://{mesh_dir}/{filename}")
            mesh.set("scale", "0.001 0.001 0.001")
            paint(visual, COLORS.get(filename, "0.7 0.7 0.72 1"), f"dh_{filename}")
        for mesh in link.findall("collision/geometry/mesh"):
            filename = mesh_file(mesh.get("filename", ""))
            mesh.set("filename", f"file://{mesh_dir}/{filename}")
            mesh.set("scale", "0.001 0.001 0.001")
        root.append(link)

    present = set()
    for joint in grip_root.findall("joint"):
        if joint.get("name") in SKIP_JOINTS:
            continue
        mimic = joint.find("mimic")
        if mimic is not None:
            joint.remove(mimic)
        if joint.get("type") == "continuous":
            joint.set("type", "revolute")
        present.add(joint.get("name"))
        root.append(joint)

    # tool0 is X left, Y up, Z out of the flange. The AG95 mesh points along +Z,
    # so this sends the fingers straight out of the wrist.
    mount = ET.SubElement(root, "joint", {"name": "dh_gripper_mount", "type": "fixed"})
    ET.SubElement(mount, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(mount, "parent", {"link": parent})
    ET.SubElement(mount, "child", {"link": "gripper_base_link"})

    if tcp is not None:
        add_tcp(root, tcp)

    for name in ACTUATED:
        if name not in present:
            raise SystemExit(f"DH gripper URDF is missing {name}.")
        add_position_controller(root, name)

    for name in CONTACT_LINKS:
        if root.find(f"link[@name='{name}']") is None:
            raise SystemExit(f"DH gripper URDF is missing link {name}.")
        tag = ET.SubElement(root, "gazebo", {"reference": name})
        ET.SubElement(tag, "mu1").text = CONTACT_MU
        ET.SubElement(tag, "mu2").text = CONTACT_MU
        ET.SubElement(tag, "kp").text = CONTACT_KP

    if ros2_control is not None:
        add_ros2_control(root, ros2_control)

    for link in root.findall("link"):
        tag = ET.SubElement(root, "gazebo", {"reference": link.get("name")})
        ET.SubElement(tag, "gravity").text = "false"

    arm.write(arm_path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("arm", help="UR arm URDF, modified in place")
    parser.add_argument("gripper", help="DH AG95 URDF (read-only)")
    parser.add_argument("mesh_dir", help="Directory with the AG95 STL meshes")
    parser.add_argument(
        "--ros2-control",
        metavar="YAML",
        help="Controller YAML for a gz_ros2_control block on the UR arm joints",
    )
    parser.add_argument(
        "--tcp",
        metavar='"x y z roll pitch yaw"',
        help="Add a fixed gripper_tcp link at this pose relative to gripper_base_link",
    )
    args = parser.parse_args()
    attach(args.arm, args.gripper, args.mesh_dir, args.ros2_control, args.tcp)


if __name__ == "__main__":
    main()
