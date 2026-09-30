"""Build a Gazebo ros2_control description from the hardware URDF."""

import copy
import os
import xml.etree.ElementTree as ET


MOVING_JOINTS = ("base_joint", "shoulder_joint", "elbow_joint")
GEAR_RATIOS = {
    "base_joint": 8.0,
    "shoulder_joint": 64.0,
    "elbow_joint": 8.0,
}
VISCOUS_PER_N2 = 2.0e-5
DRY_FRICTION_PER_N = 0.004

# base_link.STL reaches z=-0.2981336 m after the fixed Rx(0.073 rad)
# transform from base_link to base_footprint.  Keep a small visual clearance
# above Gazebo's z=0 ground plane instead of letting the chair sink into it.
GAZEBO_BASE_HEIGHT_M = 0.300
PAYLOAD_LINK = "tool_payload_link"
PAYLOAD_JOINT = "tool_payload_joint"
PAYLOAD_RADIUS_M = 0.025


def _add_point_payload(root, mass_kg, offset_xyz):
    """Attach a small spherical payload to the tool point in the Gazebo URDF."""
    mass_kg = float(mass_kg)
    if mass_kg < 0.0:
        raise ValueError("payload_mass_kg phải >= 0")
    if mass_kg == 0.0:
        return
    offset = tuple(float(value) for value in offset_xyz)
    if len(offset) != 3:
        raise ValueError("payload_offset_xyz phải có đúng 3 phần tử")

    link = ET.SubElement(root, "link", {"name": PAYLOAD_LINK})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": f"{mass_kg:.9g}"})
    # A point mass is singular for a rigid-body solver.  Use the inertia of a
    # 25 mm sphere while keeping the centre of mass exactly at the tool point.
    inertia_value = 0.4 * mass_kg * PAYLOAD_RADIUS_M ** 2
    ET.SubElement(inertial, "inertia", {
        "ixx": f"{inertia_value:.9g}",
        "ixy": "0", "ixz": "0",
        "iyy": f"{inertia_value:.9g}",
        "iyz": "0",
        "izz": f"{inertia_value:.9g}",
    })
    visual = ET.SubElement(link, "visual")
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(
        geometry, "sphere", {"radius": f"{PAYLOAD_RADIUS_M:.9g}"})
    material = ET.SubElement(visual, "material", {"name": "payload_red"})
    ET.SubElement(material, "color", {"rgba": "0.85 0.12 0.12 1"})

    joint = ET.SubElement(
        root, "joint", {"name": PAYLOAD_JOINT, "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": "lower_arm_link"})
    ET.SubElement(joint, "child", {"link": PAYLOAD_LINK})
    ET.SubElement(joint, "origin", {
        "xyz": " ".join(f"{value:.9g}" for value in offset),
        "rpy": "0 0 0",
    })


def build_gazebo_description(
        urdf_path, controllers_path, payload_mass_kg=0.0,
        payload_offset_xyz=(0.0, 0.0, 0.0)):
    """Replace only the hardware backend while preserving robot dynamics."""
    root = ET.parse(urdf_path).getroot()

    world_fixed = root.find("joint[@name='world_fixed']")
    if world_fixed is None:
        raise RuntimeError(f"Không thấy joint world_fixed trong {urdf_path}")
    world_origin = world_fixed.find("origin")
    if world_origin is None:
        world_origin = ET.SubElement(world_fixed, "origin")
    world_xyz = [float(value) for value in
                 world_origin.get("xyz", "0 0 0").split()]
    if len(world_xyz) != 3:
        raise RuntimeError("origin xyz của world_fixed phải có 3 phần tử")
    world_xyz[2] += GAZEBO_BASE_HEIGHT_M
    world_origin.set("xyz", " ".join(f"{value:.9g}" for value in world_xyz))

    source_control = root.find("ros2_control")
    if source_control is None:
        raise RuntimeError(f"Không thấy ros2_control trong {urdf_path}")

    gazebo_control = ET.Element(
        "ros2_control", {"name": "GazeboSystem", "type": "system"})
    hardware = ET.SubElement(gazebo_control, "hardware")
    ET.SubElement(hardware, "plugin").text = (
        "gazebo_ros2_control/GazeboSystem")

    for name in MOVING_JOINTS:
        source_joint = source_control.find(f"joint[@name='{name}']")
        if source_joint is None:
            raise RuntimeError(f"ros2_control thiếu khớp {name}")
        sim_joint = ET.SubElement(gazebo_control, "joint", {"name": name})
        for interface in source_joint.findall("command_interface"):
            sim_joint.append(copy.deepcopy(interface))
        for interface in source_joint.findall("state_interface"):
            state = copy.deepcopy(interface)
            if state.get("name") == "position":
                initial = ET.SubElement(
                    state, "param", {"name": "initial_value"})
                initial.text = "0.0"
            sim_joint.append(state)
        # GazeboSystem exposes the generalized force actually reported by the
        # simulated joint.  The hardware URDF deliberately has no effort state
        # yet, but diagnostic simulation needs it to distinguish a controller
        # command from the force which reached the plant.
        state_names = {
            state.get("name") for state in sim_joint.findall("state_interface")
        }
        if "effort" not in state_names:
            ET.SubElement(
                sim_joint, "state_interface", {"name": "effort"})

    control_index = list(root).index(source_control)
    root.remove(source_control)
    root.insert(control_index, gazebo_control)

    # Gazebo Classic đổi package:// thành model:// nhưng máy này không đăng ký
    # ament package như một Gazebo model. Dùng file:// tuyệt đối trong bản sinh
    # runtime; URDF nguồn và robot_state_publisher vẫn chỉ có một nguồn mesh.
    package_root = os.path.dirname(os.path.dirname(os.path.abspath(urdf_path)))
    package_prefix = "package://gim_arm_description/"
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith(package_prefix):
            relative = filename[len(package_prefix):]
            absolute_mesh = os.path.join(package_root, relative)
            mesh.set("filename", "file://" + absolute_mesh)

    # Các STL xuất từ toàn bộ cụm SolidWorks phù hợp để hiển thị nhưng không
    # phải collision mesh lồi. Chúng giao nhau ở ổ trục và cơ cấu ghế, khiến
    # ODE sinh xung lực rất lớn dù gravity = 0. Bài mô phỏng controller không
    # cần tiếp xúc môi trường, nên chỉ bỏ collision khỏi bản URDF runtime.
    for link in root.findall("link"):
        for collision in link.findall("collision"):
            link.remove(collision)

    # Khôi phục bộ ma sát cơ học trước thử nghiệm không ma sát.
    # Đây là các hệ số ước lượng, không phải gain phản hồi của controller.
    for name, gear_ratio in GEAR_RATIOS.items():
        joint = root.find(f"joint[@name='{name}']")
        dynamics = joint.find("dynamics")
        if dynamics is None:
            dynamics = ET.SubElement(joint, "dynamics")
        dynamics.set("damping", str(VISCOUS_PER_N2 * gear_ratio ** 2))
        dynamics.set("friction", str(DRY_FRICTION_PER_N * gear_ratio))

    # This link exists only in the Gazebo runtime description.  ArmDynamics
    # continues to read the source URDF, so the controller does not compensate
    # this mass and sees it as an unknown payload/disturbance.
    _add_point_payload(root, payload_mass_kg, payload_offset_xyz)

    gazebo = ET.SubElement(root, "gazebo")
    plugin = ET.SubElement(
        gazebo, "plugin",
        {"filename": "libgazebo_ros2_control.so",
         "name": "gazebo_ros2_control"})
    ET.SubElement(plugin, "parameters").text = str(controllers_path)

    return ET.tostring(root, encoding="unicode")
