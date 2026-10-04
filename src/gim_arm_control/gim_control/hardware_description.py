"""Apply hardware launch options to an expanded URDF."""

import xml.etree.ElementTree as ET


def configure_hardware_description(
        urdf_xml, can_interface='can0', set_zero_on_startup=False, zero_method='can',
        torque_joint='all'):
    """Configure startup and selected torque joints without changing source URDF."""
    if zero_method not in ('can', 'software'):
        raise ValueError('zero_method phải là can hoặc software')
    joint_names = ('base_joint', 'shoulder_joint', 'elbow_joint')
    torque_selections = {
        'base': {'base_joint'},
        'shoulder': {'shoulder_joint'},
        'elbow': {'elbow_joint'},
        'shoulder_elbow': {'shoulder_joint', 'elbow_joint'},
    }
    if torque_joint != 'all' and torque_joint not in torque_selections:
        raise ValueError('torque_joint phải là all, base, shoulder, elbow hoặc shoulder_elbow')
    root = ET.fromstring(urdf_xml)
    hardware = root.find("ros2_control[@name='GimArmSystem']/hardware")
    if hardware is None:
        raise ValueError('URDF thiếu GimArmSystem/hardware')
    for name, value in (
            ('can_interface', can_interface),
            ('set_zero_on_startup', str(set_zero_on_startup).lower()),
            ('zero_method', zero_method)):
        param = hardware.find(f"param[@name='{name}']")
        if param is None:
            param = ET.SubElement(hardware, 'param', {'name': name})
        param.text = value
    if torque_joint != 'all':
        control = root.find("ros2_control[@name='GimArmSystem']")
        joints = {joint.get('name'): joint for joint in control.findall('joint')}
        if any(name not in joints for name in joint_names):
            raise ValueError('GimArmSystem thiếu base_joint, shoulder_joint hoặc elbow_joint')
        for name in joint_names:
            param = joints[name].find("param[@name='torque_mode_enable']")
            if param is None:
                param = ET.SubElement(joints[name], 'param', {'name': 'torque_mode_enable'})
            param.text = str(name in torque_selections[torque_joint]).lower()
    return ET.tostring(root, encoding='unicode')
