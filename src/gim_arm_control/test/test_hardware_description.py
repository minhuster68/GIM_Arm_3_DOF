"""Verify startup options reach hardware without changing joint calibration."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from gim_control.hardware_description import configure_hardware_description


def test_hardware_options_preserve_joint_configuration():
    path = Path(__file__).resolve().parents[2] / 'gim_arm_description/urdf/gim_arm.urdf'
    source = path.read_text()
    original = ET.fromstring(source)
    for zero in (False, True):
        updated = ET.fromstring(configure_hardware_description(source, 'can1', zero))
        hardware = updated.find('ros2_control/hardware')
        assert hardware.find("param[@name='can_interface']").text == 'can1'
        assert hardware.find("param[@name='set_zero_on_startup']").text == str(zero).lower()
        assert hardware.find("param[@name='zero_method']").text == 'can'
        for old_joint, new_joint in zip(
                original.findall('ros2_control/joint'), updated.findall('ros2_control/joint')):
            assert ET.tostring(old_joint) == ET.tostring(new_joint)


def test_software_zero_is_explicit_and_invalid_method_is_rejected():
    source = '<robot><ros2_control name="GimArmSystem"><hardware/></ros2_control></robot>'
    updated = ET.fromstring(configure_hardware_description(
        source, set_zero_on_startup=True, zero_method='software'))
    assert updated.find("ros2_control/hardware/param[@name='zero_method']").text == 'software'
    with pytest.raises(ValueError, match='zero_method'):
        configure_hardware_description(source, zero_method='invalid')


@pytest.mark.parametrize('selected', ('base', 'shoulder', 'elbow', 'shoulder_elbow'))
def test_isolated_torque_preserves_calibration_and_locks_other_joints(selected):
    path = Path(__file__).resolve().parents[2] / 'gim_arm_description/urdf/gim_arm.urdf'
    source = path.read_text()
    original = ET.fromstring(source)
    updated = ET.fromstring(configure_hardware_description(source, torque_joint=selected))
    for old_joint, new_joint in zip(
            original.findall('ros2_control/joint'), updated.findall('ros2_control/joint')):
        flags = new_joint.findall("param[@name='torque_mode_enable']")
        assert len(flags) == 1
        selected_names = ({'shoulder_joint', 'elbow_joint'} if selected == 'shoulder_elbow'
                          else {selected + '_joint'})
        assert flags[0].text == str(new_joint.get('name') in selected_names).lower()
        new_joint.remove(flags[0])
        old_flag = old_joint.find("param[@name='torque_mode_enable']")
        if old_flag is not None:
            old_joint.remove(old_flag)
        assert ET.tostring(new_joint) == ET.tostring(old_joint)


def test_isolated_torque_rejects_invalid_joint_and_missing_joint_configuration():
    source = '<robot><ros2_control name="GimArmSystem"><hardware/></ros2_control></robot>'
    with pytest.raises(ValueError, match='torque_joint'):
        configure_hardware_description(source, torque_joint='wrist')
    with pytest.raises(ValueError, match='thiếu'):
        configure_hardware_description(source, torque_joint='elbow')


def test_isolated_torque_replaces_existing_flags_without_duplicates():
    source = ('<robot><ros2_control name="GimArmSystem"><hardware/>'
              '<joint name="base_joint"><param name="torque_mode_enable">true</param></joint>'
              '<joint name="shoulder_joint"><param name="torque_mode_enable">true</param></joint>'
              '<joint name="elbow_joint"><param name="torque_mode_enable">false</param></joint>'
              '</ros2_control></robot>')
    updated = ET.fromstring(configure_hardware_description(source, torque_joint='elbow'))
    for joint in updated.findall('ros2_control/joint'):
        flags = joint.findall("param[@name='torque_mode_enable']")
        assert len(flags) == 1
        assert flags[0].text == str(joint.get('name') == 'elbow_joint').lower()
