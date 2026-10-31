"""Geometry, derivatives and Gazebo-only rendering/goal composition."""

import logging
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from gim_control.gazebo_description import build_gazebo_description, GAZEBO_BASE_HEIGHT_M
from gim_control.gazebo_trajectory_preview import reference_sdf, sample_motion
from gim_control.gim_arm_kinematics import GimArmKinematics
from gim_control.letter_trajectory import square_strokes
from gim_control.reference_trajectory import load_trajectory
from gim_control.sweep_trajectory import TOOL_OFFSET


ROOT = Path(__file__).resolve().parents[3]
URDF = str(ROOT/'src/gim_arm_description/urdf/gim_arm.urdf')


def test_square_front_dimensions_and_all_three_joints_move():
    profile = load_trajectory(URDF, '', logging.getLogger('square-test'), 'square')
    kin = GimArmKinematics(URDF, tool_offset_xyz=TOOL_OFFSET)
    q, qd, qdd = profile.at(np.linspace(0, profile.duration, 2701))
    xyz = np.array([kin.fk_position(value) for value in q])
    np.testing.assert_allclose(np.ptp(xyz[:, [0, 2]], axis=0), [.2, .2], atol=2e-5)
    assert np.ptp(xyz[:, 1]) > .01  # Not falsely advertised as a planar square.
    assert np.all(np.ptp(q, axis=0) > .1)
    assert np.isfinite(qd).all() and np.isfinite(qdd).all()


def test_goal_sampling_has_strict_times_and_c2_home_transitions():
    p = load_trajectory(URDF, '', logging.getLogger('square-test'), 'square')
    home = np.zeros(3)
    t, q, qd, qdd, phases = sample_motion(p, home)
    assert t[0] == 0 and t[-1] == 37.
    assert np.all(np.diff(t) > 0)
    np.testing.assert_allclose(q[[0, -1]], [home, home], atol=1e-12)
    for at in [0., 5., 32., 37.]:
        i = np.flatnonzero(np.isclose(t, at))[0]
        np.testing.assert_allclose(qd[i], 0, atol=1e-12)
        np.testing.assert_allclose(qdd[i], 0, atol=1e-12)
    assert set(phases) == {'APPROACH', 'TRACK', 'RETURN'}


def test_static_path_has_no_collision_and_uses_gazebo_height():
    points = np.array([[0., 0., .4], [.1, 0., .4], [.1, 0., .5]])
    root = ET.fromstring(reference_sdf(points))
    assert root.findtext('model/static') == 'true'
    assert not root.findall('.//collision')
    visuals = root.findall('.//visual')
    assert len(visuals) == 2
    pose = np.fromstring(visuals[0].findtext('pose'), sep=' ')
    np.testing.assert_allclose(pose[:3], [.05, 0., .4+GAZEBO_BASE_HEIGHT_M])


def test_tool_marker_is_opt_in_and_does_not_add_payload_or_collision():
    original = ET.fromstring(build_gazebo_description(URDF, '/tmp/controllers.yaml'))
    marked = ET.fromstring(build_gazebo_description(
        URDF, '/tmp/controllers.yaml', show_tool_marker=True, payload_offset_xyz=TOOL_OFFSET))
    assert original.find("link[@name='preview_tool_marker']") is None
    link = marked.find("link[@name='preview_tool_marker']")
    assert link.find('inertial') is None and link.find('collision') is None
    assert marked.find("link[@name='tool_payload_link']") is None
    joint = marked.find("joint[@name='preview_tool_fixed']")
    np.testing.assert_allclose(np.fromstring(joint.find('origin').get('xyz'), sep=' '), TOOL_OFFSET)
    assert marked.findtext('.//ros2_control/hardware/plugin') == 'gazebo_ros2_control/GazeboSystem'


@pytest.mark.parametrize('side,radius', [(0, .02), (.2, 0), (.2, .1), (.2, np.nan)])
def test_bad_square_geometry_is_rejected(side, radius):
    with pytest.raises(ValueError, match='square'):
        square_strokes(side, radius)
