"""Check real-URDF geometry, shared endpoints, and reference derivatives."""

import logging
from pathlib import Path

import numpy as np
import pytest

from gim_control import sweep_trajectory as sweep
from gim_control.gim_arm_kinematics import GimArmKinematics
from gim_control.letter_trajectory import letter_strokes, stroke_positions
from gim_control.reference_trajectory import load_trajectory


@pytest.fixture(scope="module")
def references(tmp_path_factory):
    urdf = Path(__file__).resolve().parents[2] / "gim_arm_description/urdf/gim_arm.urdf"
    cache = tmp_path_factory.mktemp("letters") / "waypoints.npz"
    logger = logging.getLogger("letter-test")
    profiles = {shape: load_trajectory(str(urdf), str(cache), logger, shape)
                for shape in ("circle", "r", "a")}
    kin = GimArmKinematics(str(urdf), tool_offset_xyz=sweep.TOOL_OFFSET)
    return profiles, kin, urdf, cache


@pytest.mark.parametrize("shape", ("r", "a"))
def test_same_circle_endpoints_and_duration(references, shape):
    profiles, _, _, _ = references
    profile, circle = profiles[shape], profiles["circle"]
    assert profile.duration == circle.duration
    for time in (0.0, profile.duration):
        q, qd, qdd = profile.at(time)
        np.testing.assert_allclose(q, circle.at(time)[0], atol=1e-12)
        np.testing.assert_allclose(qd, 0.0, atol=1e-12)
        np.testing.assert_allclose(qdd, 0.0, atol=1e-12)


@pytest.mark.parametrize("shape", ("r", "a"))
def test_analytic_derivatives_match_position(references, shape):
    profile = references[0][shape]
    times = (profile.boundaries[:-1, None]
             + np.diff(profile.boundaries)[:, None] * [0.23, 0.47, 0.71]).ravel()
    step = 1e-4
    q, qd, qdd = profile.at(times)
    before, after = profile.at(times - step), profile.at(times + step)
    np.testing.assert_allclose(qd, (after[0] - before[0]) / (2*step), atol=1e-7)
    np.testing.assert_allclose(qdd, (after[1] - before[1]) / (2*step), atol=1e-7)
    assert np.max(np.abs(qd)) > 0.01
    assert np.max(np.abs(qdd)) > 0.01


@pytest.mark.parametrize("shape", ("r", "a"))
def test_stroke_joins_are_c2(references, shape):
    profile = references[0][shape]
    _, qd, qdd = profile.at(profile.boundaries)
    np.testing.assert_allclose(qd, 0.0, atol=1e-12)
    np.testing.assert_allclose(qdd, 0.0, atol=1e-12)
    for time in profile.boundaries[1:-1]:
        before, after = profile.at(time - 1e-6), profile.at(time + 1e-6)
        for left, right in zip(before, after):
            np.testing.assert_allclose(left, right, atol=2e-5)


@pytest.mark.parametrize("shape", ("r", "a"))
def test_front_view_preserves_letter_and_joint_limits(references, shape):
    profiles, kin, _, _ = references
    profile = profiles[shape]
    for index, control in enumerate(letter_strokes(shape)):
        # Recover the geometric parameter from the quintic timing law.
        u = np.linspace(0.0, 1.0, 201)
        h = 10*u**3 - 15*u**4 + 6*u**5
        expected = stroke_positions(control, profile.anchor, h)
        times = profile.boundaries[index] + u * np.diff(profile.boundaries)[index]
        q, _, _ = profile.at(times)
        actual = np.asarray([kin.fk_position(value) for value in q])
        assert np.max(np.linalg.norm(actual - expected, axis=1)) < 2e-6
        assert np.all(q >= kin.model.lowerPositionLimit + sweep.MIN_JOINT_MARGIN_RAD)
        assert np.all(q <= kin.model.upperPositionLimit - sweep.MIN_JOINT_MARGIN_RAD)
    q, qd, qdd = profile.at(np.linspace(0, profile.duration, 3001))
    assert all(np.isfinite(value).all() for value in (q, qd, qdd))
    assert np.all(np.max(np.abs(qd), axis=0)
                  < sweep.MAX_JOINT_SPEED_FRACTION * kin.model.velocityLimit)


def test_letter_selection_does_not_change_circle_cache(references):
    profiles, _, urdf, cache = references
    after = load_trajectory(str(urdf), str(cache), logging.getLogger("letter-test"))
    times = np.linspace(0, after.duration, 100)
    for actual, expected in zip(after.at(times), profiles["circle"].at(times)):
        np.testing.assert_array_equal(actual, expected)


def test_unknown_shape_is_rejected(references):
    _, _, urdf, cache = references
    with pytest.raises(ValueError, match="trajectory_shape"):
        load_trajectory(str(urdf), str(cache), logging.getLogger("letter-test"), "b")
