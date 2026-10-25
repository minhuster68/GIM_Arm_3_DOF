"""Include velocity-estimator dynamics in the physical plant stability guard."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from gim_arm_controller_lqr.factory import LqrFactory
from gim_control.arm_dynamics import ArmDynamics


ROOT = Path(__file__).resolve().parents[3]


def build(overrides=None):
    path = ROOT / 'src/gim_arm_controller_lqr/config/lqr_circle_hardware.yaml'
    values = deepcopy(yaml.safe_load(path.read_text())['lqr_controller']['ros__parameters'])
    values.update(overrides or {})
    values['diagnostic_hold'] = False
    node = SimpleNamespace(
        declare_parameter=lambda n, v: SimpleNamespace(value=values.setdefault(n, v)),
        get_parameter=lambda n: SimpleNamespace(value=values[n]))
    factory = LqrFactory()
    factory.declare_parameters(node)
    dynamics = ArmDynamics(str(ROOT / 'src/gim_arm_description/urdf/gim_arm.urdf'))
    return factory.build(node, dynamics, dynamics.tau_max, 100.)


def test_filter_is_in_stability_model_without_changing_gains_or_feedforward():
    filtered = build()
    raw = build({'velocity_filter_type': 'none'})
    np.testing.assert_array_equal(filtered.K, raw.K)
    assert filtered._closed_loop_transition(np.zeros(3), np.zeros(3), filtered.K).shape == (15, 15)
    assert raw._closed_loop_transition(np.zeros(3), np.zeros(3), raw.K).shape == (9, 9)
    assert filtered.stability_report()['stable_discrete']
    for c in (filtered, raw):
        c.compute(np.zeros(3), np.zeros(3), np.full(3, .1), np.zeros(3), np.zeros(3), .01)
    np.testing.assert_array_equal(filtered.last['tau_ff'], raw.last['tau_ff'])


def test_too_slow_filter_is_rejected_before_runtime():
    with pytest.raises(ValueError, match='PHÂN KỲ'):
        build({'velocity_kalman_jerk_std': [30., 10., 40.]})


def test_unknown_filter_is_rejected():
    with pytest.raises(ValueError, match='velocity_filter_type'):
        build({'velocity_filter_type': 'typo'})
