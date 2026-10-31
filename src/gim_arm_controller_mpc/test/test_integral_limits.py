"""Per-joint I-state caps and compatibility with scalar MPC profiles."""

from types import SimpleNamespace

import numpy as np
import pytest

from gim_arm_controller_mpc.factory import MpcFactory


class Dynamics:
    nq = 3
    tau_max = np.array([5., 40., 5.])

    def mass_matrix(self, q):
        return np.eye(3)

    def inverse_dynamics(self, q, qd, qdd):
        return np.asarray(qdd) + np.asarray(q)


def controller(**overrides):
    values = dict(prediction_horizon=6, control_horizon=3,
                  qp_solver='active_set', integral_limit=.6)
    values.update(overrides)
    node = SimpleNamespace(
        declare_parameter=lambda name, default: values.setdefault(name, default),
        get_parameter=lambda name: SimpleNamespace(value=values[name]))
    factory = MpcFactory()
    factory.declare_parameters(node)
    return factory.build(node, Dynamics(), Dynamics.tau_max, 100.)


@pytest.mark.parametrize('mode', ['coupled', 'independent'])
@pytest.mark.parametrize('sign', [-1., 1.])
def test_elbow_can_accumulate_past_old_cap_while_other_joints_remain_capped(mode, sign):
    c = controller(feedback_model_mode=mode,
                   integral_limit_per_joint=[.6, .6, .9])
    zero = np.zeros(3)
    c.integral[:] = sign * .5999
    c.compute(np.full(3, sign*.1), zero, zero, zero, zero, .01)
    np.testing.assert_allclose(c.integral, sign*np.array([.6, .6, .6009]))
    c.integral[2] = sign * .8999
    c.compute(np.full(3, sign*.1), zero, zero, zero, zero, .01)
    np.testing.assert_allclose(c.integral, sign*np.array([.6, .6, .9]))
    c.reset()
    np.testing.assert_array_equal(c.integral, np.zeros(3))
    np.testing.assert_allclose(c.integral_limit, [.6, .6, .9])


def test_legacy_profile_keeps_scalar_limit_and_zero_remains_unlimited():
    assert controller(integral_limit=.6).integral_limit == .6
    assert np.isinf(controller(integral_limit=0.).integral_limit)


def test_zero_override_inherits_scalar_limit_per_joint():
    c = controller(integral_limit_per_joint=[0., 0., .9])
    np.testing.assert_allclose(c.integral_limit, [.6, .6, .9])
    c = controller(integral_limit=0., integral_limit_per_joint=[0., 0., .9])
    assert np.all(np.isinf(c.integral_limit[:2]))
    assert c.integral_limit[2] == .9


@pytest.mark.parametrize('limits', [[.6, .9], [.6, -.1, .9],
                                   [.6, np.nan, .9], [.6, .6, np.inf]])
def test_invalid_per_joint_limits_are_rejected(limits):
    with pytest.raises(ValueError, match='integral_limit'):
        controller(integral_limit_per_joint=limits)


@pytest.mark.parametrize('limit', [np.nan, np.inf, -np.inf])
def test_invalid_scalar_limit_is_rejected_even_with_joint_overrides(limit):
    with pytest.raises(ValueError, match='integral_limit'):
        controller(integral_limit=limit, integral_limit_per_joint=[.6, .6, .9])
