"""Regressions specific to MuJoCoBackend, beyond the shared conformance
suite in test_backends.py: gripper direction and joint ordering are the two
things a scene/calibration swap could silently invert.
"""

import numpy as np
import pytest

pytest.importorskip("mujoco")

from vla_edge_manipulation.backends.sim import MuJoCoBackend  # noqa: E402
from vla_edge_manipulation.schema import GRIPPER_MAX, GRIPPER_MIN, JOINT_NAMES  # noqa: E402


@pytest.fixture
def backend():
    b = MuJoCoBackend()
    b.connect()
    yield b
    b.disconnect()


def _settle(backend, action, steps=60):
    for _ in range(steps):
        backend.send_action(action)
    return backend.get_observation()


def test_gripper_min_is_closed_and_max_is_open(backend):
    closed = np.zeros(len(JOINT_NAMES), dtype=np.float32)
    closed[-1] = GRIPPER_MIN
    opened = np.zeros(len(JOINT_NAMES), dtype=np.float32)
    opened[-1] = GRIPPER_MAX

    assert _settle(backend, closed)["observation.state"][-1] == pytest.approx(GRIPPER_MIN, abs=1.0)
    assert _settle(backend, opened)["observation.state"][-1] == pytest.approx(GRIPPER_MAX, abs=1.0)


def test_arm_joints_track_commanded_radians_in_order(backend):
    action = np.array([0.3, -0.2, 0.4, 0.1, -0.5, 50.0], dtype=np.float32)
    state = _settle(backend, action)["observation.state"]
    np.testing.assert_allclose(state[:-1], action[:-1], atol=0.02)


def test_reset_to_home_returns_to_zero_arm_pose(backend):
    action = np.array([0.3, -0.2, 0.4, 0.1, -0.5, 50.0], dtype=np.float32)
    _settle(backend, action)

    backend.reset_to_home()
    state = backend.get_observation()["observation.state"]
    np.testing.assert_allclose(state[:-1], np.zeros(5), atol=1e-6)
