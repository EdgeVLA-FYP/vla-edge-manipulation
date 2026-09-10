"""Regressions specific to MuJoCoBackend, beyond the shared conformance
suite in test_backends.py: gripper direction and joint ordering are the two
things a scene/calibration swap could silently invert.
"""

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")
pytest.importorskip("yaml")

import mujoco  # noqa: E402
import yaml  # noqa: E402

from vla_edge_manipulation.backends.sim import MuJoCoBackend, _lookat_quat  # noqa: E402
from vla_edge_manipulation.schema import GRIPPER_MAX, GRIPPER_MIN, JOINT_NAMES  # noqa: E402

_CONFIG_EXAMPLE = Path(__file__).resolve().parents[1] / "configs" / "robot_sim.yaml.example"


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
    lo, hi = backend._joint_range[-1]
    gripper_qpos_adr = backend._qpos_adr[-1]

    closed = np.zeros(len(JOINT_NAMES), dtype=np.float32)
    closed[-1] = GRIPPER_MIN
    obs = _settle(backend, closed)
    assert obs["observation.state"][-1] == pytest.approx(GRIPPER_MIN, abs=1.0)
    assert backend._data.qpos[gripper_qpos_adr] == pytest.approx(hi, abs=0.05)

    opened = np.zeros(len(JOINT_NAMES), dtype=np.float32)
    opened[-1] = GRIPPER_MAX
    obs = _settle(backend, opened)
    assert obs["observation.state"][-1] == pytest.approx(GRIPPER_MAX, abs=1.0)
    assert backend._data.qpos[gripper_qpos_adr] == pytest.approx(lo, abs=0.05)


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


def test_send_action_out_of_joint_range_raises(backend):
    action = np.array([999.0, 0.0, 0.0, 0.0, 0.0, 50.0], dtype=np.float32)
    with pytest.raises(ValueError, match="shoulder_pan"):
        backend.send_action(action)


def test_methods_reject_calls_after_disconnect():
    b = MuJoCoBackend()
    b.connect()
    b.disconnect()

    action = np.zeros(len(JOINT_NAMES), dtype=np.float32)
    with pytest.raises(RuntimeError):
        b.get_observation()
    with pytest.raises(RuntimeError):
        b.send_action(action)
    with pytest.raises(RuntimeError):
        b.reset_to_home()
    with pytest.raises(RuntimeError):
        b.launch_interactive_viewer()


def test_launch_interactive_viewer_rejects_before_connect():
    b = MuJoCoBackend()
    with pytest.raises(RuntimeError):
        b.launch_interactive_viewer()


def test_reset_to_home_randomizes_cube_within_configured_area(backend):
    positions = []
    for _ in range(5):
        backend.reset_to_home()
        adr = backend._cube_qpos_adr
        positions.append(backend._data.qpos[adr : adr + 2].copy())
    positions = np.array(positions)

    lo = backend._cube_home_center - backend._cube_area_half_extent
    hi = backend._cube_home_center + backend._cube_area_half_extent
    assert np.all((positions >= lo) & (positions <= hi))
    assert not np.allclose(positions[0], positions[1])


def test_same_seed_reproduces_cube_placement():
    def cube_xy_after_connect(seed):
        b = MuJoCoBackend(seed=seed)
        b.connect()
        adr = b._cube_qpos_adr
        xy = b._data.qpos[adr : adr + 2].copy()
        b.disconnect()
        return xy

    np.testing.assert_allclose(cube_xy_after_connect(7), cube_xy_after_connect(7))


def test_lookat_quat_handles_top_down_camera_without_nan():
    # forward parallel to world +Z (a straight-down mount) makes
    # cross(forward, +Z) the zero vector
    for cam_pos, target in [
        (np.array([0.3, -0.09, 1.0]), np.array([0.3, -0.09, 0.0])),  # straight down
        (np.array([0.3, -0.09, -1.0]), np.array([0.3, -0.09, 0.0])),  # straight up
    ]:
        quat = _lookat_quat(cam_pos, target)
        assert not np.any(np.isnan(quat))
        assert np.linalg.norm(quat) == pytest.approx(1.0)


def test_overlapping_cube_area_raises(tmp_path):
    config = yaml.safe_load(_CONFIG_EXAMPLE.read_text())
    config["randomization"]["cube_area_cm"] = [60, 60]
    config_path = tmp_path / "robot_sim.yaml"
    config_path.write_text(yaml.dump(config))

    b = MuJoCoBackend(config_path=config_path)
    with pytest.raises(ValueError, match="overlaps the bin"):
        b.connect()


def test_workspace_config_override_changes_cube_size(tmp_path):
    config = yaml.safe_load(_CONFIG_EXAMPLE.read_text())
    config["workspace"]["cube_size_cm"] = 4.0  # default is 2.0 — must actually differ
    config_path = tmp_path / "robot_sim.yaml"
    config_path.write_text(yaml.dump(config))

    b = MuJoCoBackend(config_path=config_path)
    b.connect()
    cube_geom = mujoco.mj_name2id(b._model, mujoco.mjtObj.mjOBJ_GEOM, "cube")
    np.testing.assert_allclose(b._model.geom_size[cube_geom], [0.02, 0.02, 0.02])
    b.disconnect()


def test_workspace_config_override_changes_friction(tmp_path):
    config = yaml.safe_load(_CONFIG_EXAMPLE.read_text())
    config["workspace"]["cube_friction"] = [0.3, 0.02, 0.0002]  # default is [1.5, 0.01, 0.0001]
    config_path = tmp_path / "robot_sim.yaml"
    config_path.write_text(yaml.dump(config))

    b = MuJoCoBackend(config_path=config_path)
    b.connect()
    cube_geom = mujoco.mj_name2id(b._model, mujoco.mjtObj.mjOBJ_GEOM, "cube")
    np.testing.assert_allclose(b._model.geom_friction[cube_geom], [0.3, 0.02, 0.0002])
    b.disconnect()


def test_config_missing_workspace_block_raises_clearly(tmp_path):
    # A pre-existing robot_sim.yaml from before the workspace: block existed
    # must not fail with a raw, unactionable KeyError deep inside connect().
    config = yaml.safe_load(_CONFIG_EXAMPLE.read_text())
    del config["workspace"]
    config_path = tmp_path / "robot_sim.yaml"
    config_path.write_text(yaml.dump(config))

    with pytest.raises(KeyError, match="workspace"):
        MuJoCoBackend(config_path=config_path).connect()


def test_config_missing_nested_workspace_key_raises_clearly(tmp_path):
    config = yaml.safe_load(_CONFIG_EXAMPLE.read_text())
    del config["workspace"]["table_friction"]
    config_path = tmp_path / "robot_sim.yaml"
    config_path.write_text(yaml.dump(config))

    with pytest.raises(KeyError, match="workspace.table_friction"):
        MuJoCoBackend(config_path=config_path).connect()
