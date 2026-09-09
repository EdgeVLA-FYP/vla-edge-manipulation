"""Regressions for the pick-cube task workspace (cube + bin) baked into
assets/robotstudio_so101/so101.xml — see configs/task_pickcube.yaml.example.
"""

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")

import mujoco  # noqa: E402

_SCENE_PATH = Path(__file__).resolve().parents[1] / "assets" / "robotstudio_so101" / "so101.xml"


@pytest.fixture
def model():
    return mujoco.MjModel.from_xml_path(str(_SCENE_PATH))


def test_cube_and_bin_present(model):
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube") >= 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "bin") >= 0


def test_cube_settles_on_floor_without_exploding(model):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    cube_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    for _ in range(500):
        mujoco.mj_step(model, data)
    pos = data.xpos[cube_body]
    assert np.all(np.isfinite(pos))
    assert pos[2] == pytest.approx(0.01, abs=0.01)  # resting on the floor, not sunk through it


def test_static_xml_cube_position_and_bin_do_not_overlap(model):
    # Sanity check on the fallback XML defaults only — these aren't what
    # MuJoCoBackend actually uses at runtime (the real spawn center is the
    # live 'gripper' body xpos, not this static cube body_pos). The real,
    # config-driven invariant is enforced in MuJoCoBackend.connect() and
    # covered by test_sim_backend.py::test_overlapping_cube_area_raises.
    cube_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    bin_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "bin")
    cube_xy = model.body_pos[cube_body][:2]
    bin_xy = model.body_pos[bin_body][:2]
    spawn_half = 0.10  # configs/robot_sim.yaml.example: randomization.cube_area_cm [20, 20]
    bin_half = 0.034  # outer wall extent, see so101.xml
    assert abs(cube_xy[1] - bin_xy[1]) > spawn_half + bin_half
