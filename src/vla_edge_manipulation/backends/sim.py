"""MuJoCo-backed RobotBackend for the SO-101 arm — no hardware required.

Scene/meshes are vendored under assets/robotstudio_so101/ (see NOTICE.md
there for source and license). Unit convention for this backend: the five
arm joints are in radians (MuJoCo's native unit, set by the scene's
`<compiler angle="radian">`); the gripper is remapped to the LeRobot 0-100
scale per schema.GRIPPER_MIN/MAX. Any future backend must match this
convention or document why it diverges.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import yaml

from vla_edge_manipulation.backends.base import RobotBackend
from vla_edge_manipulation.schema import (
    CAMERA_KEYS,
    FPS,
    GRIPPER_MAX,
    GRIPPER_MIN,
    IMAGE_SHAPE,
    JOINT_NAMES,
    OBS_STATE_KEY,
    STATE_DIM,
    image_key,
    validate_action,
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _default_config_path() -> Path:
    return _repo_root() / "configs" / "robot_sim.yaml"


def _load_config(path: Path) -> dict[str, Any]:
    if not path.exists():
        example = path.with_suffix(".yaml.example")
        raise FileNotFoundError(
            f"{path} not found — copy {example.name} to {path.name} and adjust it"
        )
    with path.open() as f:
        config = yaml.safe_load(f)
    if not isinstance(config, dict):
        raise ValueError(f"{path}: expected a YAML mapping, got {type(config).__name__}")
    return config


class MuJoCoBackend(RobotBackend):
    """Drives the SO-101 MuJoCo simulation for pipeline rehearsal and eval."""

    def __init__(self, config_path: str | Path | None = None):
        self._config_path = Path(config_path) if config_path else _default_config_path()
        self._model: mujoco.MjModel | None = None
        self._data: mujoco.MjData | None = None
        self._renderer: mujoco.Renderer | None = None
        self._n_substeps = 1
        self._qpos_adr = np.zeros(len(JOINT_NAMES), dtype=int)
        self._actuator_id = np.zeros(len(JOINT_NAMES), dtype=int)
        self._joint_range = np.zeros((len(JOINT_NAMES), 2))

    def connect(self) -> None:
        config = _load_config(self._config_path)
        scene_path = _repo_root() / config["scene_path"]

        render_shape = (config["render_height"], config["render_width"], 3)
        if render_shape != IMAGE_SHAPE:
            raise ValueError(
                f"{self._config_path}: render size {render_shape[:2]} != "
                f"schema.IMAGE_SHAPE {IMAGE_SHAPE[:2]}"
            )

        self._model = mujoco.MjModel.from_xml_path(str(scene_path))
        self._model.opt.timestep = config["physics_timestep"]
        self._data = mujoco.MjData(self._model)
        self._renderer = mujoco.Renderer(
            self._model, height=config["render_height"], width=config["render_width"]
        )

        for i, name in enumerate(JOINT_NAMES):
            joint_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
            actuator_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if joint_id < 0 or actuator_id < 0:
                raise ValueError(f"{scene_path}: missing joint or actuator named {name!r}")
            self._qpos_adr[i] = self._model.jnt_qposadr[joint_id]
            self._actuator_id[i] = actuator_id
            self._joint_range[i] = self._model.jnt_range[joint_id]
        # Gripper joint (last row): empirically verified (mesh-to-mesh proximity,
        # not assumed) that its *max* is the closed position and its *min* is
        # open — opposite of schema's 0=closed/100=open, so the two ends swap
        # in _schema_to_joint_gripper/_joint_to_schema_gripper below.

        self._n_substeps = max(1, round((1.0 / FPS) / self._model.opt.timestep))
        mujoco.mj_resetData(self._model, self._data)
        mujoco.mj_forward(self._model, self._data)

    def get_observation(self) -> dict[str, np.ndarray]:
        if self._data is None or self._renderer is None:
            raise RuntimeError("connect() not called")
        state = np.empty(STATE_DIM, dtype=np.float32)
        state[:-1] = self._data.qpos[self._qpos_adr[:-1]]
        state[-1] = self._joint_to_schema_gripper(self._data.qpos[self._qpos_adr[-1]])

        obs: dict[str, np.ndarray] = {OBS_STATE_KEY: state}
        for camera in CAMERA_KEYS:
            self._renderer.update_scene(self._data, camera=camera)
            obs[image_key(camera)] = self._renderer.render()
        return obs

    def send_action(self, action: np.ndarray) -> None:
        if self._model is None or self._data is None:
            raise RuntimeError("connect() not called")
        validate_action(action)
        arr = np.asarray(action, dtype=np.float64)
        self._validate_arm_joint_range(arr[:-1])
        self._data.ctrl[self._actuator_id[:-1]] = arr[:-1]
        self._data.ctrl[self._actuator_id[-1]] = self._schema_to_joint_gripper(float(arr[-1]))
        for _ in range(self._n_substeps):
            mujoco.mj_step(self._model, self._data)

    def reset_to_home(self) -> None:
        if self._model is None or self._data is None:
            raise RuntimeError("connect() not called")
        mujoco.mj_resetData(self._model, self._data)
        mujoco.mj_forward(self._model, self._data)

    def disconnect(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def _validate_arm_joint_range(self, arm_values: np.ndarray) -> None:
        # The actuator only clamps *force*, not ctrl — an out-of-range target
        # would otherwise be accepted silently and just creep to the joint's
        # hard stop instead of erroring, unlike the gripper's schema check.
        lo, hi = self._joint_range[:-1, 0], self._joint_range[:-1, 1]
        bad = (arm_values < lo) | (arm_values > hi)
        if bad.any():
            i = int(np.flatnonzero(bad)[0])
            raise ValueError(
                f"action[{i}] ({JOINT_NAMES[i]}={arm_values[i]:.4f} rad) outside "
                f"joint range [{lo[i]:.4f}, {hi[i]:.4f}]"
            )

    def _schema_to_joint_gripper(self, value: float) -> float:
        lo, hi = self._joint_range[-1]
        frac_open = (value - GRIPPER_MIN) / (GRIPPER_MAX - GRIPPER_MIN)
        return hi - frac_open * (hi - lo)

    def _joint_to_schema_gripper(self, joint_value: float) -> float:
        lo, hi = self._joint_range[-1]
        frac_open = (hi - joint_value) / (hi - lo)
        return frac_open * (GRIPPER_MAX - GRIPPER_MIN) + GRIPPER_MIN
