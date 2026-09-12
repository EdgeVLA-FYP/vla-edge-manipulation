"""MuJoCo-backed RobotBackend for the SO-101 arm — no hardware required.

Scene/meshes are vendored under assets/robotstudio_so101/ (see NOTICE.md
there for source and license). Unit convention for this backend: the five
arm joints are in radians (MuJoCo's native unit, set by the scene's
`<compiler angle="radian">`); the gripper is remapped to the LeRobot 0-100
scale per schema.GRIPPER_MIN/MAX. Any future backend must match this
convention or document why it diverges.
"""

from __future__ import annotations

import warnings
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from vla_edge_manipulation.backends.base import RobotBackend
from vla_edge_manipulation.config import default_config_path, load_config, repo_root
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


def _lookat_quat(cam_pos: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Camera orientation (wxyz) looking from cam_pos toward target, world +Z up."""
    forward = target - cam_pos
    forward /= np.linalg.norm(forward)
    world_up = (0.0, 0.0, 1.0) if abs(forward[2]) < 0.999 else (1.0, 0.0, 0.0)
    right = np.cross(forward, world_up)
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    # MuJoCo camera convention: local -Z is the view direction, +Y is up.
    rot = np.stack([right, up, -forward], axis=1)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, rot.flatten())
    return quat


class MuJoCoBackend(RobotBackend):
    """Drives the SO-101 MuJoCo simulation for pipeline rehearsal and eval."""

    # Fixed jaw's fingertip position (metres, "gripper" body's local frame),
    # measured from its mesh vertices — the vendored "gripperframe" site
    # doesn't track the moving jaw. See docs/decisions.md.
    _GRIPPER_TCP_OFFSET = np.array([-0.01103445, -0.00021821, -0.104416])
    _IK_MAX_ITERS = 200
    _IK_TOL = 1e-3  # metres — tight enough for a 2cm cube, loose enough to exit in a few iterations
    _IK_DAMPING = 0.01
    _IK_STEP_SCALE = 0.8
    # Position-only IK approached at a shallow diagonal, hitting the table
    # with the wrist housing above the actual target. Regularizing toward a
    # top-down orientation fixes it. See docs/decisions.md.
    _IK_DOWN_WORLD = np.array([0.0, 0.0, -1.0])
    _IK_ORIENT_WEIGHT = 0.02

    def __init__(self, config_path: str | Path | None = None, seed: int | None = None):
        self._config_path = Path(config_path) if config_path else default_config_path("robot_sim")
        self._rng = np.random.default_rng(seed)
        self._model: mujoco.MjModel | None = None
        self._data: mujoco.MjData | None = None
        self._ik_scratch: mujoco.MjData | None = None
        self._renderer: mujoco.Renderer | None = None
        self._scene_path: Path | None = None
        self._n_substeps = 1
        self._qpos_adr = np.zeros(len(JOINT_NAMES), dtype=int)
        self._dof_adr = np.zeros(len(JOINT_NAMES) - 1, dtype=int)
        self._actuator_id = np.zeros(len(JOINT_NAMES), dtype=int)
        self._joint_range = np.zeros((len(JOINT_NAMES), 2))
        self._gripper_body_id = 0
        self._cube_qpos_adr = 0
        self._cube_half_size = 0.01
        self._cube_area_half_extent: np.ndarray = np.zeros(2)
        self._cube_home_center = np.zeros(2)

    def connect(self) -> None:
        config = load_config(self._config_path)
        scene_path = repo_root() / config["scene_path"]
        self._scene_path = scene_path

        render_shape = (config["render_height"], config["render_width"], 3)
        if render_shape != IMAGE_SHAPE:
            raise ValueError(
                f"{self._config_path}: render size {render_shape[:2]} != "
                f"schema.IMAGE_SHAPE {IMAGE_SHAPE[:2]}"
            )

        self._model = mujoco.MjModel.from_xml_path(str(scene_path))
        self._model.opt.timestep = config["physics_timestep"]
        self._data = mujoco.MjData(self._model)
        self._ik_scratch = mujoco.MjData(self._model)
        self._renderer = mujoco.Renderer(
            self._model, height=config["render_height"], width=config["render_width"]
        )
        mujoco.mj_forward(
            self._model, self._data
        )  # populate xpos for the home-position lookup below

        for i, name in enumerate(JOINT_NAMES):
            joint_id = self._mj_id(mujoco.mjtObj.mjOBJ_JOINT, name, scene_path)
            actuator_id = self._mj_id(mujoco.mjtObj.mjOBJ_ACTUATOR, name, scene_path)
            self._qpos_adr[i] = self._model.jnt_qposadr[joint_id]
            self._actuator_id[i] = actuator_id
            self._joint_range[i] = self._model.jnt_range[joint_id]
            if i < len(self._dof_adr):  # arm joints only, gripper excluded
                self._dof_adr[i] = self._model.jnt_dofadr[joint_id]
        # Gripper joint (last row): min=closed (~6mm gap), max=open (~141mm)
        # — matches schema's 0=closed/100=open directly, no reversal needed
        # in _schema_to_joint_gripper/_joint_to_schema_gripper below. See
        # docs/decisions.md.
        for camera in CAMERA_KEYS:
            self._mj_id(mujoco.mjtObj.mjOBJ_CAMERA, camera, scene_path)

        self._gripper_body_id = self._mj_id(mujoco.mjtObj.mjOBJ_BODY, "gripper", scene_path)
        self._cube_home_center = self._data.xpos[self._gripper_body_id][:2].copy()

        cube_body_id = self._mj_id(mujoco.mjtObj.mjOBJ_BODY, "cube", scene_path)
        self._cube_qpos_adr = self._model.jnt_qposadr[self._model.body_jntadr[cube_body_id]]
        cube_area_cm = np.asarray(config["randomization"]["cube_area_cm"], dtype=float)
        self._cube_area_half_extent = (cube_area_cm / 100.0) / 2.0

        self._apply_workspace_config(config["workspace"], scene_path)

        self._n_substeps = max(1, round((1.0 / FPS) / self._model.opt.timestep))
        self.reset_to_home()

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
        """Resets the arm to its zero pose. Also re-randomizes the cube's start
        position within cube_area_cm"""
        if self._model is None or self._data is None:
            raise RuntimeError("connect() not called")
        mujoco.mj_resetData(self._model, self._data)
        offset = self._rng.uniform(-self._cube_area_half_extent, self._cube_area_half_extent)
        cube_xy = self._cube_home_center + offset
        self._data.qpos[self._cube_qpos_adr : self._cube_qpos_adr + 3] = [
            *cube_xy,
            self._cube_half_size,
        ]
        self._data.qpos[self._cube_qpos_adr + 3 : self._cube_qpos_adr + 7] = [1, 0, 0, 0]
        mujoco.mj_forward(self._model, self._data)

    def disconnect(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        self._model = None
        self._data = None
        self._ik_scratch = None

    def get_body_pose(self, name: str) -> tuple[np.ndarray, np.ndarray]:
        """Ground-truth world-frame (position, quaternion[wxyz]) of any named
        body — e.g. "cube" or "bin". Sim-only: real hardware has no
        equivalent, this is exactly the perception problem it would need to
        solve instead."""
        if self._data is None or self._scene_path is None:
            raise RuntimeError("connect() not called")
        body_id = self._mj_id(mujoco.mjtObj.mjOBJ_BODY, name, self._scene_path)
        return self._data.xpos[body_id].copy(), self._data.xquat[body_id].copy()

    def solve_ik(self, target_pos: np.ndarray, seed: np.ndarray | None = None) -> np.ndarray:
        """Damped-least-squares IK: arm-joint radians (JOINT_NAMES[:-1])
        bringing the gripper's tool-centre-point (_GRIPPER_TCP_OFFSET) to a
        world-frame Cartesian target, softly regularized toward a top-down
        orientation.

        wrist_roll is held at 0, not solved — free, it drifts between solves
        for the same target and swings the offset fingertip sideways. The
        remaining 4 DOF resolve redundancy via minimal movement from `seed`.

        If position doesn't converge, retries with orientation relaxed
        before giving up; still warns (`UserWarning`) and returns a
        best-effort solution if that fails too — never raises.

        Runs on a scratch MjData; never disturbs the live simulation.
        Sim-only: real hardware has no ground-truth Jacobian to solve with.
        """
        if self._model is None or self._data is None or self._ik_scratch is None:
            raise RuntimeError("connect() not called")
        q_seed = (
            self._data.qpos[self._qpos_adr[:4]].copy()
            if seed is None
            else np.asarray(seed, dtype=np.float64)[:4].copy()
        )
        q, converged, residual = self._solve_ik_dls(target_pos, q_seed, self._IK_ORIENT_WEIGHT)
        if not converged:
            # Relax the orientation regularizer progressively rather than
            # dropping it straight to 0 — that lets the redundant DOF settle
            # on a worse configuration (see docs/decisions.md). Only reached
            # when the default, fully-oriented solve already failed.
            for orient_weight in (self._IK_ORIENT_WEIGHT / 2, 0.0):
                q, converged, residual = self._solve_ik_dls(target_pos, q_seed, orient_weight)
                if converged:
                    break
        if not converged:
            warnings.warn(
                f"solve_ik: no convergence within {self._IK_MAX_ITERS} iterations "
                f"({residual * 1000:.1f}mm residual) for target {target_pos}, even "
                "position-only — likely unreachable at this orientation.",
                stacklevel=2,
            )
        return np.array([*q, 0.0])

    def _solve_ik_dls(
        self, target_pos: np.ndarray, q_seed: np.ndarray, orient_weight: float
    ) -> tuple[np.ndarray, bool, float]:
        """One damped-least-squares solve attempt; see solve_ik(). Returns
        (joint angles, whether position converged within _IK_TOL, final
        position residual in metres)."""
        assert self._model is not None and self._data is not None and self._ik_scratch is not None
        scratch = self._ik_scratch
        scratch.qpos[:] = self._data.qpos
        scratch.qpos[self._qpos_adr[4]] = 0.0  # wrist_roll, fixed
        q = q_seed.copy()
        # Margin keeps a limit-hugging solution valid after the float32 cast
        # send_action()/schema expect — float64-exact clipping can round back
        # outside jnt_range once narrowed to float32.
        margin = 1e-3
        lo = self._joint_range[:4, 0] + margin
        hi = self._joint_range[:4, 1] - margin
        dof_adr = self._dof_adr[:4]
        approach_local = self._GRIPPER_TCP_OFFSET / np.linalg.norm(self._GRIPPER_TCP_OFFSET)
        jacp = np.zeros((3, self._model.nv))
        jacr = np.zeros((3, self._model.nv))
        for _ in range(self._IK_MAX_ITERS):
            scratch.qpos[self._qpos_adr[:4]] = q
            mujoco.mj_forward(self._model, scratch)
            rot = scratch.xmat[self._gripper_body_id].reshape(3, 3)
            tcp_pos = scratch.xpos[self._gripper_body_id] + rot @ self._GRIPPER_TCP_OFFSET
            pos_error = target_pos - tcp_pos
            residual = float(np.linalg.norm(pos_error))
            if residual < self._IK_TOL:
                return q, True, residual
            # Orientation is a soft regularizer (biases the redundant DOF, not
            # a hard target) — an exact-zero tilt residual isn't guaranteed to
            # exist, so only position gates convergence.
            orient_error = np.cross(rot @ approach_local, self._IK_DOWN_WORLD)
            mujoco.mj_jac(self._model, scratch, jacp, jacr, tcp_pos, self._gripper_body_id)
            j = np.vstack([jacp[:, dof_adr], orient_weight * jacr[:, dof_adr]])
            error = np.concatenate([pos_error, orient_weight * orient_error])
            damped = j @ j.T + (self._IK_DAMPING**2) * np.eye(6)
            q = np.clip(q + self._IK_STEP_SCALE * (j.T @ np.linalg.solve(damped, error)), lo, hi)
        return q, False, residual

    def launch_interactive_viewer(self) -> None:
        """Opens MuJoCo's interactive viewer on the connected scene for manual
        inspection"""
        if self._model is None or self._data is None:
            raise RuntimeError("connect() not called")
        import mujoco.viewer

        mujoco.viewer.launch(self._model, self._data)

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
        return lo + frac_open * (hi - lo)

    def _joint_to_schema_gripper(self, joint_value: float) -> float:
        lo, hi = self._joint_range[-1]
        frac_open = (joint_value - lo) / (hi - lo)
        return frac_open * (GRIPPER_MAX - GRIPPER_MIN) + GRIPPER_MIN

    def _mj_id(self, objtype: mujoco.mjtObj, name: str, scene_path: Path) -> int:
        assert self._model is not None
        id_ = mujoco.mj_name2id(self._model, objtype, name)
        if id_ < 0:
            raise ValueError(f"{scene_path}: missing {objtype.name} named {name!r}")
        return id_

    def _apply_workspace_config(self, workspace: dict[str, Any], scene_path: Path) -> None:
        """Overrides scene appearance/geometry from configs/robot_sim.yaml"""
        assert self._model is not None

        self._cube_half_size = workspace["cube_size_cm"] / 100.0 / 2.0
        cube_geom = self._mj_id(mujoco.mjtObj.mjOBJ_GEOM, "cube", scene_path)
        self._model.geom_size[cube_geom] = [self._cube_half_size] * 3
        self._model.geom_rgba[cube_geom] = [*workspace["cube_color"], 1.0]
        self._model.geom_friction[cube_geom] = workspace["cube_friction"]

        table_geom = self._mj_id(mujoco.mjtObj.mjOBJ_GEOM, "table", scene_path)
        self._model.geom_rgba[table_geom] = [*workspace["table_color"], 1.0]
        self._model.geom_friction[table_geom] = workspace["table_friction"]

        bin_body = self._mj_id(mujoco.mjtObj.mjOBJ_BODY, "bin", scene_path)
        bin_xy = self._model.body_pos[bin_body][:2]
        wall_geoms = []
        for wall in ("bin_wall_north", "bin_wall_south", "bin_wall_east", "bin_wall_west"):
            geom = self._mj_id(mujoco.mjtObj.mjOBJ_GEOM, wall, scene_path)
            self._model.geom_rgba[geom] = [*workspace["bin_color"], 1.0]
            wall_geoms.append(geom)
        # Outer footprint from the actual wall geometry, not a hardcoded number
        bin_half_extent = np.max(
            [
                np.abs(self._model.geom_pos[g][:2]) + self._model.geom_size[g][:2]
                for g in wall_geoms
            ],
            axis=0,
        )
        spawn_lo = self._cube_home_center - self._cube_area_half_extent
        spawn_hi = self._cube_home_center + self._cube_area_half_extent
        bin_lo, bin_hi = bin_xy - bin_half_extent, bin_xy + bin_half_extent
        if np.all(spawn_lo < bin_hi) and np.all(bin_lo < spawn_hi):
            raise ValueError(
                f"{self._config_path}: cube spawn region (±{self._cube_area_half_extent} m "
                f"around {self._cube_home_center}) overlaps the bin footprint "
                f"(±{bin_half_extent} m around {bin_xy}) — shrink randomization.cube_area_cm "
                "or move the bin"
            )

        front_cam = self._mj_id(mujoco.mjtObj.mjOBJ_CAMERA, "front", scene_path)
        cam_pos = np.asarray(workspace["front_camera_pos"], dtype=float)
        target_xy = (self._cube_home_center + bin_xy) / 2.0
        self._model.cam_pos[front_cam] = cam_pos
        self._model.cam_quat[front_cam] = _lookat_quat(cam_pos, np.array([*target_xy, 0.02]))
