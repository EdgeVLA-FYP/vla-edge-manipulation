"""Scripted IK pick-place controller — the ground-truth "expert" for Phase 2
dataset recording (issue #7/#8). Sim-only: depends on MuJoCoBackend's
ground-truth pose/IK accessors, which have no real-hardware equivalent.

Known limitation: measured success is 94/100
(`scripts/measure_pickplace_success.sh`, seed=42). Remaining failures are a
torque-imbalanced-grasp rotational-drift mechanism, not a trajectory bug —
see docs/decisions.md for the fix history and for why an early-warning
classifier wasn't added.
"""

from __future__ import annotations

from collections.abc import Iterator

import numpy as np

from vla_edge_manipulation.backends.sim import MuJoCoBackend
from vla_edge_manipulation.schema import GRIPPER_MAX, GRIPPER_MIN

_ARM_ZERO = np.zeros(5)


class PickPlaceController:
    """Drives a MuJoCoBackend through one pick-cube-place-in-bin attempt.

    `run()` yields one action per physics step; the caller applies each via
    `backend.send_action()` (and calls `backend.get_observation()` too, if
    recording) — this controller never touches the backend's step/observe
    calls directly, so it composes cleanly with both a plain success-rate
    check (see `run_episode`) and Phase 2's frame-by-frame recorder.
    """

    _APPROACH_HEIGHT = 0.08  # m above grasp height for hover waypoints
    # Retreat radially, then slide in at grasp height, instead of descending
    # straight down — avoids pushing the cube. See docs/decisions.md.
    _RETREAT_DIST = 0.035  # m
    _SLIDE_STOP_MULT = 1.2  # stop this many cube-half-sizes short of centre
    _GRASP_HEIGHT_OFFSET = -0.004  # m from cube centre; empirically tuned

    _DESCEND_WAYPOINTS = 12
    _SLIDE_WAYPOINTS = 30
    _TRANSPORT_WAYPOINTS = 12
    _WAYPOINT_STEPS = 20  # physics steps held per intermediate waypoint
    _REACH_STEPS = 300  # physics steps held for a large single-jump reach
    _RELEASE_STEPS = 200  # physics steps held while opening over the bin
    # Short deliberately: a long hold lets grip force relax before the lift
    # starts. See docs/decisions.md.
    _SETTLE_STEPS = 10
    _LIFT_STEPS = 300  # single-jump hold for the vertical lift
    # Bin footprint/wall height baked into so101.xml, also hardcoded in
    # test_task_scene.py (MuJoCoBackend doesn't expose scene geometry live).
    _BIN_HALF_EXTENT = 0.034
    _BIN_WALL_HEIGHT = 0.02
    _RELEASE_CLEARANCE = 0.015  # above the wall top; see docs/decisions.md

    def __init__(self, backend: MuJoCoBackend):
        self._backend = backend

    def run(self) -> Iterator[np.ndarray]:
        """Yields the full pick-place action sequence. Call
        `backend.reset_to_home()` before iterating."""
        cube_pos, _ = self._backend.get_body_pose("cube")
        bin_pos, _ = self._backend.get_body_pose("bin")
        cube_half_size = cube_pos[2]  # resting on the table: centre z == half-size

        radial = cube_pos[:2] / np.linalg.norm(cube_pos[:2])
        retreat = -radial * self._RETREAT_DIST
        grasp_z = cube_pos[2] + self._GRASP_HEIGHT_OFFSET
        slide_stop = -retreat / self._RETREAT_DIST * self._SLIDE_STOP_MULT * cube_half_size

        seed = _ARM_ZERO
        above = np.array([*(cube_pos[:2] + retreat), grasp_z + self._APPROACH_HEIGHT])
        seed, action = self._goto(above, GRIPPER_MAX, seed)
        yield from self._hold(action, self._REACH_STEPS)

        for i in range(1, self._DESCEND_WAYPOINTS + 1):
            z = above[2] + (grasp_z - above[2]) * i / self._DESCEND_WAYPOINTS
            target = np.array([*(cube_pos[:2] + retreat), z])
            seed, action = self._goto(target, GRIPPER_MAX, seed)
            yield from self._hold(action, self._WAYPOINT_STEPS)

        for i in range(1, self._SLIDE_WAYPOINTS + 1):
            off = retreat + (slide_stop - retreat) * i / self._SLIDE_WAYPOINTS
            target = np.array([*(cube_pos[:2] + off), grasp_z])
            seed, action = self._goto(target, GRIPPER_MAX, seed)
            yield from self._hold(action, self._WAYPOINT_STEPS)
        grasp_xy = cube_pos[:2] + slide_stop

        for schema_g in np.arange(GRIPPER_MAX, GRIPPER_MIN - 1, -1.0):
            action = np.array([*seed, schema_g], dtype=np.float32)
            yield from self._hold(action, 6)
        yield from self._hold(action, self._SETTLE_STEPS)  # settle at fully closed

        lift_target = np.array([*grasp_xy, grasp_z + self._APPROACH_HEIGHT])
        seed, lift_action = self._goto(lift_target, GRIPPER_MIN, seed)
        yield from self._hold(lift_action, self._LIFT_STEPS)

        # Interpolated, unlike the lift jump above: a single large horizontal
        # jump here shakes the grip loose, while the equally large vertical
        # lift jump is stable. See docs/decisions.md.
        transport_z = grasp_z + self._APPROACH_HEIGHT
        for i in range(1, self._TRANSPORT_WAYPOINTS + 1):
            xy = grasp_xy + (bin_pos[:2] - grasp_xy) * i / self._TRANSPORT_WAYPOINTS
            target = np.array([*xy, transport_z])
            seed, action = self._goto(target, GRIPPER_MIN, seed)
            yield from self._hold(action, self._WAYPOINT_STEPS)

        # Reach release position, then open separately — combining them let
        # the arm's motion fling the cube sideways as it released. See
        # docs/decisions.md.
        release_z = self._BIN_WALL_HEIGHT + cube_half_size + self._RELEASE_CLEARANCE
        release_target = np.array([*bin_pos[:2], release_z])
        seed, action = self._goto(release_target, GRIPPER_MIN, seed)
        yield from self._hold(action, self._WAYPOINT_STEPS)
        open_action = np.array([*seed, GRIPPER_MAX], dtype=np.float32)
        yield from self._hold(open_action, self._RELEASE_STEPS)

        home_action = np.array([*_ARM_ZERO, GRIPPER_MAX], dtype=np.float32)
        yield from self._hold(home_action, self._REACH_STEPS)

    def run_episode(self) -> bool:
        """Convenience wrapper for measuring success rate: resets, drives one
        full attempt through `backend`, and checks
        `cube_center_within_box_bounds` (configs/task_pickcube.yaml.example)
        against the cube's final resting position — a momentary lift that
        doesn't survive to release doesn't count.
        """
        self._backend.reset_to_home()
        bin_pos, _ = self._backend.get_body_pose("bin")
        for action in self.run():
            self._backend.send_action(action)
        cube_final, _ = self._backend.get_body_pose("cube")
        return bool(np.all(np.abs(cube_final[:2] - bin_pos[:2]) <= self._BIN_HALF_EXTENT))

    def _goto(
        self, target_pos: np.ndarray, gripper: float, seed: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        q = self._backend.solve_ik(target_pos, seed=seed)
        return q, np.array([*q, gripper], dtype=np.float32)

    @staticmethod
    def _hold(action: np.ndarray, steps: int) -> Iterator[np.ndarray]:
        for _ in range(steps):
            yield action
