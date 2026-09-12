"""Tests for the scripted pick-place controller (Phase 1, GitHub issue #7).

Grasp success is measured, not asserted here: 93/100 (93%,
`scripts/measure_pickplace_success.sh`), up from a confirmed 77/100 baseline
after enabling torsional/rolling friction on the cube geom (condim=6) and
fixing two further failures found by classifying that gap: a bin-wall-clip
at release and a transport-jump grip disturbance (see docs/decisions.md).
Still noisy and not yet reliable enough to gate CI on. These tests instead
guard the parts that are actually correct and stable: the controller runs
to completion without error and only ever emits schema-valid actions.
"""

import numpy as np
import pytest

pytest.importorskip("mujoco")
pytest.importorskip("yaml")

from vla_edge_manipulation.backends.sim import MuJoCoBackend  # noqa: E402
from vla_edge_manipulation.controllers.pick_place_controller import (
    PickPlaceController,  # noqa: E402
)
from vla_edge_manipulation.schema import GRIPPER_MAX, GRIPPER_MIN, validate_action  # noqa: E402


@pytest.fixture
def backend():
    b = MuJoCoBackend(seed=0)
    b.connect()
    yield b
    b.disconnect()


def test_run_completes_and_emits_only_valid_actions(backend):
    backend.reset_to_home()
    controller = PickPlaceController(backend)
    n_actions = 0
    for action in controller.run():
        validate_action(action)
        backend.send_action(action)
        n_actions += 1
    assert n_actions > 0


def test_run_episode_returns_bool_across_seeds():
    for seed in range(3):
        backend = MuJoCoBackend(seed=seed)
        backend.connect()
        try:
            result = PickPlaceController(backend).run_episode()
            assert isinstance(result, bool)
        finally:
            backend.disconnect()


def test_run_never_moves_cube_before_the_gripper_reaches_it(backend):
    """The descend/slide-in waypoints must not bump the cube before the
    gripper is actually positioned to grasp -- a regression guard for the
    exact bug found during calibration (target = cube centre drove the jaw
    through the cube instead of stopping beside it)."""
    backend.reset_to_home()
    cube_start, _ = backend.get_body_pose("cube")
    controller = PickPlaceController(backend)

    actions = list(controller.run())
    n_descend = controller._DESCEND_WAYPOINTS * controller._WAYPOINT_STEPS
    for action in actions[: controller._REACH_STEPS + n_descend // 2]:
        backend.send_action(action)
    cube_mid, _ = backend.get_body_pose("cube")
    assert np.linalg.norm(cube_mid[:2] - cube_start[:2]) < 0.005


def test_lift_and_transport_hold_the_cube_up(backend):
    """Regression guard: a single large horizontal jump for the transport
    reach was found to transiently shake the grip loose, dropping the cube
    ~70mm before being re-caught (see docs/decisions.md) — well beyond the
    ~25mm of normal settling wobble a good grasp shows. Cube height must
    stay well above its lift peak for the whole lift+transport window,
    before the deliberate descent to release height begins."""
    backend.reset_to_home()
    controller = PickPlaceController(backend)
    actions = list(controller.run())

    n_close_ramp = len(np.arange(GRIPPER_MAX, GRIPPER_MIN - 1, -1.0)) * 6
    n_before_lift = (
        controller._REACH_STEPS
        + controller._DESCEND_WAYPOINTS * controller._WAYPOINT_STEPS
        + controller._SLIDE_WAYPOINTS * controller._WAYPOINT_STEPS
        + n_close_ramp
        + controller._SETTLE_STEPS
    )
    n_lift_and_transport = (
        controller._LIFT_STEPS + controller._TRANSPORT_WAYPOINTS * controller._WAYPOINT_STEPS
    )

    for action in actions[:n_before_lift]:
        backend.send_action(action)
    peak = backend.get_body_pose("cube")[0][2]
    for action in actions[n_before_lift : n_before_lift + n_lift_and_transport]:
        backend.send_action(action)
        cube_z = backend.get_body_pose("cube")[0][2]
        peak = max(peak, cube_z)
        assert cube_z > peak - 0.04


def test_release_does_not_move_the_arm_while_opening(backend):
    """Regression guard for the release-sequencing bug: opening the gripper
    while the arm is still moving to the release position flung the cube
    sideways instead of dropping it in place (see docs/decisions.md). Once
    the gripper starts opening, the arm's commanded joint targets must stay
    exactly fixed for the rest of the sequence."""
    backend.reset_to_home()
    controller = PickPlaceController(backend)
    actions = np.array(list(controller.run()))

    # The last fully-closed action is the moment right before the release
    # opens the gripper -- the following _RELEASE_STEPS actions are the
    # release sequence itself (after that, a separate "return home" motion
    # legitimately moves the arm again).
    last_closed = np.flatnonzero(actions[:, -1] == GRIPPER_MIN)[-1]
    release_start = last_closed + 1
    arm_during_release = actions[release_start : release_start + controller._RELEASE_STEPS, :-1]
    assert np.all(arm_during_release == arm_during_release[0])
