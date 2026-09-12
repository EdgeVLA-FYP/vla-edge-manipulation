"""Tests for the Phase 2 recording pipeline (issue #8).

Uses a small fake controller for the recording-loop tests (retry/discard
logic, dataset-level validation) rather than real, ~2800-step episodes —
the controller's own state-machine correctness is tests/test_pick_place_
controller.py's job. `record_dataset()` is exercised end-to-end via the
real MuJoCoBackend for observation capture, just with a short fake episode.
"""

import numpy as np
import pytest

pytest.importorskip("mujoco")
pytest.importorskip("yaml")
pytest.importorskip("lerobot")

from vla_edge_manipulation.backends.sim import MuJoCoBackend  # noqa: E402
from vla_edge_manipulation.recording import (  # noqa: E402
    _build_frame,
    _check_gripper_spans_full_range,
    _check_no_dropped_frames,
    _check_no_nans,
    _check_single_task,
    dataset_features,
    record_dataset,
    validate_recorded_dataset,
)
from vla_edge_manipulation.schema import (  # noqa: E402
    ACTION_DIM,
    CAMERA_KEYS,
    GRIPPER_MAX,
    GRIPPER_MIN,
    IMAGE_SHAPE,
    JOINT_NAMES,
    STATE_DIM,
)

_TASK = "Pick up the red cube and place it in the blue box"


class _FakeController:
    """Scripted controller: each entry in `outcomes` is one attempt's
    success flag; `frames_per_attempt` actions of `gripper_value` are fed
    to `on_action` per attempt via a connected backend."""

    def __init__(self, backend, outcomes, frames_per_attempt=3, gripper_value=GRIPPER_MAX):
        self._backend = backend
        self._outcomes = iter(outcomes)
        self._frames_per_attempt = frames_per_attempt
        self._gripper_value = gripper_value

    def run_episode(self, on_action):
        action = np.zeros(ACTION_DIM, dtype=np.float32)
        action[-1] = self._gripper_value
        for _ in range(self._frames_per_attempt):
            on_action(action)
        return next(self._outcomes)


class _SwingController:
    """One episode: gripper commanded fully open, then fully closed --
    satisfies _check_gripper_spans_full_range."""

    def __init__(self, backend):
        self._backend = backend

    def run_episode(self, on_action):
        for gripper_value in (GRIPPER_MAX, GRIPPER_MIN):
            action = np.zeros(ACTION_DIM, dtype=np.float32)
            action[-1] = gripper_value
            on_action(action)
        return True


@pytest.fixture
def backend():
    b = MuJoCoBackend()
    b.connect()
    yield b
    b.disconnect()


def test_dataset_features_match_schema():
    features = dataset_features()
    assert features["observation.state"]["shape"] == (STATE_DIM,)
    assert features["observation.state"]["names"] == JOINT_NAMES
    assert features["action"]["shape"] == (ACTION_DIM,)
    for camera in CAMERA_KEYS:
        assert features[f"observation.images.{camera}"]["shape"] == IMAGE_SHAPE


def test_build_frame_rejects_bad_action(backend):
    obs = backend.get_observation()
    bad_action = np.zeros(ACTION_DIM - 1, dtype=np.float32)  # wrong shape
    with pytest.raises(ValueError, match="shape"):
        _build_frame(dataset_features(), obs, bad_action, _TASK)


def test_build_frame_rejects_bad_observation(backend):
    obs = backend.get_observation()
    del obs[f"observation.images.{CAMERA_KEYS[0]}"]
    action = np.zeros(ACTION_DIM, dtype=np.float32)
    with pytest.raises(KeyError):
        _build_frame(dataset_features(), obs, action, _TASK)


def test_record_dataset_discards_failed_attempts_and_keeps_successes(backend, tmp_path):
    controller = _FakeController(backend, outcomes=[False, True, False, True])
    dataset = record_dataset(
        backend, controller, "test/discard", _TASK, n_episodes=2, root=tmp_path / "ds"
    )
    assert dataset.num_episodes == 2
    assert dataset.num_frames == 2 * 3  # frames_per_attempt, only successful attempts


def test_record_dataset_raises_when_max_attempts_exhausted(backend, tmp_path):
    controller = _FakeController(backend, outcomes=[False, False, False])
    with pytest.raises(RuntimeError, match="max_attempts"):
        record_dataset(
            backend,
            controller,
            "test/exhausted",
            _TASK,
            n_episodes=1,
            root=tmp_path / "ds",
            max_attempts=3,
        )


def test_record_dataset_output_passes_validation(backend, tmp_path):
    controller = _SwingController(backend)
    dataset = record_dataset(
        backend, controller, "test/valid", _TASK, n_episodes=1, root=tmp_path / "ds"
    )
    validate_recorded_dataset(dataset)


def test_check_no_nans_raises_on_nan():
    bad = np.array([1.0, np.nan, 2.0])
    with pytest.raises(ValueError, match="NaN"):
        _check_no_nans(bad, "observation.state")
    _check_no_nans(np.array([1.0, 2.0]), "observation.state")  # no raise


def test_check_no_dropped_frames_raises_on_gap():
    with pytest.raises(ValueError, match="dropped"):
        _check_no_dropped_frames(np.array([0, 1, 3]), np.array([0, 0, 0]))
    _check_no_dropped_frames(np.array([0, 1, 2, 0, 1]), np.array([0, 0, 0, 1, 1]))  # no raise


def test_check_single_task_raises_on_multiple():
    with pytest.raises(ValueError, match="single task"):
        _check_single_task(np.array([0, 0, 1]))
    _check_single_task(np.array([0, 0, 0]))  # no raise


def test_check_gripper_spans_full_range_raises_when_stuck():
    stuck = np.full(10, 50.0)
    with pytest.raises(ValueError, match="GRIPPER_MIN"):
        _check_gripper_spans_full_range(stuck)
    spanning = np.array([GRIPPER_MIN, 50.0, GRIPPER_MAX])
    _check_gripper_spans_full_range(spanning)  # no raise
