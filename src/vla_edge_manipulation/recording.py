"""Records PickPlaceController demonstrations into LeRobotDataset format
(issue #8). Depends on the `lerobot` package (`pip install -e ".[recording]"`),
not installed by the `sim` extra alone.

Only successful episodes (PickPlaceController's own success criterion) are
kept — a failed attempt is discarded and retried, never written with a
"failed" flag, since a policy shouldn't be trained to imitate a failure. See
docs/decisions.md.
"""

from __future__ import annotations

import argparse

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.feature_utils import (
    build_dataset_frame,
    combine_feature_dicts,
    hw_to_dataset_features,
)

from vla_edge_manipulation.backends.sim import MuJoCoBackend
from vla_edge_manipulation.config import default_config_path, load_config
from vla_edge_manipulation.controllers.pick_place_controller import PickPlaceController
from vla_edge_manipulation.schema import (
    ACTION_KEY,
    CAMERA_KEYS,
    FPS,
    GRIPPER_MAX,
    GRIPPER_MIN,
    IMAGE_SHAPE,
    JOINT_NAMES,
    OBS_STATE_KEY,
    image_key,
    validate_action,
    validate_observation,
)

_GRIPPER_INDEX = JOINT_NAMES.index("gripper")
# Allows recorded extremes to fall just short of GRIPPER_MIN/MAX (e.g. the
# ramped close/open never quite commands the exact endpoint) while still
# catching a genuinely stuck or miscalibrated gripper.
_GRIPPER_RANGE_TOLERANCE = 1.0


def dataset_features() -> dict:
    """LeRobotDataset feature spec derived from schema.py — the single
    source of truth for joint order, camera keys, and image shape."""
    obs_hw: dict[str, type | tuple] = dict.fromkeys(JOINT_NAMES, float)
    obs_hw.update(dict.fromkeys(CAMERA_KEYS, IMAGE_SHAPE))
    action_hw: dict[str, type | tuple] = dict.fromkeys(JOINT_NAMES, float)
    return combine_feature_dicts(
        hw_to_dataset_features(obs_hw, prefix=OBS_STR, use_video=True),
        hw_to_dataset_features(action_hw, prefix=ACTION, use_video=True),
    )


def _build_frame(features: dict, obs: dict, action: np.ndarray, task: str) -> dict:
    """One LeRobotDataset frame from a validated observation/action pair.
    Fails loudly (schema.py's validators raise) rather than writing bad data.
    """
    validate_observation(obs)
    validate_action(action)
    obs_values = dict(zip(JOINT_NAMES, obs[OBS_STATE_KEY], strict=True))
    obs_values.update({camera: obs[image_key(camera)] for camera in CAMERA_KEYS})
    action_values = dict(zip(JOINT_NAMES, action, strict=True))
    return {
        **build_dataset_frame(features, obs_values, OBS_STR),
        **build_dataset_frame(features, action_values, ACTION),
        "task": task,
    }


def record_dataset(
    backend: MuJoCoBackend,
    controller: PickPlaceController,
    repo_id: str,
    task: str,
    n_episodes: int,
    root: str | None = None,
    max_attempts: int | None = None,
) -> LeRobotDataset:
    """Records `n_episodes` successful pick-place demonstrations, retrying
    failed attempts rather than including them. Raises if `max_attempts`
    (default 3x `n_episodes`) is exhausted first — a sign grasp reliability
    has regressed well below the last confirmed measurement.
    """
    max_attempts = n_episodes * 3 if max_attempts is None else max_attempts
    dataset = LeRobotDataset.create(
        repo_id, fps=FPS, features=dataset_features(), root=root, video_backend="pyav"
    )

    successes = 0
    attempts = 0
    while successes < n_episodes:
        if attempts >= max_attempts:
            dataset.finalize()
            raise RuntimeError(
                f"only {successes}/{n_episodes} successful episodes in {attempts} attempts "
                f"(max_attempts={max_attempts}) — see docs/decisions.md"
            )
        attempts += 1

        def on_action(action: np.ndarray) -> None:
            obs = backend.get_observation()
            dataset.add_frame(_build_frame(dataset.features, obs, action, task))

        if controller.run_episode(on_action=on_action):
            dataset.save_episode()
            successes += 1
        else:
            dataset.clear_episode_buffer()

    dataset.finalize()
    return dataset


def _check_no_nans(array: np.ndarray, name: str) -> None:
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains NaN/inf")


def _check_no_dropped_frames(frame_index: np.ndarray, episode_index: np.ndarray) -> None:
    for ep in np.unique(episode_index):
        idx = frame_index[episode_index == ep]
        if not np.array_equal(idx, np.arange(len(idx))):
            raise ValueError(f"episode {ep} has dropped or reordered frames: {idx}")


def _check_single_task(task_index: np.ndarray) -> None:
    unique = np.unique(task_index)
    if len(unique) != 1:
        raise ValueError(f"expected a single task, found task_index values {unique}")


def _check_gripper_spans_full_range(gripper: np.ndarray) -> None:
    if gripper.min() > GRIPPER_MIN + _GRIPPER_RANGE_TOLERANCE:
        raise ValueError(
            f"gripper action never reaches GRIPPER_MIN={GRIPPER_MIN} (min={gripper.min():.1f})"
        )
    if gripper.max() < GRIPPER_MAX - _GRIPPER_RANGE_TOLERANCE:
        raise ValueError(
            f"gripper action never reaches GRIPPER_MAX={GRIPPER_MAX} (max={gripper.max():.1f})"
        )


def validate_recorded_dataset(dataset: LeRobotDataset) -> None:
    """Dataset-level checks beyond per-frame validation: FPS, no NaNs, no
    dropped frames, a single task, and the gripper actually spanning its
    full commanded range somewhere in the set. Raises on the first
    violation found — see the record-dataset skill.
    """
    if dataset.fps != FPS:
        raise ValueError(f"dataset fps {dataset.fps} != schema.FPS {FPS}")

    hf = dataset.hf_dataset
    action = np.asarray(hf[ACTION_KEY])
    _check_no_nans(np.asarray(hf[OBS_STATE_KEY]), OBS_STATE_KEY)
    _check_no_nans(action, ACTION_KEY)
    _check_no_dropped_frames(np.asarray(hf["frame_index"]), np.asarray(hf["episode_index"]))
    _check_single_task(np.asarray(hf["task_index"]))
    _check_gripper_spans_full_range(action[:, _GRIPPER_INDEX])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, required=True, help="number of successful episodes")
    parser.add_argument("--repo-id", required=True, help="e.g. EdgeVLA/so101_pickcube_sim_v1_20ep")
    parser.add_argument("--root", default=None, help="local dataset dir (default: HF_LEROBOT_HOME)")
    parser.add_argument("--push", action="store_true", help="push to the Hub after validation")
    args = parser.parse_args()

    task = load_config(default_config_path("task_pickcube"))["instruction"]
    backend = MuJoCoBackend()
    backend.connect()
    try:
        controller = PickPlaceController(backend)
        dataset = record_dataset(
            backend, controller, args.repo_id, task, args.episodes, root=args.root
        )
    finally:
        backend.disconnect()

    validate_recorded_dataset(dataset)
    print(
        f"recorded {dataset.num_episodes} episodes, {dataset.num_frames} frames -> {dataset.root}"
    )

    if args.push:
        dataset.push_to_hub()
        print(f"pushed to https://huggingface.co/datasets/{args.repo_id}")


if __name__ == "__main__":
    main()
