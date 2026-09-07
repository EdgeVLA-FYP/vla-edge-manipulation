"""The single source of truth for robot I/O conventions.

Every backend, the recorder, and training code imports from here. Nothing in
this file imports from elsewhere in the project. Changing it can silently
invalidate every dataset recorded so far — grep for usages and log the
change in docs/decisions.md before editing.
"""

from __future__ import annotations

import numpy as np

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

ACTION_DIM = 6
STATE_DIM = 6
FPS = 30

CAMERA_KEYS = ["front", "wrist"]
IMAGE_SHAPE = (480, 640, 3)  # (H, W, C), uint8

OBS_STATE_KEY = "observation.state"
OBS_IMAGE_KEY_TEMPLATE = "observation.images.{camera}"
ACTION_KEY = "action"

# LeRobot convention: 0 = fully closed, 100 = fully open.
GRIPPER_MIN = 0.0
GRIPPER_MAX = 100.0

# Placeholder task — change via config for a new task, not by editing this
# constant's usages elsewhere. See configs/task_pickcube.yaml.example.
TASK_TEXT = "Pick up the red cube and place it in the blue box"


def image_key(camera: str) -> str:
    if camera not in CAMERA_KEYS:
        raise ValueError(f"unknown camera {camera!r}, expected one of {CAMERA_KEYS}")
    return OBS_IMAGE_KEY_TEMPLATE.format(camera=camera)


def validate_action(action) -> None:
    arr = np.asarray(action)
    if arr.shape != (ACTION_DIM,):
        raise ValueError(f"action shape {arr.shape} != ({ACTION_DIM},)")
    if arr.dtype.kind not in "fi":
        raise ValueError(f"action dtype {arr.dtype} is not numeric")
    gripper = float(arr[-1])
    if not (GRIPPER_MIN <= gripper <= GRIPPER_MAX):
        raise ValueError(f"gripper value {gripper} outside [{GRIPPER_MIN}, {GRIPPER_MAX}]")


def validate_observation(obs: dict) -> None:
    if OBS_STATE_KEY not in obs:
        raise KeyError(f"observation missing {OBS_STATE_KEY!r}")
    state = np.asarray(obs[OBS_STATE_KEY])
    if state.shape != (STATE_DIM,):
        raise ValueError(f"state shape {state.shape} != ({STATE_DIM},)")

    for camera in CAMERA_KEYS:
        key = image_key(camera)
        if key not in obs:
            raise KeyError(f"observation missing {key!r}")
        img = np.asarray(obs[key])
        if tuple(img.shape) != IMAGE_SHAPE:
            raise ValueError(f"{key} shape {img.shape} != {IMAGE_SHAPE}")
        if img.dtype != np.uint8:
            raise ValueError(f"{key} dtype {img.dtype} != uint8")
