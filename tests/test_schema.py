import numpy as np
import pytest

from vla_edge_manipulation.schema import (
    ACTION_DIM,
    CAMERA_KEYS,
    GRIPPER_MAX,
    GRIPPER_MIN,
    IMAGE_SHAPE,
    OBS_STATE_KEY,
    STATE_DIM,
    image_key,
    validate_action,
    validate_observation,
)


def _valid_action():
    a = np.zeros(ACTION_DIM, dtype=np.float32)
    a[-1] = GRIPPER_MIN
    return a


def _valid_observation():
    obs = {OBS_STATE_KEY: np.zeros(STATE_DIM, dtype=np.float32)}
    for camera in CAMERA_KEYS:
        obs[image_key(camera)] = np.zeros(IMAGE_SHAPE, dtype=np.uint8)
    return obs


def test_valid_action_passes():
    validate_action(_valid_action())


def test_action_wrong_shape_raises():
    with pytest.raises(ValueError):
        validate_action(np.zeros(ACTION_DIM + 1))


def test_action_gripper_out_of_range_raises():
    a = _valid_action()
    a[-1] = GRIPPER_MAX + 1
    with pytest.raises(ValueError):
        validate_action(a)


def test_valid_observation_passes():
    validate_observation(_valid_observation())


def test_observation_missing_camera_raises():
    obs = _valid_observation()
    del obs[image_key(CAMERA_KEYS[0])]
    with pytest.raises(KeyError):
        validate_observation(obs)


def test_observation_wrong_image_shape_raises():
    obs = _valid_observation()
    obs[image_key(CAMERA_KEYS[0])] = np.zeros((1, 1, 3), dtype=np.uint8)
    with pytest.raises(ValueError):
        validate_observation(obs)


def test_observation_non_numeric_state_raises():
    obs = _valid_observation()
    obs[OBS_STATE_KEY] = np.array(["a", "b", "c", "d", "e", "f"])
    with pytest.raises(ValueError):
        validate_observation(obs)


def test_image_key_unknown_camera_raises():
    with pytest.raises(ValueError):
        image_key("nonexistent_camera")
