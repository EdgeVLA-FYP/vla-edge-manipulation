"""Fake backend: correctly-shaped random data, no hardware or simulator.

Lets tests, the recorder, and the eval harness run with zero external
dependencies. Write real backends (sim, real) against the same conformance
tests this one passes — see tests/test_backends.py.
"""

from __future__ import annotations

import numpy as np

from vla_edge_manipulation.backends.base import RobotBackend
from vla_edge_manipulation.schema import (
    CAMERA_KEYS,
    IMAGE_SHAPE,
    OBS_STATE_KEY,
    STATE_DIM,
    image_key,
    validate_action,
)


class MockBackend(RobotBackend):
    def __init__(self, seed: int | None = None):
        self._rng = np.random.default_rng(seed)
        self._connected = False

    def connect(self) -> None:
        self._connected = True

    def get_observation(self) -> dict[str, np.ndarray]:
        obs: dict[str, np.ndarray] = {
            OBS_STATE_KEY: self._rng.uniform(-1, 1, size=STATE_DIM).astype(np.float32)
        }
        for camera in CAMERA_KEYS:
            obs[image_key(camera)] = self._rng.integers(0, 256, size=IMAGE_SHAPE, dtype=np.uint8)
        return obs

    def send_action(self, action: np.ndarray) -> None:
        validate_action(action)

    def reset_to_home(self) -> None:
        pass

    def disconnect(self) -> None:
        self._connected = False
