"""Conformance tests run against every RobotBackend implementation.

Add a new backend to BACKEND_FACTORIES and it gets the same assertions as
every other one — that is the point of the shared interface.
"""

from collections.abc import Callable

import numpy as np
import pytest

from vla_edge_manipulation.backends.base import RobotBackend
from vla_edge_manipulation.backends.mock import MockBackend
from vla_edge_manipulation.schema import validate_observation

BACKEND_FACTORIES: dict[str, Callable[[], RobotBackend]] = {
    "mock": lambda: MockBackend(seed=0),
}

try:
    from vla_edge_manipulation.backends.sim import MuJoCoBackend

    BACKEND_FACTORIES["sim"] = MuJoCoBackend
except ImportError:
    pass  # `sim` extra not installed — mock-only conformance still runs


@pytest.fixture(params=BACKEND_FACTORIES.keys())
def backend(request):
    b = BACKEND_FACTORIES[request.param]()
    b.connect()
    yield b
    b.disconnect()


def test_observation_matches_schema(backend):
    validate_observation(backend.get_observation())


def test_send_valid_action_does_not_raise(backend):
    obs = backend.get_observation()
    action = np.zeros_like(obs["observation.state"])
    backend.send_action(action)


def test_reset_to_home_does_not_raise(backend):
    backend.reset_to_home()
