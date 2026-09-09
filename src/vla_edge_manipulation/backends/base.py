"""The contract every robot implementation (sim, real, mock) must satisfy.

Nothing outside backends/ may branch on which implementation is running —
that is the whole point of this abstraction.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np


class RobotBackend(ABC):
    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def get_observation(self) -> dict[str, np.ndarray]:
        """Returns a dict matching schema.validate_observation()."""

    @abstractmethod
    def send_action(self, action: np.ndarray) -> None:
        """Accepts an array matching schema.validate_action()."""

    @abstractmethod
    def reset_to_home(self) -> None:
        """Call between episodes. Resets the arm."""

    @abstractmethod
    def disconnect(self) -> None: ...
