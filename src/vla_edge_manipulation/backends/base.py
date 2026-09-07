"""The contract every robot implementation (sim, real, mock) must satisfy.

Nothing outside backends/ may branch on which implementation is running —
that is the whole point of this abstraction. See the new-backend skill
before adding an implementation.
"""

from __future__ import annotations

from abc import ABC, abstractmethod


class RobotBackend(ABC):
    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def get_observation(self) -> dict:
        """Returns a dict matching schema.validate_observation()."""

    @abstractmethod
    def send_action(self, action) -> None:
        """Accepts an array matching schema.validate_action()."""

    @abstractmethod
    def reset_to_home(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...
