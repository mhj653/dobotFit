from __future__ import annotations

from abc import ABC, abstractmethod

from core.models import Result


class IGripper(ABC):
    @abstractmethod
    def open(self) -> Result: ...

    @abstractmethod
    def close(self) -> Result: ...

    @abstractmethod
    def test(self) -> Result: ...

