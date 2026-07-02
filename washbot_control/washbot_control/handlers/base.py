"""Common interface for plan-step handlers."""

from __future__ import annotations

import abc
from typing import Callable

from washbot_planning.plan import PlanStep

# done_cb(success, message) — must be called exactly once per start().
DoneCallback = Callable[[bool, str], None]


class ActionHandler(abc.ABC):
    """Executes one kind of PDDL action on the robot.

    Handlers are asynchronous: :meth:`start` returns immediately and the
    outcome is delivered through the callback (from the node's executor
    thread). The mission controller guarantees a single in-flight step at a
    time and may call :meth:`cancel` on timeout or shutdown.
    """

    def __init__(self, node, world):
        self.node = node
        self.world = world

    @abc.abstractmethod
    def start(self, step: PlanStep, done_cb: DoneCallback) -> None:
        """Begin executing ``step``; report the outcome via ``done_cb``."""

    def cancel(self) -> None:
        """Best-effort cancellation of the in-flight step (optional)."""
