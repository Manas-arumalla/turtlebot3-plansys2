"""Handler for the PDDL ``recharge`` action (temporal domain).

Simulated docking charge: a fixed-duration wait after which the world
model's battery is restored to 100%. The duration is deliberately shorter
than the planning-model's 30s so simulated missions stay brisk; the planner
only cares about the ordering constraint, not wall-clock fidelity.
"""

from __future__ import annotations

from typing import Optional

from washbot_control.handlers.base import ActionHandler, DoneCallback
from washbot_planning.plan import PlanStep


class RechargeHandler(ActionHandler):

    def __init__(self, node, world, duration_seconds: float = 8.0):
        super().__init__(node, world)
        self._duration = duration_seconds
        self._timer = None
        self._done_cb: Optional[DoneCallback] = None

    def start(self, step: PlanStep, done_cb: DoneCallback) -> None:
        self._done_cb = done_cb
        self.node.get_logger().info(
            f'recharging at "{step.args[-1]}" for {self._duration:.0f}s '
            f'(battery {self.world.battery_pct:.0f}%)')
        self._timer = self.node.create_timer(self._duration, self._finished)

    def cancel(self) -> None:
        self._drop_timer()
        self._done_cb = None

    def _finished(self) -> None:
        self._drop_timer()
        self.world.recharge()
        callback, self._done_cb = self._done_cb, None
        if callback is not None:
            callback(True, '')

    def _drop_timer(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self.node.destroy_timer(self._timer)
            self._timer = None
