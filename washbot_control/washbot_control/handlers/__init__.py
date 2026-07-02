"""Action handlers: map PDDL action names to robot behaviours.

Adding a new capability to the robot is a three-step recipe:

1. model the action in the PDDL domain,
2. write a handler class implementing :class:`ActionHandler`,
3. register it in :func:`build_handlers`.

The mission controller looks handlers up by the action name that appears in
the plan, so nothing else needs to change.
"""

from washbot_control.handlers.base import ActionHandler  # noqa: F401
from washbot_control.handlers.clean import CleanHandler
from washbot_control.handlers.navigate import NavigateHandler
from washbot_control.handlers.recharge import RechargeHandler


def build_handlers(node, world, stamped_cmd_vel: bool = False) -> dict:
    """Instantiate the default handler set for a mission controller node."""
    return {
        'move': NavigateHandler(node, world),
        'clean': CleanHandler(node, world, stamped_cmd_vel=stamped_cmd_vel),
        'deep_clean': CleanHandler(node, world, deep=True,
                                   stamped_cmd_vel=stamped_cmd_vel),
        'recharge': RechargeHandler(node, world),
    }
