"""Mission controller node: the executive that turns goals into behaviour.

Wires the pieces together:

  world.yaml ──> WorldModel ──> MissionEngine (plan / replan decisions)
                                    │
                              PlanStep by step
                                    ▼
                    ActionHandlers (Nav2, cleaning, recharge)

The node stays deliberately thin — every decision lives in the ROS-free
:class:`~washbot_control.engine.MissionEngine`, and every robot skill lives
in a handler. What remains here is dispatch, timeouts, status/marker
publishing, and the end-of-mission report.

Planning goes through a local backend (bundled search or POPF) or through
the PlanSys2 planner service (``planner_backend: plansys2``) when a PlanSys2
stack is running.
"""

from __future__ import annotations

import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from washbot_control.engine import (
    MissionEngine,
    MissionState,
    make_local_plan_fn,
    parse_goals_param,
)
from washbot_control.handlers import build_handlers
from washbot_control.markers import build_markers
from washbot_control.world_model import WorldModel
from washbot_planning.backends import PlanningResult
from washbot_planning.data import read_pddl
from washbot_planning.plan import Plan

from visualization_msgs.msg import MarkerArray


class MissionController(Node):

    def __init__(self):
        super().__init__('mission_controller')

        self.declare_parameter('world_file', '')
        self.declare_parameter('goals', ['all'])
        self.declare_parameter('domain_mode', 'strips')  # strips | temporal
        self.declare_parameter('planner_backend', 'auto')
        self.declare_parameter('planning_timeout', 30.0)
        self.declare_parameter('max_replans', 5)
        self.declare_parameter('max_step_retries', 1)
        self.declare_parameter('action_timeout', 180.0)
        self.declare_parameter('autostart', True)
        self.declare_parameter('report_dir', '~/.washbot/reports')
        # True for stacks whose cmd_vel is TwistStamped (Jazzy TurtleBot3).
        self.declare_parameter('stamped_cmd_vel', False)

        world_file = self.get_parameter('world_file').value
        if not world_file:
            raise RuntimeError('the "world_file" parameter is required')
        self.world = WorldModel.from_yaml(os.path.expanduser(world_file))

        domain_mode = self.get_parameter('domain_mode').value
        domain_file = ('domain_temporal.pddl' if domain_mode == 'temporal'
                       else 'domain_strips.pddl')
        goals = parse_goals_param(self.get_parameter('goals').value, self.world)

        self.engine = MissionEngine(
            world=self.world,
            goals=goals,
            domain_text=read_pddl(domain_file),
            plan_fn=self._make_plan_fn(),
            domain_mode=domain_mode,
            max_replans=int(self.get_parameter('max_replans').value),
            max_step_retries=int(self.get_parameter('max_step_retries').value),
        )
        self.handlers = build_handlers(
            self, self.world,
            stamped_cmd_vel=bool(self.get_parameter('stamped_cmd_vel').value))

        self._status_pub = self.create_publisher(String, '/washbot/mission_status', 10)
        self._battery_pub = self.create_publisher(Float32, '/washbot/battery', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/washbot/world_markers', 10)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self._step_started_at = 0.0
        self._step_timings = []
        self._mission_started_at = 0.0
        self._watchdog = None
        self._active_handler = None
        self._report_written = False

        self.create_subscription(String, '/washbot/mission_command',
                                 self._on_command, 10)
        if bool(self.get_parameter('autostart').value):
            # Give discovery a moment before the first Nav2 goal goes out.
            self._start_timer = self.create_timer(2.0, self._deferred_start)
        else:
            self.get_logger().info(
                'autostart disabled — waiting for "start" on /washbot/mission_command')

    # ------------------------------------------------------------ lifecycle

    def _deferred_start(self) -> None:
        self._start_timer.cancel()
        self.destroy_timer(self._start_timer)
        self.start_mission()

    def _on_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command == 'start' and self.engine.state is MissionState.IDLE:
            self.start_mission()
        elif command == 'abort':
            self._cancel_active_step()
            self.engine.state = MissionState.ABORTED
            self.engine.abort_reason = 'aborted by operator'
            self._finish_mission()

    def start_mission(self) -> None:
        goals = ', '.join(self.engine.goals)
        self.get_logger().info(
            f'mission start: clean [{goals}] from "{self.world.robot_at}" '
            f'(mode={self.engine.domain_mode}, '
            f'backend={self.get_parameter("planner_backend").value})')
        self._mission_started_at = time.monotonic()
        if self.engine.start():
            self._log_plan()
            self._dispatch_current_step()
        else:
            self._finish_mission()

    # ------------------------------------------------------------- dispatch

    def _dispatch_current_step(self) -> None:
        step = self.engine.current_step()
        if step is None:
            self._finish_mission()
            return

        handler = self.handlers.get(step.name)
        if handler is None:
            self.engine.report_step_result(
                False, f'no handler registered for action "{step.name}"')
            self._after_step_result()
            return

        total = len(self.engine.plan.steps)
        self.get_logger().info(
            f'step {self.engine.step_index + 1}/{total}: {step.signature}')
        self._active_handler = handler
        self._step_started_at = time.monotonic()
        self._arm_watchdog()
        handler.start(step, self._on_step_done)

    def _on_step_done(self, success: bool, message: str) -> None:
        self._disarm_watchdog()
        self._active_handler = None
        step = self.engine.current_step()
        if step is not None:
            self._step_timings.append({
                'step': step.signature,
                'success': success,
                'seconds': round(time.monotonic() - self._step_started_at, 2),
            })
        self.engine.report_step_result(success, message)
        self._after_step_result()

    def _after_step_result(self) -> None:
        if self.engine.state is MissionState.EXECUTING:
            if self.engine.events and self.engine.events[-1].kind == 'replan':
                self._log_plan()
            self._dispatch_current_step()
        else:
            self._finish_mission()

    # ------------------------------------------------------------- watchdog

    def _arm_watchdog(self) -> None:
        timeout = float(self.get_parameter('action_timeout').value)
        self._watchdog = self.create_timer(timeout, self._on_step_timeout)

    def _disarm_watchdog(self) -> None:
        if self._watchdog is not None:
            self._watchdog.cancel()
            self.destroy_timer(self._watchdog)
            self._watchdog = None

    def _on_step_timeout(self) -> None:
        self._disarm_watchdog()
        timeout = float(self.get_parameter('action_timeout').value)
        self.get_logger().warning(f'step exceeded {timeout:.0f}s — cancelling')
        self._cancel_active_step()
        self.engine.report_step_result(False, f'timed out after {timeout:.0f}s')
        self._after_step_result()

    def _cancel_active_step(self) -> None:
        handler, self._active_handler = self._active_handler, None
        if handler is not None:
            handler.cancel()

    # ------------------------------------------------------------- planning

    def _make_plan_fn(self):
        backend_name = self.get_parameter('planner_backend').value
        timeout = float(self.get_parameter('planning_timeout').value)
        if backend_name == 'plansys2':
            return self._plansys2_plan_fn(timeout)
        return make_local_plan_fn(backend_name, timeout_seconds=timeout)

    def _plansys2_plan_fn(self, timeout: float):
        """Plan through the PlanSys2 ``planner/get_plan`` service."""
        def plan_fn(domain_text: str, problem_text: str) -> PlanningResult:
            try:
                from plansys2_msgs.srv import GetPlan
            except ImportError:
                return PlanningResult(
                    success=False, backend='plansys2',
                    error='plansys2_msgs is not installed on this machine')

            helper = rclpy.create_node('washbot_plansys2_planner_client')
            try:
                client = helper.create_client(GetPlan, 'planner/get_plan')
                if not client.wait_for_service(timeout_sec=timeout):
                    return PlanningResult(
                        success=False, backend='plansys2',
                        error='planner/get_plan service not available — '
                              'is PlanSys2 running?')
                request = GetPlan.Request()
                request.domain = domain_text
                request.problem = problem_text
                future = client.call_async(request)
                rclpy.spin_until_future_complete(helper, future, timeout_sec=timeout)
                if not future.done():
                    return PlanningResult(success=False, backend='plansys2',
                                          error=f'service call timed out ({timeout}s)')
                response = future.result()
                if not response.success:
                    return PlanningResult(success=False, backend='plansys2',
                                          error=response.error_info)
                return PlanningResult(
                    success=True, backend='plansys2',
                    plan=Plan.from_plansys2_items(response.plan.items))
            finally:
                helper.destroy_node()

        return plan_fn

    # ------------------------------------------------------ status & report

    def _log_plan(self) -> None:
        if self.engine.plan is None:
            return
        self.get_logger().info(
            f'plan ({self.engine.plan.source}, {len(self.engine.plan)} steps):')
        for step in self.engine.plan.steps:
            self.get_logger().info(f'   {step.index + 1:2d}. {step.signature}')

    def _publish_status(self) -> None:
        summary = self.engine.summary()
        summary.pop('events')  # keep the topic light; events go in the report
        summary['step_index'] = self.engine.step_index
        self._status_pub.publish(String(data=json.dumps(summary)))
        self._battery_pub.publish(Float32(data=float(self.world.battery_pct)))
        self._marker_pub.publish(build_markers(
            self.world, self.get_clock().now().to_msg(),
            plan=self.engine.plan, next_step_index=self.engine.step_index))

    def _finish_mission(self) -> None:
        if self._report_written:
            return
        self._report_written = True

        state = self.engine.state
        elapsed = time.monotonic() - self._mission_started_at
        if state is MissionState.SUCCEEDED:
            self.get_logger().info(
                f'mission SUCCEEDED in {elapsed:.1f}s — cleaned: '
                f'{", ".join(sorted(self.world.cleaned))} '
                f'(replans: {self.engine.replans}, '
                f'battery: {self.world.battery_pct:.0f}%)')
        else:
            self.get_logger().error(
                f'mission {state.value.upper()} after {elapsed:.1f}s: '
                f'{self.engine.abort_reason}')
        self._write_report(elapsed)
        self._publish_status()

    def _write_report(self, elapsed_seconds: float) -> None:
        report_dir = os.path.expanduser(self.get_parameter('report_dir').value)
        if not report_dir:
            return
        try:
            os.makedirs(report_dir, exist_ok=True)
            report = self.engine.summary()
            report['elapsed_seconds'] = round(elapsed_seconds, 2)
            report['step_timings'] = self._step_timings
            path = os.path.join(report_dir,
                                time.strftime('mission_%Y%m%d_%H%M%S.json'))
            with open(path, 'w', encoding='utf-8') as handle:
                json.dump(report, handle, indent=2)
            self.get_logger().info(f'mission report written to {path}')
        except OSError as error:
            self.get_logger().warning(f'could not write mission report: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
