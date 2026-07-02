"""ROS 2 node that requests a plan and prints it — the quickest smoke test.

Successor of the original ``call_plansys2_planner.py`` script: it can still
call the PlanSys2 ``planner/get_plan`` service, but it can also plan through
any local backend, so the same tool works with or without PlanSys2 running:

    ros2 run washbot_planning planner_client
    ros2 run washbot_planning planner_client --ros-args -p backend:=plansys2
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node

from washbot_planning import backends
from washbot_planning.data import find_pddl
from washbot_planning.pddl_core import parse_domain, parse_problem, validate_plan
from washbot_planning.plan import Plan


class PlannerClient(Node):

    def __init__(self):
        super().__init__('washbot_planner_client')
        self.declare_parameter('domain_file', 'domain_strips.pddl')
        self.declare_parameter('problem_file', 'washroom_small.pddl')
        self.declare_parameter('backend', 'auto')
        self.declare_parameter('timeout', 30.0)
        self.declare_parameter('validate', True)

    def run(self) -> int:
        domain_file = self.get_parameter('domain_file').value
        problem_file = self.get_parameter('problem_file').value
        backend_name = self.get_parameter('backend').value
        timeout = float(self.get_parameter('timeout').value)

        with open(find_pddl(domain_file), 'r', encoding='utf-8') as handle:
            domain_text = handle.read()
        with open(find_pddl(problem_file), 'r', encoding='utf-8') as handle:
            problem_text = handle.read()

        if backend_name == 'plansys2':
            plan, error = self._plan_via_plansys2(domain_text, problem_text, timeout)
            source = 'plansys2'
        else:
            backend = backends.resolve(backend_name, timeout_seconds=timeout)
            result = backend.solve(domain_text, problem_text)
            plan, error, source = result.plan, result.error, result.backend

        if plan is None:
            self.get_logger().error(f'planning failed ({source}): {error}')
            return 1

        self.get_logger().info(f'plan from {source}: {len(plan)} steps, '
                               f'makespan {plan.makespan:.1f}')
        for step in plan:
            print(f'{step.index + 1:3d}  t={step.start_time:7.3f}  '
                  f'{step.signature}  [{step.duration:.3f}]')

        if bool(self.get_parameter('validate').value):
            try:
                report = validate_plan(parse_domain(domain_text),
                                       parse_problem(problem_text), plan.as_tuples())
                self.get_logger().info(f'validation: {report.summary()}')
                if not report.valid:
                    return 1
            except Exception as error:  # noqa: BLE001 - temporal domains, etc.
                self.get_logger().info(f'validation skipped: {error}')
        return 0

    def _plan_via_plansys2(self, domain_text: str, problem_text: str,
                           timeout: float):
        try:
            from plansys2_msgs.srv import GetPlan
        except ImportError:
            return None, 'plansys2_msgs is not installed on this machine'

        client = self.create_client(GetPlan, 'planner/get_plan')
        if not client.wait_for_service(timeout_sec=timeout):
            return None, 'planner/get_plan service not available — is PlanSys2 running?'

        request = GetPlan.Request()
        request.domain = domain_text
        request.problem = problem_text
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            return None, f'service call timed out after {timeout}s'
        response = future.result()
        if not response.success:
            return None, response.error_info
        return Plan.from_plansys2_items(response.plan.items), ''


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = PlannerClient()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main(sys.argv))
