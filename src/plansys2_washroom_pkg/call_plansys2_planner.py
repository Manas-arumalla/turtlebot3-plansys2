#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from plansys2_msgs.srv import GetPlan
import os, sys
from pprint import pprint

class PlannerClient(Node):
    def __init__(self):
        super().__init__('plansys2_planner_client')
        self.cli = self.create_client(GetPlan, 'planner/get_plan')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('planner/get_plan service not available')
            raise SystemExit(1)

    def call_planner(self, domain_text, problem_text, timeout=10.0):
        req = GetPlan.Request()
        req.domain = domain_text
        req.problem = problem_text
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            self.get_logger().error('Planner service call timed out')
            return None
        return fut.result()

def read_file_text(path):
    with open(path, 'r') as f:
        return f.read()

def main(argv=None):
    rclpy.init(args=argv)
    node = PlannerClient()
    script_dir = os.path.abspath(os.path.dirname(__file__))
    pkg_root = script_dir

    domain_path = os.path.join(pkg_root, 'pddl', 'washroom_domain.pddl')
    problem_path = os.path.join(pkg_root, 'pddl', 'washroom_problem.pddl')

    if not os.path.exists(domain_path) or not os.path.exists(problem_path):
        print('PDDL files not found at expected paths:')
        print('  domain:', domain_path)
        print('  problem:', problem_path)
        return 2

    domain_text = read_file_text(domain_path)
    problem_text = read_file_text(problem_path)

    resp = node.call_planner(domain_text, problem_text)
    if resp is None:
        print('No response from planner')
        return 1

    print('Planner success:', resp.success)
    print('Error info:', resp.error_info)
    print('Full plan object (repr):')
    pprint(resp.plan)

    print('\n--- Decoded plan actions (index, time, action, duration) ---')
    try:
        items = getattr(resp.plan, 'items', [])
        for i, it in enumerate(items, start=1):
            # it.action is a string like "(navigate r1 start basin)"
            action_str = it.action.strip()
            print(f'{i:2d}: time={it.time} action={action_str} duration={it.duration}')
    except Exception as e:
        print('Could not decode plan fields cleanly:', e)

    node.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == '__main__':
    sys.exit(main())
