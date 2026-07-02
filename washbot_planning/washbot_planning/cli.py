"""``washbot_plan`` — command-line front end for the planning layer.

Everything here runs without ROS, which is how I develop and benchmark the
planning side on machines that have no ROS install:

    washbot_plan solve --domain domain_strips.pddl --problem washroom_small.pddl
    washbot_plan gen-problem --rooms 8 --out /tmp/big.pddl
    washbot_plan benchmark --rooms 2,4,8 --out benchmarks/results/results.csv
    washbot_plan render-world --map-yaml <map.yaml> --world <world.yaml> --out world.png
"""

from __future__ import annotations

import argparse
import os
import sys

from washbot_planning import backends
from washbot_planning.data import find_pddl
from washbot_planning.generator import generate_world, goal_all_fixtures
from washbot_planning.pddl_core import parse_domain, parse_problem, validate_plan
from washbot_planning.plan import Plan
from washbot_planning.problem_builder import build_strips_problem, build_temporal_problem


def _read_pddl_arg(value: str) -> str:
    """Accept either a filesystem path or a packaged PDDL file name."""
    path = value if os.path.isfile(value) else find_pddl(value)
    with open(path, 'r', encoding='utf-8') as handle:
        return handle.read()


def _cmd_solve(args: argparse.Namespace) -> int:
    domain_text = _read_pddl_arg(args.domain)
    problem_text = _read_pddl_arg(args.problem)
    backend = backends.resolve(args.backend, timeout_seconds=args.timeout)
    result = backend.solve(domain_text, problem_text)

    if not result.success:
        print(f'[{result.backend}] planning failed: {result.error}', file=sys.stderr)
        return 1

    print(f'[{result.backend}] plan with {len(result.plan)} steps '
          f'(makespan {result.plan.makespan:.1f}):')
    print(result.plan.to_text())
    for key, value in sorted(result.stats.items()):
        print(f'  {key}: {value:.4f}' if isinstance(value, float) else f'  {key}: {value}')

    if args.validate:
        report = validate_plan(parse_domain(domain_text), parse_problem(problem_text),
                               result.plan.as_tuples())
        print(f'validation: {report.summary()}')
        if not report.valid:
            return 1
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as handle:
            handle.write(result.plan.to_text() + '\n')
        print(f'plan written to {args.out}')
    return 0


def _cmd_validate(args: argparse.Namespace) -> int:
    domain = parse_domain(_read_pddl_arg(args.domain))
    problem = parse_problem(_read_pddl_arg(args.problem))
    with open(args.plan, 'r', encoding='utf-8') as handle:
        plan = Plan.from_popf_output(handle.read(), source='file')
    if not plan.steps:
        print('no plan steps could be parsed from the file', file=sys.stderr)
        return 1
    report = validate_plan(domain, problem, plan.as_tuples())
    print(report.summary())
    return 0 if report.valid else 1


def _cmd_gen_problem(args: argparse.Namespace) -> int:
    snapshot = generate_world(rooms=args.rooms, fixtures_per_room=args.fixtures,
                              seed=args.seed)
    goals = goal_all_fixtures(snapshot)
    if args.temporal:
        text = build_temporal_problem(snapshot, goals,
                                      problem_name=f'generated_{args.rooms}_rooms')
    else:
        text = build_strips_problem(snapshot, goals,
                                    problem_name=f'generated_{args.rooms}_rooms')
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as handle:
            handle.write(text)
        print(f'{len(snapshot.locations)} locations, {len(goals)} goals -> {args.out}')
    else:
        print(text)
    return 0


def _cmd_benchmark(args: argparse.Namespace) -> int:
    from washbot_planning.benchmark.runner import run_benchmark, write_csv
    sizes = [int(s) for s in args.rooms.split(',')]
    backend_names = args.backends.split(',') if args.backends \
        else backends.available_backends()
    rows = run_benchmark(sizes=sizes, backend_names=backend_names,
                         fixtures_per_room=args.fixtures, repeats=args.repeats,
                         timeout_seconds=args.timeout)
    write_csv(rows, args.out)
    print(f'{len(rows)} runs written to {args.out}')
    return 0


def _cmd_plot_benchmark(args: argparse.Namespace) -> int:
    from washbot_planning.benchmark.plots import plot_all
    written = plot_all(args.csv, args.out_dir)
    for path in written:
        print(f'wrote {path}')
    return 0


def _cmd_render_world(args: argparse.Namespace) -> int:
    from washbot_planning.viz import render_world
    path = render_world(args.map_yaml, args.world, args.out,
                        title=args.title, path_csv=args.path_csv)
    print(f'wrote {path}')
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='washbot_plan',
        description='Plan, validate, generate and benchmark WashBot PDDL tasks.')
    sub = parser.add_subparsers(dest='command', required=True)

    solve = sub.add_parser('solve', help='produce a plan for a domain/problem pair')
    solve.add_argument('--domain', required=True)
    solve.add_argument('--problem', required=True)
    solve.add_argument('--backend', default='auto',
                       help='auto | internal-gbfs | internal-astar | popf')
    solve.add_argument('--timeout', type=float, default=60.0)
    solve.add_argument('--validate', action='store_true',
                       help='replay the plan through the validator afterwards')
    solve.add_argument('--out', help='write the plan to this file')
    solve.set_defaults(func=_cmd_solve)

    validate = sub.add_parser('validate', help='validate a plan file against PDDL')
    validate.add_argument('--domain', required=True)
    validate.add_argument('--problem', required=True)
    validate.add_argument('--plan', required=True, help='plan file in POPF format')
    validate.set_defaults(func=_cmd_validate)

    gen = sub.add_parser('gen-problem', help='generate a synthetic facility problem')
    gen.add_argument('--rooms', type=int, required=True)
    gen.add_argument('--fixtures', type=int, default=2)
    gen.add_argument('--seed', type=int, default=7)
    gen.add_argument('--temporal', action='store_true')
    gen.add_argument('--out')
    gen.set_defaults(func=_cmd_gen_problem)

    bench = sub.add_parser('benchmark', help='run the planning benchmark suite')
    bench.add_argument('--rooms', default='2,4,8,16,32',
                       help='comma-separated facility sizes')
    bench.add_argument('--fixtures', type=int, default=2)
    bench.add_argument('--backends', default='',
                       help='comma-separated backend names (default: all available)')
    bench.add_argument('--repeats', type=int, default=3)
    bench.add_argument('--timeout', type=float, default=60.0)
    bench.add_argument('--out', default='benchmarks/results/results.csv')
    bench.set_defaults(func=_cmd_benchmark)

    plot = sub.add_parser('plot-benchmark', help='render benchmark plots from a CSV')
    plot.add_argument('--csv', required=True)
    plot.add_argument('--out-dir', default='docs/media')
    plot.set_defaults(func=_cmd_plot_benchmark)

    render = sub.add_parser('render-world', help='render the map + waypoint graph')
    render.add_argument('--map-yaml', required=True)
    render.add_argument('--world', required=True)
    render.add_argument('--out', required=True)
    render.add_argument('--title', default='WashBot world model')
    render.add_argument('--path-csv', default='',
                        help='overlay a trajectory recorded by pose_recorder')
    render.set_defaults(func=_cmd_render_world)

    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
