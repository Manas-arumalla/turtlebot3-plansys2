# Contributing

Thanks for taking an interest in the project. Issues and pull requests are welcome —
this page keeps the friction low for both of us.

## Ground rules

- **Open an issue first** for anything beyond a small fix, so we can agree on the
  approach before you invest time.
- **Keep the layering intact.** Decision logic (planning, the mission engine, the
  world model) stays ROS-free and unit-tested; ROS nodes stay thin. If your change
  puts a decision inside a node callback, it probably belongs in `engine.py`.
- **PDDL changes come with validator coverage.** If you touch a domain, add or adjust
  a test that pins the intended semantics (see `test_validator.py` for the pattern).
- **New PDDL constructs** must either be supported by the internal parser or rejected
  by it with an error that names a backend that can handle them.

## Development setup

```bash
git clone https://github.com/Manas-arumalla/turtlebot3-plansys2.git
cd turtlebot3-plansys2
pip install -r requirements-dev.txt

# Fast loop (no ROS needed for the interesting parts):
python3 -m pytest washbot_planning/test washbot_control/test
python3 -m flake8 washbot_planning washbot_control washbot_bringup

# Full loop (ROS 2 workspace):
colcon build --symlink-install && source install/setup.bash
ros2 launch washbot_bringup mission_sim.launch.py fail_edges:='[hall->commode]'
```

## Pull request checklist

- [ ] `pytest` and `flake8` pass locally (CI runs both, plus a colcon build).
- [ ] Behaviour changes come with a test — engine-level for recovery logic,
      validator-level for PDDL semantics.
- [ ] If you changed missions' observable behaviour, run the recovery scenario above
      and paste the relevant log lines into the PR description.
- [ ] Docs updated where they would otherwise lie (`docs/architecture.md` for design
      changes, `docs/extending.md` for new extension points).
- [ ] Benchmark-affecting changes: regenerate `benchmarks/results/results.csv` and
      the plots, and say so in the PR (numbers are machine-specific; the CSV
      documents its environment).

## Style

- Python: what `.flake8` enforces (pyflakes/pycodestyle, 99-column lines). Match the
  surrounding code; comments explain *why*, not *what*.
- Commit messages: imperative subject line, body explains the why when it is not
  obvious.

## Reporting bugs

Use the bug template. The two artifacts that make a mission bug diagnosable are the
console log and the mission report JSON (`~/.washbot/reports/`) — attach both when
you can.
