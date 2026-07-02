## What

<!-- One or two sentences: what does this change do? -->

## Why

<!-- The problem or motivation. Link the issue if there is one. -->

## How it was verified

- [ ] `python3 -m pytest washbot_planning/test washbot_control/test`
- [ ] `python3 -m flake8 washbot_planning washbot_control washbot_bringup`
- [ ] (behaviour changes) ran a simulated mission and checked the outcome:
      `ros2 launch washbot_bringup mission_sim.launch.py fail_edges:='[hall->commode]'`

<!-- Paste relevant log lines for behaviour changes. -->

## Notes for the reviewer

<!-- Design decisions, trade-offs, anything you want a second opinion on. -->
