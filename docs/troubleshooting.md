# Troubleshooting

The failure modes I actually hit while building and running this, with fixes.

## Planning

**`planning failed (...): goal unreachable (relaxed)`**
The goal cannot be reached from the current state — usually a missing edge (remember
both directions are needed if you edit PDDL by hand; the problem builder emits both
automatically) or a goal fixture that is not `dirty`. During a mission this can be
*correct* behaviour: after repeated navigation failures, enough edges get blocked
that the goal is provably unreachable, and the mission aborts on purpose.

**`"(:functions" is not supported by the internal planner`**
You pointed the internal backend at the temporal domain. That is a guardrail, not a
bug — use `planner_backend:=popf` (or `plansys2`) for temporal mode.

**`popf binary not found on this machine`**
Install it (`sudo apt install ros-jazzy-popf`) or let `auto` fall back to the
internal search — which handles the STRIPS domain only.

**`planner/get_plan service not available — is PlanSys2 running?`**
Start it first: `ros2 launch washbot_bringup plansys2.launch.py`. If it is running,
check for a ROS_DOMAIN_ID mismatch between terminals.

**Planner returns an empty or trivial plan.**
The goals are already satisfied in the initial state. The engine short-circuits this
case (`all goals already satisfied`); if you see it unexpectedly, check which
fixtures are `dirty` in your world file.

## Execution

**`NavigateToPose action server not available`**
Neither the fake server nor Nav2 is up, or they are in different ROS domains. For
simulation, `mission_sim.launch.py` starts both halves in the right order; if you run
the nodes by hand, start the fake server first (the handler polls for up to 10 s).

**Mission hangs, then `step exceeded 180s — cancelling`.**
The watchdog works; something below it does not. Usual suspects: Nav2 stuck without
aborting (check the controller/costmap logs), or a goal pose outside the map. The
timeout is the `action_timeout` parameter.

**Robot drives *through* obstacles in simulation.**
Expected: the fake server is a point moving in a straight line; it validates the
mission logic, not geometry. Keep world-file edges straight-line-traversable, or run
against real Nav2 (`mission_tb3.launch.py`) for true collision-aware motion.

**Every mission is a straight success and I wanted drama.**
Fault injection is opt-in: `fail_edges:='[hall->commode]'` for a detour,
`fail_locations:='[commode]'` for an abort, `fail_times:=1` for a transient fault.

## Launch & parameters

**`Expected a non-empty sequence ... Got inconsistent input for "fail_edges"`**
An empty YAML list (`[]`) cannot be type-inferred as a string array by launch. Use
`['']` (the nodes filter empty strings) — the launch files' defaults already do.

**`the "world_file" parameter is required`**
`mission_controller` was run without a world file. Either launch through
`mission_sim.launch.py` / `mission_tb3.launch.py` (they inject the installed default)
or pass `-p world_file:=<path>` explicitly.

**Changed Python code but behaviour did not change.**
Build with `--symlink-install`, or rebuild after every edit. Also make sure you
sourced `install/setup.bash` in *this* terminal.

## TurtleBot3 / Nav2

**The robot never moves in Gazebo: Nav2 is "active", recoveries pile up, `/odom` stays at zero.**
This is a `Twist` vs `TwistStamped` mismatch, and it cost me an afternoon. The Jazzy
TurtleBot3 gz bridge subscribes to `/cmd_vel` as **`geometry_msgs/TwistStamped`**
(their bridge YAML says so explicitly), so plain `Twist` commands are silently
dropped — Nav2 believes it is driving, the controller reports "failed to make
progress", and the behavior server's recovery spins do nothing either. Diagnose with
`ros2 topic info /cmd_vel -v` (two message types on one topic is the tell). The fix
is threefold and already in this repo: `enable_stamped_cmd_vel: true` for
`controller_server` and `behavior_server` in `nav2_params.yaml`, and
`stamped_cmd_vel: true` on the mission controller (set by `mission_tb3.launch.py`)
so the cleaning spin uses the same convention. If you run against an older stack
whose base wants plain `Twist`, flip those three switches back.

**The robot spins in circles endlessly with nothing commanding it.**
The gz DiffDrive plugin latches the last velocity command. If anything (a teleop
test, a killed node mid-spin) leaves a nonzero command as the last message, the
robot keeps executing it forever. Publish a single zero command to stop it:
`ros2 topic pub -t 3 /cmd_vel geometry_msgs/msg/TwistStamped "{}"`.

**AMCL never localizes.**
Seed it: `ros2 run washbot_control initial_pose_publisher --ros-args -p x:=-2.05 -p
y:=-0.55 -p yaw:=0.0` (the dock pose on the shipped map), or use RViz's 2D Pose
Estimate. `mission_tb3.launch.py` does this automatically at t+6 s.

**Nav2 lifecycle nodes fail to configure on Humble.**
`nav2_params.yaml` targets Jazzy. Nav2 occasionally renames plugin identifiers
between releases (e.g. behavior plugin namespaces); diff against the
`nav2_bringup` defaults of your distro and adjust the plugin lines.

**`use_sim_time` confusion.**
Gazebo: everything `use_sim_time:=true` (the TB3 launch files default to it). Real
robot: `false` everywhere. Mixed clocks manifest as TF extrapolation errors.

## Reports & data

**Where did my mission report go?**
`~/.washbot/reports/mission_<stamp>.json` (configurable via `report_dir`). It
contains the full event history — for a failed mission, `events` and `abort_reason`
are the fastest route to what happened.

**Benchmark numbers differ from the committed ones.**
They will — different CPU, thermals, Python version. The committed CSV documents its
environment in [benchmarks.md](benchmarks.md); regenerate on your machine for
apples-to-apples comparisons. Seeds are fixed, so the *worlds* are identical.
