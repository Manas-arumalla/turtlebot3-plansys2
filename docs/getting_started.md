# Getting Started

Everything here was verified on ROS 2 **Jazzy** / Ubuntu 24.04 / Python 3.12. The
code sticks to stable `rclpy` and Nav2 interfaces, so Humble and Iron should work the
same way, but Jazzy is what I test on.

## 0. No ROS? Start here anyway

The planning layer runs on plain Python (only `pyyaml` needed; `matplotlib` for the
figures):

```bash
git clone https://github.com/Manas-arumalla/turtlebot3-plansys2.git
cd turtlebot3-plansys2/washbot_planning

# Solve and validate the washroom problem
python3 -m washbot_planning.cli solve \
    --domain domain_strips.pddl --problem washroom_small.pddl --validate

# Generate a 12-room facility and solve that
python3 -m washbot_planning.cli gen-problem --rooms 12 --out /tmp/big.pddl
python3 -m washbot_planning.cli solve --domain domain_strips.pddl \
    --problem /tmp/big.pddl --backend internal-gbfs

# Run the test suite
python3 -m pytest test
```

## 1. Workspace setup

```bash
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions  # if needed
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Manas-arumalla/turtlebot3-plansys2.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y   # nav2 msgs etc.
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` matters if you plan to hack on the Python code: edits take effect
without rebuilding.

## 2. Simulated missions (no robot, no Gazebo)

The fake Nav2 server exposes the real `NavigateToPose` interface, so these runs
exercise exactly the code paths a robot would.

```bash
# Nominal: plan and clean everything marked dirty in world.yaml
ros2 launch washbot_bringup mission_sim.launch.py

# Clean a subset
ros2 launch washbot_bringup mission_sim.launch.py goals:='[basin]'

# Recovery: the hall->commode corridor is blocked; watch the detour
ros2 launch washbot_bringup mission_sim.launch.py fail_edges:='[hall->commode]'

# Graceful failure: commode unreachable from everywhere; watch the clean abort
ros2 launch washbot_bringup mission_sim.launch.py fail_locations:='[commode]'

# Transient fault: first attempt fails, second succeeds
ros2 launch washbot_bringup mission_sim.launch.py \
    fail_edges:='[hall->basin]' fail_times:=1
```

What to expect (from a real recovery run —
[full log](examples/mission_log_recovery.txt)):

```
[mission_controller]: plan (internal-astar, 6 steps):
    ...
[mission_controller]: step 5/6: (move washbot hall commode)
[fake_nav2_server]:   injected failure for "hall->commode" (attempt 1, mode=abort)
[mission_controller]: plan (internal-astar, 4 steps):
    1. (move washbot hall basin)
    2. (move washbot basin shower)
    3. (move washbot shower commode)
    4. (clean washbot commode)
[mission_controller]: mission SUCCEEDED in 63.8s — cleaned: basin, commode (replans: 1, battery: 71%)
```

Useful extras:

```bash
# Live visualization (map + waypoint graph + planned route + robot pose)
ros2 launch washbot_bringup mission_sim.launch.py use_rviz:=true

# Watch mission state / battery from another terminal
ros2 topic echo /washbot/mission_status
ros2 topic echo /washbot/battery

# Manual mission control instead of autostart
ros2 launch washbot_bringup mission_sim.launch.py autostart:=false
ros2 topic pub -1 /washbot/mission_command std_msgs/String "{data: start}"
```

Every mission writes a JSON report to `~/.washbot/reports/` with the plan, every
event, and per-step timings.

### Recording the executed path

The trajectory figures in the README are produced like this:

```bash
ros2 run washbot_control pose_recorder --ros-args \
    -p topic:=/washbot/sim_pose -p output:=/tmp/mission_path.csv   # terminal 1
ros2 launch washbot_bringup mission_sim.launch.py \
    fail_edges:='[hall->commode]'                                   # terminal 2
# afterwards:
python3 -m washbot_planning.cli render-world \
    --map-yaml $(ros2 pkg prefix washbot_bringup)/share/washbot_bringup/maps/washroom_world.yaml \
    --world $(ros2 pkg prefix washbot_control)/share/washbot_control/config/world.yaml \
    --path-csv /tmp/mission_path.csv --out /tmp/mission.png
```

## 3. TurtleBot3 in Gazebo (verified end to end)

Requires Nav2 (`ros-jazzy-navigation2 ros-jazzy-nav2-bringup`) and the TurtleBot3
simulation:

```bash
sudo apt install ros-jazzy-turtlebot3-gazebo
```

or, without root, build it from source next to any workspace:

```bash
mkdir -p ~/tb3_gazebo_ws/src && cd ~/tb3_gazebo_ws/src
git clone -b jazzy --depth 1 https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/tb3_gazebo_ws && colcon build --packages-select turtlebot3_gazebo --symlink-install
source install/setup.bash
```

Then two terminals:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py          # terminal 1
ros2 launch washbot_bringup mission_tb3.launch.py                 # terminal 2
```

`mission_tb3.launch.py` starts the Nav2 stack on the washroom map
(`washbot_bringup/config/nav2_params.yaml`, tuned for a Burger on Jazzy —
including `enable_stamped_cmd_vel`, see [troubleshooting](troubleshooting.md)),
seeds AMCL with the dock pose at t+8 s, and starts the mission controller at
t+20 s, once localization has settled. A representative run on my machine: the
7-step A* plan executed in **111 s of real physics** — drive to the basin wing,
clean all three fixtures around the loop, zero replans. Watch the robot spin
slowly in place while a fixture is being "cleaned".

On a physical robot, set `use_sim_time:=false` and localize before the mission
starts (drive around briefly or set the pose in RViz).

The world file's waypoints match the classic `turtlebot3_world` map shipped in
`washbot_bringup/maps/`. For your own space: SLAM a map, drop it in `maps/`, write a
`world.yaml` with your waypoints, and pass both via launch arguments.

## 4. PlanSys2 integration

With PlanSys2 installed (`sudo apt install ros-jazzy-plansys2-*`), planning can go
through the PlanSys2 planner service instead of a local backend:

```bash
ros2 launch washbot_bringup plansys2.launch.py                    # terminal 1
ros2 launch washbot_bringup mission_sim.launch.py \
    planner_backend:=plansys2                                     # terminal 2
```

The quickest connectivity check is the planner client, which sends the packaged
domain/problem to whatever backend you point it at:

```bash
ros2 run washbot_planning planner_client --ros-args -p backend:=plansys2
```

## 5. Temporal mode (durations, battery, recharging)

The temporal domain needs a temporal planner — POPF, which PlanSys2 installs
(`ros-jazzy-popf`):

```bash
ros2 launch washbot_bringup mission_sim.launch.py \
    domain_mode:=temporal planner_backend:=popf
```

In this mode the problem builder emits travel times (derived from waypoint distances
and `nominal_speed`), per-fixture cleaning durations, battery drains and the charger
location; POPF minimizes makespan and inserts `recharge` actions when the battery
budget demands it. Try `washbot_planning/pddl/problems/washroom_small_temporal.pddl`
directly — its starting battery (18%) is deliberately too low to finish without a
recharge:

```bash
python3 -m washbot_planning.cli solve --domain domain_temporal.pddl \
    --problem washroom_small_temporal.pddl --backend popf
```

## Version notes

- **Jazzy**: what I develop and test on; the fully-simulated stack (`mission_sim`)
  additionally runs in CI against a Jazzy container.
- **Humble/Iron**: no known incompatibilities — the code avoids distro-specific
  paths and uses long-stable APIs — but I do not test them routinely. The one thing
  to check on Humble is the Nav2 params file (plugin names occasionally shift
  between Nav2 releases; see [troubleshooting](troubleshooting.md)).
