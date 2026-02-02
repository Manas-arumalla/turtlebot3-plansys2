# TurtleBot3 PlanSys2 Washroom Cleaning Demo

A ROS 2 project that demonstrates autonomous task planning for a TurtleBot3 robot using PlanSys2. The robot plans and executes a washroom cleaning routine by navigating between locations and performing cleaning actions.

## Overview

This project integrates:
- **PlanSys2** - AI planning framework for ROS 2
- **TurtleBot3** - Mobile robot platform
- **PDDL** - Planning Domain Definition Language for task specification

The robot autonomously plans how to clean multiple locations (basin, commode) in optimal order.

## Project Structure

```
turtlebot3-plansys2-main/
├── src/plansys2_washroom_pkg/
│   ├── pddl/                    # Planning domain & problem files
│   │   ├── washroom_domain.pddl
│   │   └── washroom_problem.pddl
│   ├── config/                  # ROS 2 parameters
│   ├── maps/                    # Navigation maps
│   └── call_plansys2_planner.py # Planner test client
├── plansys2_washroom/           # Python module
└── scripts/
    └── publish_initialpose.py   # Robot pose initializer
```

## Dependencies

- ROS 2 Humble
- rclpy
- nav2_msgs
- geometry_msgs
- PlanSys2
- popf (PDDL planner)

## Installation

```bash
# Clone into your ROS 2 workspace
cd ~/ros2_ws/src
git clone <repository-url> turtlebot3-plansys2

# Build
cd ~/ros2_ws
colcon build --packages-select plansys2_washroom_pkg
source install/setup.bash
```

## How It Works

### PDDL Domain (`washroom_domain.pddl`)

Defines the robot's world model with two actions:

**Types:**
- `robot` - The cleaning robot
- `location` - Places the robot can visit

**Predicates:**
- `(robot_at ?r ?l)` - Robot r is at location l
- `(connected ?l1 ?l2)` - Locations are connected
- `(cleaned ?l)` - Location has been cleaned

**Actions:**

| Action | Parameters | What it does |
|--------|------------|--------------|
| `navigate` | robot, from, to | Moves robot between connected locations |
| `clean` | robot, location | Cleans the current location |

### PDDL Problem (`washroom_problem.pddl`)

Defines the specific cleaning scenario:

**Objects:**
- Robot: `r1`
- Locations: `start`, `basin`, `commode`

**Initial State:**
- Robot starts at `start`
- Connections: start↔basin, start↔commode

**Goal:** Clean both `basin` and `commode`

### Generated Plan

The planner produces an optimal sequence:
```
1. navigate r1 start basin
2. clean r1 basin
3. navigate r1 basin start
4. navigate r1 start commode
5. clean r1 commode
```

## Usage

### 1. Start PlanSys2

```bash
ros2 launch plansys2_bringup plansys2_bringup_launch_distributed.py
```

### 2. Test the Planner

```bash
ros2 run plansys2_washroom_pkg call_plansys2_planner
```

This reads the PDDL files and prints the generated plan.

### 3. Set Initial Robot Pose

```bash
python3 scripts/publish_initialpose.py 0.0 0.0 0.0
```

Arguments: `X Y YAW_in_radians`

## Nodes

| Node | Description |
|------|-------------|
| `plansys2_executor_bridge` | Executes planned actions on the robot |
| `fake_nav2_server` | Simulates Nav2 for testing without real navigation |
| `fake_nav2_server_reject_commode` | Test variant that fails on commode navigation |

## Code Explanation

### `call_plansys2_planner.py`

A ROS 2 client that tests the planning service:

1. Creates a client for `planner/get_plan` service
2. Reads domain and problem PDDL files
3. Sends planning request to PlanSys2
4. Parses and displays the action sequence with timing

### `publish_initialpose.py`

Sets the robot's initial position for localization:

1. Takes X, Y, YAW from command line
2. Converts yaw angle to quaternion
3. Publishes `PoseWithCovarianceStamped` to `/initialpose`

## Configuration Files

| File | Purpose |
|------|---------|
| `nav2_params_copy.yaml` | Navigation stack parameters |
| `amcl_params.yaml` | Localization (AMCL) settings |
| `map_server_params.yaml` | Map server configuration |
| `my_map.yaml` + `my_map.pgm` | Environment map |

## License

Apache-2.0
