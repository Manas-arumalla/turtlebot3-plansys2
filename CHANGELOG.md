# Changelog

## 1.1.0

Verified the full stack end to end in Gazebo (Harmonic) with a TurtleBot3 Burger
on ROS 2 Jazzy, and fixed what that surfaced.

### Fixed
- Jazzy TurtleBot3 compatibility: the gz bridge expects `TwistStamped` on
  `cmd_vel`, so `nav2_params.yaml` now sets `enable_stamped_cmd_vel` for the
  controller and behavior servers, and the cleaning handler's spin honours a new
  `stamped_cmd_vel` parameter (enabled by `mission_tb3.launch.py`). Without this,
  Nav2 believes it is driving while the robot never moves.
- `mission_tb3.launch.py` sequencing: the initial pose is now published repeatedly
  over ~8 s (covering slow AMCL activation) and the mission controller starts at
  t+20 s, after localization has settled — a navigation goal sent into a
  half-initialized stack used to read as a blocked passage.

### Changed
- `planner_backend: auto` now falls back to the internal A* instead of GBFS when
  POPF is absent: at mission scale the plans are noticeably shorter (single-loop
  tours instead of revisits) for a few extra milliseconds of search.
- `pose_recorder` can record `PoseWithCovarianceStamped` (AMCL) and `Odometry`
  streams in addition to `PoseStamped`, so executed-path figures can be produced
  from Gazebo and real-robot runs.

## 1.0.0

Ground-up rebuild of the original washroom-cleaning demo into a complete
plan–execute–recover system.

### Added
- `washbot_planning`: pure-Python PDDL toolkit (parser, grounder with static-predicate
  pruning, A*/GBFS search with the additive heuristic, plan validator), pluggable
  planner backends (PlanSys2 service / POPF subprocess / internal search), snapshot-based
  problem builder, scalable facility generator, benchmark harness with plots, and the
  `washbot_plan` CLI (solve / validate / gen-problem / benchmark / render-world).
- `washbot_control`: YAML world model, ROS-free mission engine with explicit recovery
  policy (edge blocking, retry budgets, replan limits), action-handler layer
  (Nav2 `move`, timed `clean`/`deep_clean`, `recharge`), mission controller node with
  status topics, RViz markers, watchdogs and JSON mission reports, simulated Nav2
  server with edge/location fault injection, pose recorder, initial-pose publisher.
- `washbot_bringup`: simulation, TurtleBot3+Nav2, and PlanSys2 launch profiles; complete
  Jazzy Nav2 configuration for a TB3 Burger; washroom map; RViz profile.
- Temporal PDDL 2.1 domain with durations, battery drain and forced recharging,
  alongside the STRIPS domain.
- 49 unit tests, including the full recovery matrix of the mission engine; CI with
  lint + core tests + colcon build.
- Documentation set: architecture, getting started, PDDL modelling, benchmarks
  (with committed results), extension recipes, troubleshooting.

### Changed
- Restructured from a single mislaid package into three packages with a strict
  dependency direction (`bringup → control → planning`).
- Replaced the `navigate`/`clean` demo domain with a richer model: directed
  connectivity, dirt as a consumable precondition, deep-clean tier.
- The map, AMCL/Nav2 parameters and initial-pose tooling were rewritten without
  hardcoded user paths or distro-specific file references.

### Fixed
- The initial-pose script no longer uses invented rclpy APIs and publishes reliably.
- Entry points declared in the original packaging (`plansys2_executor_bridge`,
  `fake_nav2_server`, `fake_nav2_server_reject_commode`) now exist as working,
  superseding implementations (`mission_controller`, `fake_nav2_server` with
  parameterized fault injection).
