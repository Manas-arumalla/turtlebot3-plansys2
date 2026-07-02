# Extending WashBot

The system was structured so the three most likely changes are local, mechanical and
hard to get wrong. Each recipe below ends with how to verify the change without a
robot.

## 1. Change the world (new map, new waypoints, new fixtures)

Everything lives in one file: `washbot_control/config/world.yaml`.

```yaml
world:
  locations:
    mirror: {x: 0.55, y: -1.75, yaw: 3.14, dirty: true, clean_duration: 6.0}
  edges:
    - [hall, mirror]
```

Rules the loader enforces: every edge endpoint must be a declared location; `start`
and `charger` must exist. Rules you should enforce: poses in free space, edges
traversable in a straight line if you use the fake server (Nav2 plans around
obstacles; the fake server drives straight).

For a new building: SLAM a map, put the `.pgm`/`.yaml` in `washbot_bringup/maps/`,
write the world file, and pass both to the launch files. Check your graph visually —
this renders the map with waypoints and edges overlaid:

```bash
python3 -m washbot_planning.cli render-world \
    --map-yaml <your_map.yaml> --world <your_world.yaml> --out /tmp/world.png
```

**Verify:** `ros2 launch washbot_bringup mission_sim.launch.py world_file:=<path>`.
The fake server, mission controller and markers all read the same file.

## 2. Teach the robot a new skill (new PDDL action)

Example: a `mop` action for floor zones. Three steps, three files.

**Model it** — add to `washbot_planning/pddl/domain_strips.pddl`:

```pddl
(:predicates ... (needs_mopping ?l - location) (mopped ?l - location))

(:action mop
  :parameters (?r - robot ?l - location)
  :precondition (and (robot_at ?r ?l) (needs_mopping ?l))
  :effect (and (not (needs_mopping ?l)) (mopped ?l)))
```

**Handle it** — `washbot_control/washbot_control/handlers/mop.py`, implementing
`ActionHandler.start(step, done_cb)` / `cancel()`. `CleanHandler` is the template for
timed work; `NavigateHandler` for work delegated to another action server. The
contract: `start()` returns immediately, `done_cb(success, message)` is called exactly
once, and `cancel()` must prevent any late callback.

**Register it** — in `washbot_control/handlers/__init__.py`:

```python
'mop': MopHandler(node, world),
```

Wire the symbolic effect in `MissionEngine._apply_effects` if the action changes
world state the planner needs to see (as `clean` marks fixtures cleaned), extend the
problem builder to emit the new predicates from the snapshot, and add a case to the
engine's failure policy if a failed `mop` should mean something other than
retry-then-abort.

**Verify:** unit-test at engine level first (see `test_engine.py` — simulate step
outcomes, no ROS needed), then run `mission_sim.launch.py`.

## 3. Add a planner backend

Implement one method in `washbot_planning/backends.py`:

```python
class MyBackend:
    name = 'my-planner'
    def solve(self, domain_text: str, problem_text: str) -> PlanningResult: ...
```

Return a `Plan` (see `Plan.from_popf_output` for parsing planner output text) and
register the name in `resolve()`. Everything downstream — validation, benchmarks,
mission execution — works unchanged, and `washbot_plan benchmark
--backends my-planner,...` will benchmark it against the others for free.

## 4. Swap simulated cleaning for a real tool

`CleanHandler` is intentionally the only place that "cleaning" exists. Replace its
timed loop with your device's interface (a serial command, a service call, an action
client) and keep the callback contract. The planner, engine, and mission controller
never know the difference — that was the point of the handler layer.

## Conventions

- Anything that makes *decisions* stays out of ROS nodes (see the layering rule in
  [architecture.md](architecture.md)) so it can be unit-tested.
- New PDDL constructs must either be supported by the internal parser or rejected by
  it with a useful error — extend `parser.py` accordingly.
- `python3 -m pytest washbot_planning/test washbot_control/test` and
  `python3 -m flake8` must pass; CI runs both.
