# Example mission artifacts

Real, unedited artifacts from simulated mission runs (timestamps and all), referenced
throughout the documentation:

| File | Scenario | Outcome |
|---|---|---|
| [mission_log_nominal.txt](mission_log_nominal.txt) | No faults, goals `[basin, commode]` | SUCCEEDED, 6 steps, 46.5 s, 0 replans |
| [mission_log_recovery.txt](mission_log_recovery.txt) | `fail_edges:='[hall->commode]'` | SUCCEEDED via shower detour, 1 replan |
| [mission_log_unreachable_abort.txt](mission_log_unreachable_abort.txt) | `fail_locations:='[commode]'` | ABORTED: goal unreachable after exhausting both approaches |
| [mission_log_gazebo.txt](mission_log_gazebo.txt) | Gazebo + real Nav2, `mission_tb3.launch.py`, goals `[all]` | SUCCEEDED, 7 steps, 82.8 s, 0 replans |
| [mission_report_recovery.json](mission_report_recovery.json) | Report written by the recovery run | Full event history + per-step timings |
| [mission_report_gazebo.json](mission_report_gazebo.json) | Report written by the Gazebo run | Full event history + per-step timings |

Reproduce any of them with the corresponding launch arguments — see
[getting_started.md](../getting_started.md). Two executed trajectories are plotted
from recorded data:

- [../media/mission_detour_path.png](../media/mission_detour_path.png) — the recovery
  run against the simulated server
  (raw data: [../media/mission_detour_path.csv](../media/mission_detour_path.csv))
- [../media/gazebo_mission_path.png](../media/gazebo_mission_path.png) — the Gazebo +
  Nav2 run, recorded from `/amcl_pose` with
  `pose_recorder` (raw data:
  [../media/gazebo_mission_path.csv](../media/gazebo_mission_path.csv))
