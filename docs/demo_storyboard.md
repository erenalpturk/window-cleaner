# Demo Video — Storyboard / Shot List

Roadmap tasks 4.7–4.9 (screen recording + editing) are **manual, user-owned**
— a binary video cannot be produced by the AI. This shot list maps the
roadmap's ~3-minute structure to concrete commands and the exact Foxglove
panels to show. Record on the Mac, edit in iMovie/DaVinci, drop the final
clip at `media/demo.mp4` and note its path in roadmap AI-Notes Phase 4.

## Capture setup

* **Tool:** QuickTime Player → File → New Screen Recording (simplest on
  macOS; OBS only if you want a webcam/overlay).
* **Layout:** full-screen Foxglove Studio with the saved panel layout from
  [RUNNING.md](RUNNING.md) §4 (Panels: 3D, Image, Teleop, Raw Messages).
  Optionally a small terminal tile top-right showing the launch commands.
* **Speed-up:** record the autonomous run at 1× and speed the coverage
  segment ~8× in editing (a full run is several minutes on M4).
* Royalty-free music + section titles in editing.

## Pre-roll (before recording)

```bash
# Terminal 1 — sim (basic world), headless + Foxglove
docker compose -f docker/docker-compose.yml run --service-ports --rm \
  --name wc-sim ros2-dev \
  bash -c "source install/setup.bash && \
    ros2 launch window_cleaner_bringup sim.launch.py gui:=false foxglove:=true"
# wait for: foxglove_bridge ... listening on 8765
```
Connect Foxglove (`ws://localhost:8765`, **Foxglove WebSocket**), load the
saved layout. In the default (no-Nav2) mode `sim.launch.py` publishes the
static `map→odom` TF, so the 3D Fixed frame can stay `map`.

## Segments (~3:00 total)

| Time | Content | Commands / panels |
|---|---|---|
| 0:00–0:15 | **Title.** Project name, "ROS 2 + Gazebo, 2-D planar abstraction of a window cleaner", student name. | Title card (editing). |
| 0:15–0:45 | **Spawn + manual drive.** Robot on the blue glass with brown dirt discs; drive a few metres with the Teleop pad. | Foxglove 3D panel + Teleop panel. `ros2 run teleop_twist_keyboard ...` optional. |
| 0:45–1:30 | **Perception.** Camera feed; frame boundary (blue) on `/perception/debug_frame`; dirt contours (green) + centres (red) on `/perception/debug_dirt`. | Start `perception.launch.py` (Terminal 3, `docker compose exec wc-sim ...`). Foxglove Image panel cycling the debug topics. |
| 1:30–2:30 | **Autonomous coverage (≈8× sped up).** Default driver = `coverage_planner` + `waypoint_follower` (no Nav2); show the boustrophedon `/planning/coverage_path` in 3D, the robot snaking with clean two-stage U-turns, `/control/state` flipping MOVING↔CLEANING, mission_state WAITING→RUNNING→DONE. Be honest: coverage is intentionally partial (planner inset + strip gap). | Terminals per RUNNING.md "Running coverage — default mode" (coverage_planner, waypoint_follower) + control. 3D panel: `/planning/coverage_path` + `/planning/glass_boundary_viz` + TF; Raw Messages: `/control/mission_state`. (Nav2 reference flow is the alternative — see RUNNING.md.) |
| 2:30–3:00 | **Metrics + conclusion.** Show `results/metrics.csv` and the four `media/plots/*.png`; one line on the honest partial-coverage / 2-D-abstraction framing. | `cat results/metrics.csv`; the plot PNGs. Closing card → [known_issues.md](known_issues.md) takeaway. |

## Optional: record straight from the benchmark

For a hands-off coverage segment you can screen-record while
`run_benchmark.sh --world glass_basic --runs 1` executes, then trim. The
rosbag (`--bag`) of run 1 can also be replayed for a clean re-record:

```bash
ros2 bag play media/bags/glass_basic_run1
```

## Deliverable checklist

- [ ] `media/demo.mp4` recorded (~3 min, sped-up coverage segment)
- [ ] Section titles + the explicit **Limitations** mention (2-D
      abstraction, partial coverage — earns academic credit)
- [ ] Path + duration recorded in roadmap **AI Notes — Phase 4**
