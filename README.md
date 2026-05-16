# Autonomous Window-Cleaning Robot — ROS 2 + Gazebo Simulation

A ROS 2 Humble + Gazebo Fortress simulation of a robot that autonomously
sweeps a glass surface, detects dirty regions with a camera, and achieves
coverage without colliding with the frame. Runs fully containerised on
Apple Silicon (MacBook Air M4).

> **Key modelling decision:** the robot does not climb a real vertical
> wall. The world uses `<gravity>0 0 0</gravity>` and a **2-D planar
> abstraction** of the glass — *"vertical-surface kinematics modelled via
> 2-D planar abstraction."* This is intentional and defended in the report;
> see [docs/architecture.md](docs/architecture.md) and
> [docs/known_issues.md](docs/known_issues.md).

## Demo

Screen-recorded per [docs/demo_storyboard.md](docs/demo_storyboard.md)
(shot list). Place the final clip at `media/demo.mp4`.

## Architecture

```
Gazebo (Ignition, headless)
  │  /robot/camera/image_raw  /robot/scan  /odom  /clock  /tf
  ▼
Perception      frame_detector → /perception/glass_boundary
                dirt_segmenter → /perception/dirty_regions
  ▼
Planning        coverage_planner → /planning/coverage_path (boustrophedon)
                path_follower    → Nav2 NavigateThroughPoses
  ▼
Nav2            NavFn (global) + Regulated Pure Pursuit (local)
  ▼
Control         cleaning_controller → /control/{vacuum,brush}_cmd
                IDLE / MOVING / CLEANING / EMERGENCY
  ▼
Evaluation      metrics_node → results/metrics.csv (Phase 4)
```

Full layer diagram and the topic-contract table:
[docs/architecture.md](docs/architecture.md). Algorithms (boustrophedon,
OpenCV pipelines, coverage metric): [docs/algorithms.md](docs/algorithms.md).

## Quick start

All ROS 2 work runs inside Docker (not the host Mac).

```bash
# Build the dev image (~6 min on M4, arm64 native)
docker compose -f docker/docker-compose.yml build

# Bring the sim up (see docs/RUNNING.md for the full multi-terminal flow)
docker compose -f docker/docker-compose.yml run --service-ports --rm \
  --name wc-sim ros2-dev \
  bash -c "source install/setup.bash && \
    ros2 launch window_cleaner_bringup sim.launch.py gui:=false foxglove:=true"
```

Visualisation is via **Foxglove Studio** (WebSocket, port 8765) — XQuartz
GLX is broken on Apple Silicon so Gazebo runs headless and RViz is
unavailable. Bring-up details:
[docs/RUNNING.md](docs/RUNNING.md) · Türkçe: [docs/RUNNING.tr.md](docs/RUNNING.tr.md).

### Benchmark (Phase 4)

```bash
docker compose -f docker/docker-compose.yml run --service-ports --rm \
  --name wc-bench ros2-dev \
  bash -c "source install/setup.bash && \
    bash install/window_cleaner_evaluation/share/window_cleaner_evaluation/scripts/run_benchmark.sh --all --runs 3"
```

6 unattended runs (`glass_basic` + `glass_small` × 3) → `results/metrics.csv`
→ `media/plots/`. Results & analysis: [docs/results.md](docs/results.md).

## Folder structure

```
docker/                         Dockerfile, compose, entrypoint
src/
  window_cleaner_description/    URDF / xacro robot model
  window_cleaner_worlds/         SDF worlds (glass_basic/small/large/obstacles)
  window_cleaner_bringup/        launch files, Nav2 config, occupancy maps
  window_cleaner_perception/     camera, frame detector, dirt segmenter
  window_cleaner_planning/       boustrophedon planner, path follower
  window_cleaner_control/        vacuum / brush state machine
  window_cleaner_evaluation/     metrics_node, benchmark, plotting (Phase 4)
docs/                           RUNNING(.tr), architecture, algorithms,
                                results, known_issues, demo_storyboard
results/                        metrics.csv (benchmark output)
media/                          plots/, bags/, demo video
PROJECT_ROADMAP (1).md          single source of truth (phase tracking)
```

## Results summary

6-run benchmark over the two working worlds. Coverage is intentionally
**partial** (the frozen Phase-3 navigation covers ~4–6 of 9 strips on the
basic world); ABORTED/TIMEOUT runs are recorded honestly. Full table,
plots and analysis: [docs/results.md](docs/results.md).

## Known limitations

Carried over from Phase 3 and documented, not hidden — honest limitations
earn academic credit. Obstacle-world navigation, U-turn overshoot, low M4
RTF, lidar height. See [docs/known_issues.md](docs/known_issues.md).

## License

MIT.
