# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Status

This repo currently contains **only** [PROJECT_ROADMAP (1).md](PROJECT_ROADMAP (1).md). There is no code, no `src/`, no Docker setup yet — the project is at **Phase 0** (prerequisites). Do not assume any folder structure described in the roadmap already exists; create it as you progress.

## Source of Truth

[PROJECT_ROADMAP (1).md](PROJECT_ROADMAP (1).md) is the **single source of truth** for this project. Before doing any work:
1. Read the roadmap to determine `current_phase` (frontmatter field).
2. Find the first unchecked `- [ ]` task in that phase.
3. Verify all checkboxes in prior phases are `[x]` before moving forward.
4. After completing tasks, flip `- [ ]` → `- [x]` in the roadmap itself.
5. At the end of every phase, fill the **"AI Notes — Phase N"** section (in Turkish).
6. Update `current_phase` and `last_updated` frontmatter fields at the start of each new phase.

If you and the user agree on a plan change, **edit the roadmap first**, then implement.

## Language Policy (CRITICAL)

- **Talk to the user in Turkish.** All chat messages, explanations, questions, error reports, progress updates → Turkish. The user is a Turkish-speaking student (Alp Eren Türk).
- **Code, comments, commit messages, technical docs → English.** Industry standard.
- **AI Notes sections in the roadmap → Turkish.**
- The roadmap itself stays in English (for accurate parsing), but translate when quoting it to the user.
- If you catch yourself replying in English, switch to Turkish.

## Project Architecture (Big Picture)

**Goal:** ROS2 + Gazebo simulation of an autonomous window-cleaning robot that detects dirty regions via camera and achieves full coverage without hitting frames.

**Stack:**
- ROS2 Humble (Apple Silicon ARM64, via Docker)
- Gazebo Fortress (official pairing with Humble)
- Nav2 (navigation, costmap, behavior tree)
- OpenCV 4.x + cv_bridge (Python)
- Dev environment: Docker Desktop + XQuartz (X11 forwarding) on MacBook Air M4

**Key architectural decision — defend in the report:** The robot does NOT climb a real vertical wall. The world uses `<gravity>0 0 0</gravity>` and a **2D planar abstraction** of the glass surface. Framed as "vertical-surface kinematics modeled via 2D planar abstraction." This is intentional, not a bug.

**Package layout (to be created under `src/`):**
- `window_cleaner_description` — URDF/xacro, meshes (`ament_cmake`)
- `window_cleaner_worlds` — SDF worlds, materials (`ament_cmake`)
- `window_cleaner_bringup` — launch files, Nav2 config (`ament_cmake`)
- `window_cleaner_perception` — camera, frame detector, dirt segmenter (`ament_python`)
- `window_cleaner_planning` — Boustrophedon coverage planner, path follower (`ament_python`)
- `window_cleaner_control` — vacuum/brush state machine (`ament_python`)
- `window_cleaner_evaluation` — metrics collection, plotting (`ament_python`)

**Topic contracts between layers (do not break):**
- Perception publishes: `/perception/glass_boundary` (PolygonStamped), `/perception/dirty_regions` (PoseArray + Float32MultiArray for areas), `/perception/debug_*` (Image)
- Planning consumes `glass_boundary`, publishes `/planning/coverage_path` (nav_msgs/Path)
- Control consumes `/odom`, `/perception/dirty_regions`, `/robot/scan`; publishes `/control/vacuum_cmd`, `/control/brush_cmd`, `/control/state`
- Gazebo bridges: `/robot/camera/image_raw`, `/robot/camera/camera_info`, `/robot/scan`, `/cmd_vel`, `/tf`, `/tf_static`, `/clock`

## Common Commands

All ROS2 work runs inside the Docker container, not the host Mac.

```bash
# Build the dev image (Phase 1)
docker compose -f docker/docker-compose.yml build

# Interactive shell in container
docker compose -f docker/docker-compose.yml run --rm ros2-dev bash

# Attach another terminal to a running container
docker compose -f docker/docker-compose.yml exec ros2-dev bash

# Inside container — build workspace
cd /workspace && colcon build --symlink-install
colcon build --packages-select PACKAGE_NAME    # single package
source install/setup.bash

# Run the full simulation
ros2 launch window_cleaner_bringup sim.launch.py
ros2 launch window_cleaner_perception perception.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Debug
ros2 topic list
ros2 topic hz /topic_name
ros2 topic echo /topic_name --once
ros2 node list
rviz2

# URDF validation
xacro src/window_cleaner_description/urdf/robot.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf

# Standalone Gazebo
gz sim worlds/glass_basic.sdf
```

**Host-side X11 setup (re-run after every reboot — consider adding to `.zshrc`):**
```bash
xhost +localhost
```

## Workflow Rules

1. **One task at a time, in order.** Don't skip ahead or batch-implement multiple roadmap tasks before verifying the previous works.
2. **URDF/SDF errors → delete and rewrite, do not iterate.** XML inertia/joint errors compound; starting clean is faster than debugging.
3. **Nav2 tuning → change ONE parameter at a time**, test, log the result. Never change five parameters simultaneously.
4. **OpenCV pipelines → always publish a debug image topic** for visual verification in RViz before integrating downstream.
5. **Test new nodes standalone with `ros2 run` BEFORE adding to a launch file.**
6. **Architectural changes require asking the user first (in Turkish).** Don't silently restructure packages or change message types.
7. **After every commit, verify the push to remote succeeded.**
8. **Record errors in AI Notes (Turkish):** if a bash command fails unexpectedly, write both the error and the fix into the relevant phase's AI Notes section.

## Verifying Phase Completion

Before claiming a phase is done:
- All `- [ ]` in that phase are flipped to `- [x]`
- AI Notes section filled in Turkish (commit SHA, key parameter values, problems encountered, screen recording path)
- End-of-phase commit pushed with the message format from the roadmap (e.g., `feat(phase-1): URDF, world, bringup complete — manual driving works`)
- User has seen a demo (screen recording or photo)
