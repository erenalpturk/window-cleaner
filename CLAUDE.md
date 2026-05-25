# Claude Behavioral Guidelines (Karpathy-Inspired)

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

Tradeoff: These guidelines bias toward caution over speed. For trivial tasks, use judgment.

1. Think Before Coding
Don't assume. Don't hide confusion. Surface tradeoffs.

Before implementing:

State your assumptions explicitly. If uncertain, ask.
If multiple interpretations exist, present them - don't pick silently.
If a simpler approach exists, say so. Push back when warranted.
If something is unclear, stop. Name what's confusing. Ask.
2. Simplicity First
Minimum code that solves the problem. Nothing speculative.

No features beyond what was asked.
No abstractions for single-use code.
No "flexibility" or "configurability" that wasn't requested.
No error handling for impossible scenarios.
If you write 200 lines and it could be 50, rewrite it.
Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

3. Surgical Changes
Touch only what you must. Clean up only your own mess.

When editing existing code:

Don't "improve" adjacent code, comments, or formatting.
Don't refactor things that aren't broken.
Match existing style, even if you'd do it differently.
If you notice unrelated dead code, mention it - don't delete it.
When your changes create orphans:

Remove imports/variables/functions that YOUR changes made unused.
Don't remove pre-existing dead code unless asked.
The test: Every changed line should trace directly to the user's request.

4. Goal-Driven Execution
Define success criteria. Loop until verified.

Transform tasks into verifiable goals:

"Add validation" → "Write tests for invalid inputs, then make them pass"
"Fix the bug" → "Write a test that reproduces it, then make it pass"
"Refactor X" → "Ensure tests pass before and after"
For multi-step tasks, state a brief plan:

1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

These guidelines are working if: fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.

---

# CLAUDE.md — Project-Specific Guidelines

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Status

**Phase 4 complete** (`status: phase_4_complete`, `current_phase: 4` in the roadmap frontmatter). The full codebase exists: `docker/`, all seven `src/window_cleaner_*` packages, `docs/`, `results/`, and `media/plots/`. All phase work (1–4) plus the post-Phase-3 plan change (deterministic `waypoint_follower` replacing Nav2 for driving — see below) is committed. Remaining open items are the manual, user-owned demo video (roadmap tasks 4.7–4.9, see [docs/demo_storyboard.md](docs/demo_storyboard.md)). Always re-check the roadmap frontmatter for the live status.

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
- Gazebo Fortress / Ignition (official pairing with Humble; CLI is `ign gazebo`, not `gz sim`)
- Deterministic `waypoint_follower` is the **default driving controller** (plan change 2026-05-16, see roadmap Phase 3 AI Notes). Nav2 (NavFn + Regulated Pure Pursuit + custom BT) is **retained as the documented advanced/alternative mode**, not the default.
- OpenCV 4.x + cv_bridge (Python)
- Dev environment: Docker Desktop + **Foxglove Studio** (WebSocket, port 8765) on MacBook Air M4. **XQuartz/X11 and RViz do not work on Apple Silicon** (GLX context cannot be created); Gazebo runs headless and all visualisation is via Foxglove.

**Key architectural decision — defend in the report:** The robot does NOT climb a real vertical wall. The world uses `<gravity>0 0 0</gravity>` and a **2D planar abstraction** of the glass surface. Framed as "vertical-surface kinematics modeled via 2D planar abstraction." This is intentional, not a bug.

**Package layout (all exist under `src/`):**
- `window_cleaner_description` — URDF/xacro, meshes (`ament_cmake`)
- `window_cleaner_worlds` — SDF worlds, materials (`ament_cmake`)
- `window_cleaner_bringup` — launch files, Nav2 config, occupancy maps (`ament_cmake`)
- `window_cleaner_perception` — camera, frame detector, dirt segmenter (`ament_python`)
- `window_cleaner_planning` — Boustrophedon coverage planner, `waypoint_follower` (default driver), `path_follower` (Nav2 alternative mode) (`ament_python`)
- `window_cleaner_control` — vacuum/brush state machine (`ament_python`)
- `window_cleaner_evaluation` — metrics collection, benchmark, plotting (`ament_python`)

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

# Run the sim (headless + Foxglove — see docs/RUNNING.md for the full flow)
ros2 launch window_cleaner_bringup sim.launch.py rviz:=false gui:=false foxglove:=true
ros2 launch window_cleaner_perception perception.launch.py

# Default autonomous coverage (NO Nav2 — deterministic waypoint follower)
ros2 run window_cleaner_planning coverage_planner --ros-args --params-file <planning_params.yaml>
ros2 run window_cleaner_planning waypoint_follower --ros-args -p use_sim_time:=true

# Advanced/alternative Nav2 mode (retained reference flow)
ros2 launch window_cleaner_bringup nav2.launch.py        # then coverage_planner + path_follower

ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Debug (visualisation is Foxglove only — RViz/rviz2 cannot run on Apple Silicon)
ros2 topic list
ros2 topic hz /topic_name
ros2 topic echo /topic_name --once
ros2 node list

# URDF validation
xacro src/window_cleaner_description/urdf/robot.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf

# Standalone Gazebo (Ignition Fortress — NOT `gz sim`)
ign gazebo worlds/glass_basic.sdf
```

Visualisation/teleop is **Foxglove Studio** over WebSocket (`ws://localhost:8765`).
XQuartz/X11 forwarding is **not** used (GLX is broken on Apple Silicon), so there
is no `xhost` step. Full bring-up: [docs/RUNNING.md](docs/RUNNING.md) ·
Türkçe [docs/RUNNING.tr.md](docs/RUNNING.tr.md).

## Workflow Rules

1. **One task at a time, in order.** Don't skip ahead or batch-implement multiple roadmap tasks before verifying the previous works.
2. **URDF/SDF errors → delete and rewrite, do not iterate.** XML inertia/joint errors compound; starting clean is faster than debugging.
3. **Nav2 tuning → change ONE parameter at a time**, test, log the result. Never change five parameters simultaneously.
4. **OpenCV pipelines → always publish a debug image topic** for visual verification (Foxglove Image panel) before integrating downstream.
5. **Test new nodes standalone with `ros2 run` BEFORE adding to a launch file.**
6. **Architectural changes require asking the user first (in Turkish).** Don't silently restructure packages or change message types.
7. **After every commit, verify the push to remote succeeded.**
8. **Record errors in AI Notes (Turkish):** if a bash command fails unexpectedly, write both the error and the fix into the relevant phase's AI Notes section.
9. **Keep [docs/RUNNING.md](docs/RUNNING.md) AND its Turkish counterpart [docs/RUNNING.tr.md](docs/RUNNING.tr.md) current.** If any change affects how the sim is brought up — `docker/Dockerfile`, `docker/docker-compose.yml`, `src/window_cleaner_bringup/launch/*`, exposed ports, env vars, required external tools, or the Foxglove workflow — update **both** files in the **same commit**. The Turkish version is the user's primary reference; out-of-date bring-up instructions cost time every new session. Code/commands/file paths stay in English in both files; only the prose narration is translated.

## Verifying Phase Completion

Before claiming a phase is done:
- All `- [ ]` in that phase are flipped to `- [x]`
- AI Notes section filled in Turkish (commit SHA, key parameter values, problems encountered, screen recording path)
- End-of-phase commit pushed with the message format from the roadmap (e.g., `feat(phase-1): URDF, world, bringup complete — manual driving works`)
- User has seen a demo (screen recording or photo)
