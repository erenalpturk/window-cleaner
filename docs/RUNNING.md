# Running the Simulation

How to bring the autonomous window-cleaning robot simulation up and down after
the initial Phase 1 setup is complete.

> Türkçe sürümü: [RUNNING.tr.md](RUNNING.tr.md). Bu dosyayı her güncellediğinde
> Türkçe çevirisini de aynı commit'te güncelle.
>
> **Keep this document in sync** with `docker/`, `src/window_cleaner_bringup/launch/`,
> port mappings, env vars, and any external-tool requirements. If you change
> how the sim is launched, update this file in the same commit.

## Prerequisites (one-time)

| Tool | Purpose | Install |
|---|---|---|
| Docker Desktop (Apple Silicon) | Container runtime | https://www.docker.com/products/docker-desktop |
| Foxglove Studio (Apple Silicon) | 3D + camera visualisation + teleop UI | https://foxglove.dev/download |
| Homebrew | Install Foxglove via `brew install --cask foxglove-studio` (optional) | https://brew.sh |

XQuartz is **not** required — Apple Silicon XQuartz cannot create a working
GLX context for OGRE2 / RViz, so the project uses Foxglove WebSocket instead.

First-time-only image build:

```bash
cd /Users/erenalpturk/Desktop/Projects/Robotic
docker compose -f docker/docker-compose.yml build
```

This takes ~10 minutes on M4 the first time, ~1 minute on Docker layer cache hits after that.

## Bring-up (every session)

### 1. Start Docker Desktop

```bash
open -a Docker
```

Wait until the menu-bar whale icon stops animating.

### 2. Launch the sim

```bash
cd /Users/erenalpturk/Desktop/Projects/Robotic

docker compose -f docker/docker-compose.yml run \
  --service-ports --rm --name wc-sim ros2-dev \
  bash -c "source install/setup.bash && \
           ros2 launch window_cleaner_bringup sim.launch.py \
           rviz:=false gui:=false foxglove:=true"
```

To run against the **obstacles** world (Phase 3 hard-world test) add a
`world:=` argument:

```bash
docker compose -f docker/docker-compose.yml run \
  --service-ports --rm --name wc-sim ros2-dev \
  bash -c "source install/setup.bash && \
           ros2 launch window_cleaner_bringup sim.launch.py \
           world:=/workspace/install/window_cleaner_worlds/share/window_cleaner_worlds/worlds/glass_obstacles.sdf \
           rviz:=false gui:=false foxglove:=true"
```

The robot spawns at the **SW corner** `(-2.15, -1.15, 0.05)` so the
boustrophedon plan's first waypoint sits right next to the robot. The
`map → odom` static TF (set in `nav2.launch.py`) offsets back to keep
world coordinates in the `map` frame consistent with the static map.

What this starts inside the container:

| Process | Role |
|---|---|
| `ign gazebo -s --headless-rendering` | Physics + sensor render (no GUI window) |
| `robot_state_publisher` | TF tree from URDF |
| `ros_gz_sim create` | Spawns the robot into the world |
| `ros_gz_bridge` | Translates ROS ↔ Gazebo topics |
| `foxglove_bridge` | WebSocket server on `0.0.0.0:8765` |

The sim is ready when the log shows:

```
[foxglove_bridge-5] [INFO] ... Server listening on port 8765
```

Important flags:

- `--service-ports` — required. `docker compose run` ignores `ports:` from compose.yml without this flag, so 8765 would not reach the Mac host.
- `--name wc-sim` — lets a second terminal join via `docker exec`.
- `rviz:=false gui:=false foxglove:=true` — Apple Silicon defaults. On a Linux host with working OpenGL you can flip these.

### 3. Connect Foxglove Studio

1. Open Foxglove Studio.
2. `Open Connection` → **Foxglove WebSocket** (not Rosbridge — different protocols).
3. URL: `ws://localhost:8765` → **Open**.

The status indicator next to the URL turns green once the bridge accepts
the connection. If it stays red, see Troubleshooting at the bottom.

After connecting, build the layout described in the next section.

### 4. Foxglove panel layout

This is the canonical layout for the project. Build it once, then save it
with **Layout → Save layout as…** so it auto-loads on the next session.

Foxglove starts empty. To add a panel: click the **+** in the tab bar, or
right-click an existing panel → **Split panel** (right / down) and pick
the panel type from the popup.

Recommended layout: 2×2 grid — top-left **3D**, top-right **Image**,
bottom-left **Teleop**, bottom-right **Raw Messages**. You can rearrange
freely; only the per-panel settings below matter.

#### Panel 1 — 3D (sim world + robot + path + costmap)

1. Add panel → **3D**.
2. Click the panel, then click the **gear (⚙) icon** in its top-right.
   The settings open in the left side bar.
3. Set under **Frame**:
   - **Fixed frame**: `map` (use `odom` only if Nav2 is not running yet).
   - **Display frame**: `base_link`.
   - **Follow mode**: `Pose (position + attitude)` so the camera tracks
     the robot.
4. Set under **Scene**:
   - **Label scale**: `0.14` (the default 1.0 makes the TF labels cover
     the robot).
   - Optionally toggle **Show labels: Off** if the labels still bother you.
5. Open **Topics** in the side bar and toggle the eye-icon ON for:
   - `/robot_description` — renders the URDF mesh.
   - `/tf` — TF axes for every link.
   - `/robot/scan` (LaserScan) — laser dots.
   - `/map` (OccupancyGrid) — the static glass map (white interior, black
     frame walls). Phase 3+.
   - `/global_costmap/costmap` (OccupancyGrid) — Nav2 inflation halo
     around the walls. Phase 3+.
   - `/planning/coverage_path` (Path) — the boustrophedon zigzag overlay.
     Phase 3+. Under its sub-settings: **Type: Line**, **Line width: 0.05**.
   - `/plan` (Path) — Nav2's live global plan for the current segment.
     Phase 3+. **Type: Line**, **Line width: 0.03**, different colour.

#### Panel 2 — Image (camera feed + perception debug overlays)

1. Add panel → **Image**.
2. In the panel settings (gear icon):
   - **Topic**: `/robot/camera/image_raw` for the raw feed, or one of
     the perception debug topics:
     - `/perception/debug_image` — frame counter overlay
     - `/perception/debug_frame` — Hough lines + glass boundary
     - `/perception/debug_dirt` — dirt contours in green
   - To watch multiple images at once, add multiple Image panels.

#### Panel 3 — Teleop (manual driving with the on-screen pad)

1. Add panel → **Teleop**.
2. In the panel settings:
   - **Topic**: `/cmd_vel`
   - **Publish rate**: `10` Hz
   - **Linear x max**: `0.20` m/s (matches Nav2's RPP cap)
   - **Angular z max**: `1.0` rad/s
3. The on-screen arrow buttons publish Twist messages while held.

#### Panel 4 — Raw Messages (mission state, parameter values, etc.)

1. Add panel → **Raw Messages**.
2. **Topic**: `/control/mission_state` — flips through WAITING → RUNNING
   → DONE during autonomous coverage (Phase 3+).
3. To watch a different topic, change the Topic field. Useful ones:
   - `/perception/dirty_regions` — PoseArray of detected dirty centres
   - `/odom` — current robot pose
   - `/rosout` — all node logs

#### (Optional) Manual goal-pose publish (Phase 3+)

To send a single Nav2 goal manually without waiting for the autonomous
planner:

1. Add panel → **Publish**.
2. Settings:
   - **Topic**: `/goal_pose`
   - **Message schema**: `geometry_msgs/msg/PoseStamped`
   - **Editing mode**: `On`.
3. In the JSON editor, fill in (example — middle of the glass):
   ```json
   {
     "header": {"frame_id": "map"},
     "pose": {
       "position":    {"x": 1.5, "y": 0.5, "z": 0},
       "orientation": {"x": 0,   "y": 0,   "z": 0, "w": 1}
     }
   }
   ```
   The `frame_id: "map"` field is mandatory — Nav2 rejects empty frames.
4. Click **Publish** to send the goal. The robot drives there.

> Foxglove's "click on 3D to publish" mode also exists but the topic
> remap is finicky on the current build (defaults to `/move_base_simple/goal`
> instead of `/goal_pose`). The Publish panel above is the reliable path.

#### Save the layout

**Layout → Save layout as…** → name it e.g. `wc-default`. Next session,
**Layout → Open** picks it back up.

### 5. Teleoperation

**Option A (recommended): Foxglove Teleop panel** — already configured in
[Panel 3 above](#panel-3-teleop-manual-driving-with-the-on-screen-pad).
Click and hold the arrow buttons to drive.

**Option B: keyboard in a second terminal.**

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keys: `i` forward, `,` back, `j`/`l` rotate, `k` stop, `q`/`z` increase/decrease speed.

## Rebuilding after code changes

Code edits inside `src/` (URDF, SDF, launch files, Python nodes):

```bash
docker compose -f docker/docker-compose.yml run --rm ros2-dev \
  bash -c "colcon build --symlink-install"
```

Thanks to `--symlink-install`, launch / URDF / SDF / Python sources are
hot-linked into `install/`. After this single build, future edits to those
files are picked up just by restarting the launch — no re-build needed. New
packages, new entry points, or C++ changes still require a build.

Dockerfile / image changes:

```bash
docker compose -f docker/docker-compose.yml build
```

## Shut-down

In the launch terminal: **Ctrl+C** (graceful — propagates SIGINT to the sim, bridge, and Foxglove).

From another terminal, if the launch terminal is gone:

```bash
docker stop wc-sim
```

## Running the perception layer (Phase 2+)

Start the sim first, then in a second terminal:

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 launch window_cleaner_perception perception.launch.py
```

This starts three nodes: `camera_node`, `frame_detector`, `dirt_segmenter`.

To verify visually, open the Image panel from
[Foxglove panel layout](#4-foxglove-panel-layout) and switch its **Topic**
through `/perception/debug_image` (raw frame counter), `/perception/debug_frame`
(detected glass boundary), and `/perception/debug_dirt` (dirty contours in
green).

Tune detection thresholds without rebuilding by editing
`src/window_cleaner_perception/config/perception_params.yaml` and restarting
`perception.launch.py`.

## Running Nav2 (Phase 3 Sub-phase B+)

> **Advanced/alternative mode only.** Default coverage no longer uses
> Nav2 — see "Running coverage — default mode" below. This section
> applies if you deliberately run the Nav2 reference flow. Launch the
> sim with `map_odom_tf:=false` in this mode so the sim's static
> `map→odom` does not collide with the one Nav2 publishes.

The Nav2 stack consumes a static occupancy grid (`maps/glass_basic.pgm`)
and publishes the `map` frame plus the global/local costmaps. It owns
the identity `map → odom` static TF — no AMCL is used because `/odom`
is ground-truth from the Ignition DiffDrive plugin and the robot spawns
at world (0, 0).

This is the Sub-phase B test surface: run Nav2 standalone and prove the
robot reaches a manually-published `/goal_pose`. From Sub-phase C
onwards Nav2 is part of the Planning launch flow (next section).

**1. Make sure the sim is running.** Run the [§"Launch the sim"](#2-launch-the-sim) command in Terminal 1.

**2. In a second terminal, bring up the Nav2 stack:**

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 launch window_cleaner_bringup nav2.launch.py
```

You should see the lifecycle manager log lines (in order, within
~3-5 seconds):

```
Configuring map_server
Activating map_server
... Configuring controller_server
... Activating bt_navigator
Managed nodes are active
```

**3. Verify in Foxglove.** With the [§"Foxglove panel layout"](#4-foxglove-panel-layout)
loaded, the 3D panel should now show:

- **`/map`** — white glass interior with black frame walls
- **`/global_costmap/costmap`** — coloured inflation halo around the
  walls
- The robot at world (0, 0), in the middle of the glass

**4. Send a manual goal** using the Publish panel described in the
[§"(Optional) Manual goal-pose publish"](#optional-manual-goal-pose-publish-phase-3)
subsection. The robot should plan and drive there.

Regenerate the map after editing the SDF world:

```bash
python3 src/window_cleaner_bringup/maps/gen_map.py
```

## Running coverage — default mode (deterministic waypoint follower)

The **default** driving controller is `waypoint_follower`: a deterministic
turn-then-drive node that consumes the same `/planning/coverage_path` and
publishes the same `/control/mission_state` (WAITING → RUNNING → DONE) and
`/cmd_vel`. It bypasses Nav2 entirely. Rationale: the world is an empty,
gravity-free planar abstraction with ground-truth odometry, so Nav2's
reactive stack was overkill and the source of path-following instability.
See PROJECT_ROADMAP Phase 3 AI Notes (plan change 2026-05-16). The Nav2
flow below is retained as the advanced/alternative mode.

Needs **three** terminals — sim, coverage planner, waypoint follower. No Nav2.

**Terminal 1 — Sim:** run the [§"Launch the sim"](#2-launch-the-sim) command.

**Terminal 2 — Coverage planner:**

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 run window_cleaner_planning coverage_planner \
  --ros-args --params-file /workspace/install/window_cleaner_planning/share/window_cleaner_planning/config/planning_params.yaml
```

**Terminal 3 — Waypoint follower:**

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 run window_cleaner_planning waypoint_follower --ros-args -p use_sim_time:=true
```

Expected: the robot drives each strip and performs a clean two-stage
U-turn (turn ~90° → advance ~strip spacing → turn ~90°) at every strip
transition; `/control/mission_state` goes WAITING → RUNNING → DONE. The
cleaning controller and evaluation layers attach exactly as before (same
contract). Tune live without rebuilding via `--ros-args -p`:
`max_linear_vel`, `max_angular_vel`, `turn_threshold`,
`position_tolerance`, `control_frequency` (defaults in
`waypoint_follower.py`).

**Foxglove (simple mode):** `sim.launch.py` now publishes a static
`map→odom` TF (`map_odom_tf:=true`, default), so the 3D panel **Fixed
frame can stay `map`**. There is no `/map` OccupancyGrid in this mode
(no Nav2); the window outline is drawn from `coverage_planner`'s
**`/planning/glass_boundary_viz`** (latched Path). In the 3D panel enable:
`/robot_description`, `/tf`, `/planning/glass_boundary_viz` (window
frame), `/planning/coverage_path` (zigzag). The robot should move over
both.

---

## Running the planning layer — advanced Nav2 mode (Phase 3 Sub-phase C+)

> **Alternative/reference mode.** The default is the deterministic
> waypoint follower above. This Nav2 flow (custom `RemovePassedGoals`
> BT + RPP) is kept for reference; see PROJECT_ROADMAP Phase 3 AI Notes
> (plan change 2026-05-16).

The planning layer has two nodes:

| Node | Topic / Action | Role |
|---|---|---|
| `coverage_planner` | publishes `/planning/coverage_path` (latched) | Builds the boustrophedon zigzag once at startup. |
| `path_follower` | calls `navigate_through_poses` action; publishes `/control/mission_state` | Hands the path to Nav2 and reports WAITING / RUNNING / DONE / ABORTED. |

**Full autonomous coverage** needs three terminals: sim, Nav2, planning.

**Terminal 1 — Sim:** run the [§"Launch the sim"](#2-launch-the-sim) command.

**Terminal 2 — Nav2:** run the [§"Running Nav2"](#running-nav2-phase-3-sub-phase-b)
command. **Wait for `Managed nodes are active`.**

**Terminal 3 — Coverage planner:**

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 run window_cleaner_planning coverage_planner \
  --ros-args --params-file /workspace/install/window_cleaner_planning/share/window_cleaner_planning/config/planning_params.yaml
```

For the obstacles world, swap the params file for the obstacle-aware variant:

```bash
ros2 run window_cleaner_planning coverage_planner \
  --ros-args --params-file /workspace/install/window_cleaner_planning/share/window_cleaner_planning/config/planning_params_obstacles.yaml
```

`coverage_planner` publishes the boustrophedon path once with
`TRANSIENT_LOCAL` durability, so it can stay running while you restart
`path_follower`. Expected log line for the basic world:

```
Published coverage path with 90 waypoints (9 strips, spacing=0.50m) ...
```

For the obstacles world the count is lower (~76) because waypoints
inside inflated obstacle AABBs are dropped.

**Terminal 4 — Path follower:**

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 run window_cleaner_planning path_follower --ros-args -p use_sim_time:=true
```

`path_follower` dispatches the dense waypoint trail as a single
`NavigateThroughPoses` goal; the robot drives the pattern and stops
when the action returns `STATUS_SUCCEEDED`. Mission state transitions
are visible in the terminal:

```
received coverage path with 90 waypoints; waiting for action server...
sending NavigateThroughPoses goal with 90 poses
goal accepted by Nav2
mission_state -> RUNNING
... (long run) ...
Nav2 reported SUCCESS — coverage complete
mission_state -> DONE
```

> **Why separate terminals instead of `planning.launch.py`?** Phase 3
> debugging showed that bundling all three launches via shell `&` made
> the `path_follower` log get drowned in `foxglove_bridge` output. Running
> `coverage_planner` and `path_follower` as standalone `ros2 run` commands
> in their own terminals keeps each node's logs isolated and lets you
> restart one without restarting Nav2 or the sim. The `planning.launch.py`
> file is still installed for batch use.

To watch progress in Foxglove use the layout from
[§"Foxglove panel layout"](#4-foxglove-panel-layout):

- **3D panel** shows the live plan (`/plan`), the full coverage path
  (`/planning/coverage_path`), the costmap halo, and the robot moving
  through the strips.
- **Raw Messages panel** with topic `/control/mission_state` flips
  `WAITING → RUNNING → DONE` as the action progresses.

To run the planner **alone** (visualising the path without driving the
robot), launch only `planning.launch.py` without Nav2 — but you must
add your own static `map → odom` TF or set the 3D panel's Fixed frame
to `odom`, because Nav2's TF will not be there.

Tune the bounds, strip width, or starting corner without rebuilding by
editing `src/window_cleaner_planning/config/planning_params.yaml` and
restarting `planning.launch.py`.

## Running the cleaning controller (Phase 3 Sub-phase D+)

The cleaning controller runs a 4-state machine (IDLE / MOVING / CLEANING /
EMERGENCY) and publishes `/control/vacuum_cmd`, `/control/brush_cmd`,
`/control/state`. It gates on `/control/mission_state` (from `path_follower`),
listens to `/odom` for dirt-proximity matching, and listens to `/robot/scan`
for emergency stops.

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 launch window_cleaner_control control.launch.py
```

Watch transitions in a fifth terminal:

```bash
ros2 topic echo /control/state
```

Expected sequence over a single mission:

```
IDLE       (mission_state == WAITING)
MOVING     (mission_state -> RUNNING, robot far from dirt centres)
CLEANING   (robot pose within 0.20 m of a cached dirt centre — brush ON)
MOVING     (robot leaves the dirt patch)
...
IDLE       (mission_state -> DONE)
```

`EMERGENCY` fires whenever the lidar minimum range falls below `0.08 m`
and clears after the lane stays open for `1.0 s` (hysteresis prevents
flicker). In EMERGENCY the controller zeroes `/cmd_vel` to override Nav2.

## Running the benchmark (Phase 4)

Phase 4 adds the `window_cleaner_evaluation` package: a passive
`metrics_node` (one CSV row per mission), an unattended `run_benchmark.sh`
sweep, and an offline `plot_results.py`.

**Benchmark matrix:** `glass_basic` + `glass_small`, 3 runs each = 6 runs
(plan change 2026-05-16 — `glass_obstacles` reliably ABORTs, a documented
Phase-3 limitation; `glass_large` is geometrically identical to
`glass_basic`). Many runs end ABORTED/partial — that is the honest dataset,
not a failure.

Run the whole sweep with **one** command (one long-lived container; the
script resets Gazebo/DDS between runs by killing processes). It is fully
unattended (~30–50 min on the M4):

```bash
docker compose -f docker/docker-compose.yml run --service-ports --rm \
  --name wc-bench ros2-dev \
  bash -c "source install/setup.bash && \
    bash install/window_cleaner_evaluation/share/window_cleaner_evaluation/scripts/run_benchmark.sh --all --runs 3"
```

Options: `--world glass_basic|glass_small`, `--runs N`, `--timeout S`
(hard wall-clock per run, default 480), `--bag` (record a rosbag on run 1
of each world → `media/bags/`, task 4.6).

Outputs:

* `results/metrics.csv` — one row per run: `timestamp,world,run_index,
  mission_result,coverage_pct,collisions,duration_s,distance_m` (committed).
* `results/benchmark_summary.txt`, `results/logs/<world>_run<N>/*.log` —
  per-process logs for post-hoc ABORT diagnosis (git-ignored).
* `media/plots/*.png` — coverage / duration / collision / summary plots,
  auto-generated at the end (committed).

Single mission, manual (debug): start the normal Phase-3 stack (sim → nav2
→ perception → planning → control) and add `metrics_node`:

```bash
docker compose -f docker/docker-compose.yml exec wc-sim bash -c \
  "source install/setup.bash && \
   ros2 launch window_cleaner_evaluation metrics.launch.py run_id:=1"
```

Regenerate Nav2 maps after editing a world SDF (writes every world's
`.pgm` + `.yaml`; the `glass_basic` grid content is unchanged by design):

```bash
docker compose -f docker/docker-compose.yml exec wc-sim bash -c \
  "python3 src/window_cleaner_bringup/maps/gen_map.py"
```

`nav2.launch.py` now takes additive `odom_offset_x` / `odom_offset_y`
arguments (default `-2.15` / `-1.15`, i.e. unchanged for the 5×3 worlds).
The benchmark passes the per-world SW spawn for `glass_small`, whose 2×1
surface does not fit the hard-coded offset.

Re-plot from an existing CSV at any time:

```bash
docker compose -f docker/docker-compose.yml exec wc-sim bash -c \
  "python3 install/window_cleaner_evaluation/share/window_cleaner_evaluation/scripts/plot_results.py"
```

## Topic cheat-sheet

| Topic | Type | Hz | Direction |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | 1000 | GZ → ROS |
| `/cmd_vel` | `geometry_msgs/Twist` | as-published | ROS → GZ |
| `/odom` | `nav_msgs/Odometry` | 30 | GZ → ROS |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | as-changed | GZ → ROS |
| `/joint_states` | `sensor_msgs/JointState` | high | GZ → ROS |
| `/robot/scan` | `sensor_msgs/LaserScan` | ~9 | GZ → ROS |
| `/robot/camera/image_raw` | `sensor_msgs/Image` | ~22 | GZ → ROS |
| `/robot/camera/camera_info` | `sensor_msgs/CameraInfo` | ~22 | GZ → ROS |
| `/robot_description` | `std_msgs/String` | latched | ROS only |
| `/perception/debug_image` | `sensor_msgs/Image` | ~22 | ROS only |
| `/perception/debug_frame` | `sensor_msgs/Image` | ~22 | ROS only |
| `/perception/debug_dirt` | `sensor_msgs/Image` | ~22 | ROS only |
| `/perception/glass_boundary` | `geometry_msgs/PolygonStamped` | ~22 | ROS only |
| `/perception/dirty_regions` | `geometry_msgs/PoseArray` | ~22 | ROS only |
| `/perception/dirty_areas` | `std_msgs/Float32MultiArray` | ~22 | ROS only |
| `/planning/coverage_path` | `nav_msgs/Path` | latched (1) | ROS only |
| `/map` | `nav_msgs/OccupancyGrid` | latched | ROS only (map_server) |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | 1 | ROS only |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | 2 | ROS only |
| `/goal_pose` | `geometry_msgs/PoseStamped` | as-published | Foxglove → Nav2 |
| `/plan` | `nav_msgs/Path` | per goal | Nav2 → ROS |
| `/control/mission_state` | `std_msgs/String` | 1 + on change | path_follower → ROS |
| `/control/state` | `std_msgs/String` | 10 | cleaning_controller → ROS |
| `/control/vacuum_cmd` | `std_msgs/Bool` | 10 | cleaning_controller → ROS |
| `/control/brush_cmd` | `std_msgs/Bool` | 10 | cleaning_controller → ROS |

## Troubleshooting

| Symptom | Fix |
|---|---|
| Foxglove "Connection failed — rosbridge WebSocket server…" | Wrong connection type. Pick **Foxglove WebSocket**, not Rosbridge. |
| Port 8765 closed on host (`nc -z localhost 8765` → CLOSED) | Sim was started without `--service-ports`. Restart with the flag. |
| `Cannot connect to the Docker daemon` | Docker Desktop is not running. `open -a Docker` and wait. |
| `gz sim ...` command not found | Use `ign gazebo` — Humble pairs with Ignition Gazebo 6 (Fortress). |
| `package 'window_cleaner_...' not found` while xacro runs | `colcon build --symlink-install` first, then `source install/setup.bash`. |
| Camera / lidar topics silent | `ign gazebo` process died — check launch terminal for `Aborted` / `segfault`. |
| `docker exec wc-sim …` returns "No such container" | Sim never started or already stopped. Re-run the bring-up command. |
| 3D panel Fixed-frame dropdown is missing `map` | Nav2 has not yet activated. Wait for `Managed nodes are active`, then in Foxglove disconnect + reconnect to refresh the TF tree. |
| 3D panel topic toggles do nothing (eye icons stay grey) | The topic isn't being published yet. Check `ros2 topic list` in the container. |
| Publish panel button is grey / says "frame_id required" | The JSON's `header.frame_id` is empty. Set it to `"map"`. |
| TF labels cover the whole robot | 3D panel settings → **Scene → Label scale: 0.14**, or toggle **Show labels: Off**. |

## Why this Foxglove workflow exists

XQuartz on Apple Silicon cannot create an OpenGL/GLX context, so Gazebo's
OGRE2 GUI and RViz2 both segfault at window creation. The project therefore
runs Gazebo headless (`-s --headless-rendering`), renders sensors off-screen,
and uses Foxglove Studio (native Mac app, WebSocket protocol) to visualise
ROS topics and publish teleop commands. Full rationale is in
[PROJECT_ROADMAP — AI Notes Phase 0 / Phase 1](../PROJECT_ROADMAP%20%281%29.md).
