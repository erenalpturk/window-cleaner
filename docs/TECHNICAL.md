# Technical Reference

System architecture, algorithms, and honest limitations of the
autonomous window-cleaning robot simulation. Bring-up instructions live
in [RUNNING.md](RUNNING.md); benchmark numbers in [RESULTS.md](RESULTS.md).

---

## 1. System Architecture

### 1.1 Pipeline overview

The simulation is a five-layer pipeline plus a passive evaluation observer.
Each layer communicates only through the ROS 2 topic/action contract below;
layers can be started and tested independently (this is how the project was
built phase by phase).

```
                ┌─────────────────────────────────────────────┐
                │  Gazebo Fortress (Ignition, headless on M4)  │
                │  zero-gravity 2-D planar glass world         │
                └─────────────────────────────────────────────┘
   /robot/camera/image_raw │ /robot/scan │ /odom │ /clock │ /tf │ /tf_static
                           ▼
   ┌───────────── Perception (window_cleaner_perception) ─────────────┐
   │ camera_node    : republishes camera + FPS metrics                │
   │ frame_detector : Gray→Canny→Hough → /perception/glass_boundary   │
   │ dirt_segmenter : HSV→morph→contours → /perception/dirty_regions  │
   └──────────────────────────────────────────────────────────────────┘
                           ▼ /perception/glass_boundary
   ┌───────────── Planning (window_cleaner_planning) ─────────────────┐
   │ coverage_planner  : boustrophedon → /planning/coverage_path      │
   │ waypoint_follower : DEFAULT — deterministic turn-then-drive on    │
   │   ground-truth /odom → /cmd_vel; publishes /control/mission_state │
   │ path_follower     : ALT mode — Path → Nav2 NavigateThroughPoses   │
   └──────────────────────────────────────────────────────────────────┘
              default ▼ /cmd_vel        alt ▼ action: navigate_through_poses
   ┌───────────── Nav2 (window_cleaner_bringup) — ALT mode only ──────┐
   │ map_server (static occupancy) · NavFn global planner ·           │
   │ Regulated Pure Pursuit local · bt_navigator · lifecycle_manager  │
   │ static TF map→odom (no AMCL: odom is ground truth — see below)   │
   └──────────────────────────────────────────────────────────────────┘
                           ▼ /cmd_vel
   ┌───────────── Control (window_cleaner_control) ───────────────────┐
   │ cleaning_controller : IDLE/MOVING/CLEANING/EMERGENCY state        │
   │   machine; /control/{vacuum,brush}_cmd; emergency /cmd_vel        │
   │   override with 1.0 s hysteresis                                  │
   └──────────────────────────────────────────────────────────────────┘
                           ▼ (passive observation, Phase 4)
   ┌───────────── Evaluation (window_cleaner_evaluation) ─────────────┐
   │ metrics_node : subscribe-only; coverage %, collisions, duration, │
   │   distance → results/metrics.csv (one row per mission)           │
   └──────────────────────────────────────────────────────────────────┘
```

### 1.2 Driving controller: default vs. Nav2 (plan change 2026-05-16)

Two interchangeable Planning drivers consume the same
`/planning/coverage_path` and publish the same `/control/mission_state`
(WAITING → RUNNING → DONE) + `/cmd_vel`, so Perception, Control and
Evaluation are identical either way:

* **`waypoint_follower` — default.** Deterministic turn-then-drive: rotate
  in place toward the next waypoint until the heading error is small, drive
  straight to the position tolerance, repeat — directly on ground-truth
  `/odom`, straight to `/cmd_vel`. No costmap, carrot, behaviour tree or
  replanning. On the gravity-free planar abstraction with ground-truth
  odometry, Nav2's reactive stack was overkill and the sole source of the
  observed path-following instability (roadmap Phase 3 AI Notes, plan
  change). Strip-to-strip transitions are a pure perpendicular hop, so it
  naturally performs the clean two-stage U-turn.
* **Nav2 (`path_follower` + `nav2.launch.py` + custom BT) — retained
  alternative/reference mode.** NavFn global + Regulated Pure Pursuit local
  over a static occupancy map. The descriptions in [§2 Algorithms](#2-algorithms)
  §2.4 still apply to this mode; it is kept (not deleted) and is academically
  reportable.

The benchmark in [RESULTS.md](RESULTS.md) predates the plan change and
reflects the Nav2 reference flow (stated honestly there).

### 1.3 The 2-D planar abstraction (defended)

A real window cleaner climbs a vertical pane. Simulating climbing dynamics
(suction adhesion, gravity along the surface, slip) is out of scope and not
the assignment's point. Instead the world sets `<gravity>0 0 0</gravity>`
and treats the glass as a horizontal plane the robot drives on with a
differential drive. This is **"vertical-surface kinematics modelled via a
2-D planar abstraction"** and is standard in window-cleaning-robot
literature. Consequences, all intentional:

* No AMCL/localisation: Gazebo's DiffDrive plugin makes `/odom` ground
  truth. A **static `map → odom` TF** equal to the spawn offset keeps
  map-frame coordinates equal to world coordinates. In the default mode
  `sim.launch.py` publishes it (`map_odom_tf:=true`); in the Nav2 mode
  `nav2.launch.py` owns it instead (launch sim with `map_odom_tf:=false`),
  exposed as additive `odom_offset_x/y` arguments — default `-2.15/-1.15`,
  overridden per world by the benchmark.
* Zero friction: linear motion slightly overshoots commanded distance
  (documented in roadmap AI-Notes Phase 1). Velocities are kept low
  (`0.15 m/s`).
* The vacuum/brush are modelled as boolean state, not fluid physics.

### 1.4 Topic & action contract (do not break)

| Topic / Action | Type | Producer → Consumer |
|---|---|---|
| `/robot/camera/image_raw` | `sensor_msgs/Image` | Gazebo → perception |
| `/robot/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo → perception |
| `/robot/scan` | `sensor_msgs/LaserScan` (BEST_EFFORT) | Gazebo → control, metrics |
| `/odom` | `nav_msgs/Odometry` | Gazebo → Nav2, control, metrics |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → all (`use_sim_time`) |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Gazebo + static publishers |
| `/perception/glass_boundary` | `geometry_msgs/PolygonStamped` | frame_detector → coverage_planner |
| `/perception/dirty_regions` | `geometry_msgs/PoseArray` | dirt_segmenter → control |
| `/perception/dirty_areas` | `std_msgs/Float32MultiArray` | dirt_segmenter |
| `/perception/debug_*` | `sensor_msgs/Image` | perception → Foxglove |
| `/planning/coverage_path` | `nav_msgs/Path` (TRANSIENT_LOCAL) | coverage_planner → path_follower |
| `navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | path_follower → Nav2 (alt mode only) |
| `/control/mission_state` | `std_msgs/String` (VOLATILE, 1 Hz re-pub) | waypoint_follower (default) / path_follower (Nav2 mode) → control, metrics |
| `/control/state` | `std_msgs/String` | cleaning_controller (debug) |
| `/control/vacuum_cmd`, `/control/brush_cmd` | `std_msgs/Bool` | cleaning_controller |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / control → Gazebo |

`mission_state` values: `WAITING → RUNNING → DONE | ABORTED`. This is the
mission lifecycle signal `metrics_node` keys off (start the duration timer
on the first `RUNNING`, finalise on the terminal state).

### 1.5 Per-world single source of truth (Phase 4)

A world's geometry must agree across four consumers: the coverage_planner
bounds, the metrics coverage denominator, the Nav2 occupancy map, and the
spawn / `map→odom` offset. The Phase-3 nodes keep their own parameter files
(conservative — no refactor), and
[`src/window_cleaner_evaluation/config/worlds_manifest.yaml`](../src/window_cleaner_evaluation/config/worlds_manifest.yaml)
is the canonical table the benchmark reads to keep them consistent.

**Invariant to preserve when editing a world:** `spawn == map→odom offset`,
and `spawn == planner_inset_min + strip_width/2` (SW corner). The metrics
`glass_*` bounds are the **real glass surface**, deliberately *wider* than
the coverage_planner inset — so an uncleaned wall-adjacent strip lowers the
reported coverage honestly rather than being hidden by a shrunken
denominator. Keep `planning_params_small.yaml` bounds and the manifest in
sync (cross-referenced in both files).

### 1.6 Build / runtime environment

ROS 2 Humble + Gazebo Fortress (Ignition branding: `ign gazebo`,
`ignition::gazebo::systems::*`, `ignition.msgs.*`). `rmw_cyclonedds_cpp`
(FastDDS has macOS-Docker networking issues). `numpy<2` pinned (cv_bridge
ABI). Headless Gazebo + Foxglove instead of XQuartz/RViz (Apple Silicon GLX
broken). See [RUNNING.md](RUNNING.md) and roadmap AI-Notes for the full
rationale.

---

## 2. Algorithms

Final parameter values are from the roadmap AI-Notes (Phase 2/3) and the
config YAMLs; this section describes the logic, not a re-derivation.

### 2.1 Boustrophedon coverage planner

`src/window_cleaner_planning/window_cleaner_planning/coverage_planner.py`
(`compute_path`). Input: the static glass interior bounds (inset by
`strip_width/2` so strip endpoints clear the walls). Output: a latched
`nav_msgs/Path` of densely-sampled waypoints, consumed by the default
`waypoint_follower` (§2.4) — or, in the retained Nav2 mode, sent to Nav2 as
one `NavigateThroughPoses` goal.

```
margin   = strip_width / 2
y_first  = glass_min_y + margin
y_last   = glass_max_y - margin
span     = y_last - y_first

if span < strip_width:                 # area narrower than one strip
    ys = [ (y_first + y_last) / 2 ]    # single centreline pass (fallback)
else:
    n_strips = ceil(span / strip_width) + 1
    ys       = n_strips points evenly spaced in [y_first, y_last]

x_low  = glass_min_x + margin
x_high = glass_max_x - margin
if start_corner in (NW, NE): reverse(ys)      # start row
east_first = start_corner in (SE, NE)

for i, y in enumerate(ys):                     # snake / serpentine
    (x_a, x_b) = (x_low, x_high) if (i even) == east_first else (x_high, x_low)
    emit interpolated waypoints x_a → x_b at y, spacing = waypoint_spacing
    # n_intervals = max(1, ceil(strip_length / waypoint_spacing))
    # heading (yaw) faces the travel direction; quaternion via yaw_to_quaternion

# Obstacle handling (glass_obstacles only): any waypoint whose centre is
# inside an obstacle AABB inflated by `obstacle_inflation` is dropped.
```

Final values: `strip_width = 0.30 m` (vacuum-pad 0.24 m + 0.06 m safety),
`waypoint_spacing = 0.50 m` (dense carrot trail so RPP tracks the strip),
`start_corner = SW`. Basic world ⇒ 9 strips / ~90 waypoints; small world ⇒
narrow-span centreline fallback (~2 strips). Unit-tested in
`src/window_cleaner_planning/test/test_coverage_planner.py`.

**Known gap (Phase 3 → documented):** the obstacle filter only *removes*
blocked waypoints; it does not inject a detour, so on `glass_obstacles` the
straight segment between two kept waypoints still crosses an obstacle and
RPP halts. See [§3 Known Issues](#3-known-issues--limitations).

### 2.2 Glass-frame detection (perception)

`frame_detector.py`:

```
BGR → Gray
GaussianBlur(kernel = blur_kernel)
edges = Canny(canny_low=50, canny_high=150)
lines = HoughLinesP(min_length=50, max_gap=20)
group lines by angle (≈horizontal vs ≈vertical), pick the longest 4
→ 4-corner polygon → /perception/glass_boundary (PolygonStamped)
+ /perception/debug_frame (Hough lines green, boundary blue)
```

All thresholds are loaded from `config/perception_params.yaml` so they are
tunable in the field (roadmap requirement). Detection accuracy ≈70–80 % at
the start pose; all four edges are captured as the robot moves (roadmap
AI-Notes Phase 2).

### 2.3 Dirty-region segmentation (perception)

`dirt_segmenter.py`:

```
roi_mask: zero the top roi_top_fraction (0.35) of the image — the frame
          edge falls there; real dirt is mid/lower
BGR → HSV
mask = inRange(HSV, lower, upper)            # dirt H 70–130, S 25–255, V 100–230
mask = mask AND roi_mask
mask = morphologyEx(mask, MORPH_CLOSE, ellipse(morph_k))   # de-speckle
contours = findContours(mask, RETR_EXTERNAL)
for c in contours:
    if contourArea(c) < min_area (200 px): skip      # noise filter
    centroid = moments(c) → (cx, cy)
→ /perception/dirty_regions (PoseArray) + /perception/dirty_areas
  (Float32MultiArray) + /perception/debug_dirt (contours green, centres red)
```

The ROI mask + the `V ≤ 230` ceiling were the two fixes that removed the
bright glass-edge false positives and flicker (roadmap AI-Notes Phase 2:
false-positive = 0 across 5 poses after tuning). Standard messages chosen
over a custom type (PoseArray + Float32MultiArray).

### 2.4 Driving the coverage path

**Default — deterministic `waypoint_follower`** (plan change 2026-05-16,
roadmap Phase 3 AI Notes). `waypoint_follower.py` consumes the same latched
`/planning/coverage_path` and runs a classic turn-then-drive loop on
ground-truth `/odom`:

```
for each waypoint:
    heading_err = atan2(dy, dx) - yaw
    if |heading_err| > turn_threshold (0.15 rad ≈ 8.6°):
        rotate in place        # w = clamp(heading_kp · heading_err)
    else:
        drive straight         # v = clamp(linear_kp · dist),
                               # small drive_heading_kp yaw correction
    advance when within position_tolerance
publish /control/mission_state WAITING → RUNNING → DONE; → /cmd_vel
```

No costmap / carrot / behaviour tree / replanning, so there is almost no
failure mode. Defaults mirror the old Nav2 limits (`max_linear_vel 0.15`,
`max_angular_vel 1.0`), live-tunable via `--ros-args -p`. Consecutive
strips share an x endpoint, so each transition is a pure perpendicular hop
→ the clean two-stage U-turn falls out for free, no connector waypoints.
Rationale: the gravity-free planar abstraction with ground-truth odometry
made Nav2's reactive stack overkill and the sole source of path-following
instability.

**Alternative / reference mode — Nav2.** Retained, not deleted; still
academically reportable. The benchmark in [RESULTS.md](RESULTS.md) predates
the plan change and reflects this Nav2 flow.

Global: **NavFn** on a static occupancy grid (frame walls = occupied;
`obstacle_layer` removed from the global costmap so the 6 Hz lidar can't
re-mark the static walls inconsistently — Phase-3 tuning iteration 3).
Local: **Regulated Pure Pursuit** — a geometric controller suits the
zero-friction planar model better than DWB roll-outs. Final tuning:
`inflation_radius 0.15`, `lookahead_dist 0.20`,
`rotate_to_heading_min_angle 0.30`, `controller_frequency 5 Hz` (matches
the measured ~6 Hz lidar on M4), `desired_linear_vel 0.15 m/s`. Maps are
generated by `src/window_cleaner_bringup/maps/gen_map.py` (frame-wall AABBs
derived from glass dimensions; one `.pgm` + `.yaml` per world).

### 2.5 Cleaning controller state machine

`cleaning_controller.py`, 10 Hz tick, gated by `/control/mission_state`:

| State | Condition | vacuum | brush |
|---|---|---|---|
| `IDLE` | mission not RUNNING | off | off |
| `MOVING` | RUNNING, not over dirt | on | off |
| `CLEANING` | within 0.20 m of a dirt centre | on | on |
| `EMERGENCY` | lidar min range < 0.08 m | on | off + `/cmd_vel`=0 |

`EMERGENCY` clears only after the lane stays open for `1.0 s` (hysteresis
prevents flicker / chattering between Nav2 and the override).

### 2.6 Coverage / collision metrics (Phase 4)

`src/window_cleaner_evaluation/window_cleaner_evaluation/metrics_node.py`
— passive (subscribe-only). All maths is in unit-tested pure functions
(`test/test_metrics.py`).

* **Coverage %**: the real glass surface is discretised into
  `cell_size = 0.05 m` cells. While the mission is `RUNNING`, every `/odom`
  pose stamps a disc of `footprint_radius = 0.12 m` (the vacuum-pad radius,
  *not* the 0.18 m chassis radius — coverage means the cleaning head passed
  over the cell) into a boolean grid. `coverage_pct = 100 · visited /
  countable`, denominator over the **real** surface (so an uncleaned
  wall-adjacent strip lowers the number honestly). Obstacle AABBs, if any,
  are excluded from the denominator.
* **Collisions**: an edge-counted FSM. A *period* starts when
  `min(/robot/scan) < 0.05 m`; it ends only after the lane stays clear for
  `1.0 s` (same hysteresis idea as the controller), so one sustained graze
  is one event and threshold chatter does not inflate the count.
* **Duration**: sim-clock seconds between the first `RUNNING` and the
  terminal `DONE`/`ABORTED`/`TIMEOUT` (sim clock because RTF < 1 on M4 —
  wall-clock would be wrong).
* **Distance**: cumulative `/odom` position deltas while `RUNNING`; a
  single step > 0.5 m (teleport/reset artefact) is rejected.

A sim watchdog finalises a stuck mission as `TIMEOUT`; `run_benchmark.sh`
adds an outer wall-clock backstop, so an unattended sweep always produces a
complete `metrics.csv`.

---

## 3. Known Issues & Limitations

Required by the roadmap's Phase Transition Checklist. These are documented
honestly rather than hidden — the roadmap explicitly notes that an honest
limitations section earns academic credit. None of these are fixed in
Phase 4: the Phase-4 instruction is *"be conservative, only add metric
collection and world variants"*, so the Phase-3 navigation/control code is
frozen and these are **measured and reported**, not patched.

> **Scope note (plan change 2026-05-16).** Items that name Nav2 internals
> — §3.1 "Nav2 recovery thrash", §3.2 "Regulated Pure Pursuit reports
> collision ahead", and the controller-frequency framing of §3.3 — describe
> the **Nav2 reference flow**, which is the mode the committed benchmark
> ([RESULTS.md](RESULTS.md)) was run on. The current **default** driver is
> the deterministic `waypoint_follower` (no costmap/RPP/BT). Its behaviour
> has not been re-benchmarked, so these limitations are reported as-measured
> for the Nav2 mode and **not** re-characterised for the default driver
> here (no data is invented). The 2-D-abstraction, RTF and sensor items
> (§3.4–§3.8) are mode-independent and still apply.

### Navigation / coverage

**3.1 Partial coverage on the basic world.** The frozen Phase-3 stack
covers ~4–6 of 9 strips per run (~278 s for the one full `SUCCESS`); most
runs end `ABORTED` on a U-turn after Nav2 recovery thrash. This is the
expected behaviour `metrics_node` records; `mission_result` and the honest
`coverage_pct` denominator (real surface, not the planner inset) make it
analysable. No frame collisions occur.

**3.2 Obstacle world does not complete.** `glass_obstacles` reliably
`ABORT`s: the coverage planner's obstacle filter only *drops* blocked
waypoints, it does not inject a detour, so the straight segment between
two kept waypoints still crosses a mullion and Regulated Pure Pursuit
reports "collision ahead" and stops (patience exceeded → ABORTED). The
robot never physically hits anything. A planner-side detour (or a
navigation re-design) is future work and out of Phase-4 scope — which is
why `glass_obstacles` is **excluded from the benchmark matrix** (plan
change 2026-05-16; the static occupancy map also does not paint the
interior mullions).

**3.3 U-turn overshoot / drift.** At strip ends the robot sometimes
overshoots the 180° turn. Root cause is the low M4 real-time factor
combined with the ~6 Hz lidar feeding a controller that would run at
10 Hz on real hardware; mitigated (not eliminated) by
`rotate_to_heading_min_angle 0.30` and the dense waypoint trail.

### Platform (Apple Silicon M4)

**3.4 Low Gazebo RTF.** Headless Ignition on M4 runs well below 1.0 RTF,
so the controller frequency was reduced to 5 Hz and `desired_linear_vel`
to 0.15 m/s. On real hardware these would be 10 Hz / higher. All Phase-4
durations are therefore reported in **sim-clock seconds**, not wall-clock.

**3.5 No GUI / RViz.** XQuartz cannot create an OpenGL/GLX context on
Apple Silicon, so Gazebo runs headless and visualisation is via Foxglove
Studio over WebSocket. This shaped the whole workflow (see
[RUNNING.md](RUNNING.md) "Why this Foxglove workflow exists").

### Sensor / model

**3.6 Lidar height ≈ wall height.** Both the lidar and the frame walls
are at ~0.10 m, so some rays graze the wall top and produce irregular
costmap speckle. A URDF fix (raise the lidar joint origin by ~0.05 m)
would help but is a perception/URDF change, out of Phase-4 conservative
scope. The metrics collision threshold (0.05 m) is tight enough that
only genuine near-contact is counted, and the 1.0 s clear-hysteresis
prevents speckle from inflating the collision count.

**3.7 Foxglove lidar decay is long** — the 3-D panel keeps the lidar
trail visible for a while. Purely cosmetic; does not affect Nav2 or
metrics.

### 2-D abstraction (by design, not a defect)

**3.8** The robot does not climb a real vertical pane; the world is
zero-gravity with a planar glass surface ("vertical-surface kinematics
modelled via a 2-D planar abstraction"). Zero friction causes a small
linear-distance overshoot. This is an intentional modelling choice
defended in the report — see [§1.3](#13-the-2-d-planar-abstraction-defended).

### Phase-4 consistency caveat

**3.9 Bounds drift risk.** The metrics coverage bounds, the
`coverage_planner` inset bounds, and the Nav2 map come from three files
in two packages (no shared source — a shared file was rejected as too
invasive for the conservative Phase-4 scope). They are kept consistent
via the canonical
[`worlds_manifest.yaml`](../src/window_cleaner_evaluation/config/worlds_manifest.yaml)
that `run_benchmark.sh` reads, plus cross-reference comments. If a
world's geometry is edited, all of: the SDF surface size,
`planning_params*.yaml`, the manifest, and a `gen_map.py` regeneration
must be updated together.
