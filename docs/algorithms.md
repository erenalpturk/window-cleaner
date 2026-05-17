# Algorithms

Final parameter values are from the roadmap AI-Notes (Phase 2/3) and the
config YAMLs; this document describes the logic, not a re-derivation.

## 1. Boustrophedon coverage planner

`src/window_cleaner_planning/window_cleaner_planning/coverage_planner.py`
(`compute_path`). Input: the static glass interior bounds (inset by
`strip_width/2` so strip endpoints clear the walls). Output: a latched
`nav_msgs/Path` of densely-sampled waypoints, consumed by the default
`waypoint_follower` (§4) — or, in the retained Nav2 mode, sent to Nav2 as
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
RPP halts. See [known_issues.md](known_issues.md).

## 2. Glass-frame detection (perception)

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

## 3. Dirty-region segmentation (perception)

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

## 4. Driving the coverage path

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
academically reportable. The benchmark in [results.md](results.md) predates
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

## 5. Cleaning controller state machine

`cleaning_controller.py`, 10 Hz tick, gated by `/control/mission_state`:

| State | Condition | vacuum | brush |
|---|---|---|---|
| `IDLE` | mission not RUNNING | off | off |
| `MOVING` | RUNNING, not over dirt | on | off |
| `CLEANING` | within 0.20 m of a dirt centre | on | on |
| `EMERGENCY` | lidar min range < 0.08 m | on | off + `/cmd_vel`=0 |

`EMERGENCY` clears only after the lane stays open for `1.0 s` (hysteresis
prevents flicker / chattering between Nav2 and the override).

## 6. Coverage / collision metrics (Phase 4)

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
