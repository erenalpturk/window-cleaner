# Architecture

## System overview

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
   │ coverage_planner : boustrophedon → /planning/coverage_path       │
   │ path_follower    : Path → Nav2 NavigateThroughPoses;             │
   │                    publishes /control/mission_state              │
   └──────────────────────────────────────────────────────────────────┘
                           ▼ action: navigate_through_poses
   ┌───────────── Nav2 (window_cleaner_bringup) ──────────────────────┐
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

## The 2-D planar abstraction (defended)

A real window cleaner climbs a vertical pane. Simulating climbing dynamics
(suction adhesion, gravity along the surface, slip) is out of scope and not
the assignment's point. Instead the world sets `<gravity>0 0 0</gravity>`
and treats the glass as a horizontal plane the robot drives on with a
differential drive. This is **"vertical-surface kinematics modelled via a
2-D planar abstraction"** and is standard in window-cleaning-robot
literature. Consequences, all intentional:

* No AMCL/localisation: Gazebo's DiffDrive plugin makes `/odom` ground
  truth. Nav2 runs with a **static `map → odom` TF** equal to the spawn
  offset, so map-frame coordinates equal world coordinates. (`nav2.launch.py`
  exposes this as additive `odom_offset_x/y` arguments — default
  `-2.15/-1.15`, overridden per world by the benchmark.)
* Zero friction: linear motion slightly overshoots commanded distance
  (documented in roadmap AI-Notes Phase 1). Velocities are kept low
  (`0.15 m/s`).
* The vacuum/brush are modelled as boolean state, not fluid physics.

## Topic & action contract (do not break)

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
| `navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | path_follower → Nav2 |
| `/control/mission_state` | `std_msgs/String` (VOLATILE, 1 Hz re-pub) | path_follower → control, metrics |
| `/control/state` | `std_msgs/String` | cleaning_controller (debug) |
| `/control/vacuum_cmd`, `/control/brush_cmd` | `std_msgs/Bool` | cleaning_controller |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / control → Gazebo |

`mission_state` values: `WAITING → RUNNING → DONE | ABORTED`. This is the
mission lifecycle signal `metrics_node` keys off (start the duration timer
on the first `RUNNING`, finalise on the terminal state).

## Per-world single source of truth (Phase 4)

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

## Build / runtime environment

ROS 2 Humble + Gazebo Fortress (Ignition branding: `ign gazebo`,
`ignition::gazebo::systems::*`, `ignition.msgs.*`). `rmw_cyclonedds_cpp`
(FastDDS has macOS-Docker networking issues). `numpy<2` pinned (cv_bridge
ABI). Headless Gazebo + Foxglove instead of XQuartz/RViz (Apple Silicon GLX
broken). See [RUNNING.md](RUNNING.md) and roadmap AI-Notes for the full
rationale.
