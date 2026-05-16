# Known Issues & Limitations

Required by the roadmap's Phase Transition Checklist. These are documented
honestly rather than hidden — the roadmap explicitly notes that an honest
limitations section earns academic credit. None of these are fixed in
Phase 4: the Phase-4 instruction is *"be conservative, only add metric
collection and world variants"*, so the Phase-3 navigation/control code is
frozen and these are **measured and reported**, not patched.

## Navigation / coverage

1. **Partial coverage on the basic world.** The frozen Phase-3 stack covers
   ~4–6 of 9 strips per run (~278 s for the one full `SUCCESS`); most runs
   end `ABORTED` on a U-turn after Nav2 recovery thrash. This is the
   expected behaviour `metrics_node` records; `mission_result` and the
   honest `coverage_pct` denominator (real surface, not the planner inset)
   make it analysable. No frame collisions occur.

2. **Obstacle world does not complete.** `glass_obstacles` reliably
   `ABORT`s: the coverage planner's obstacle filter only *drops* blocked
   waypoints, it does not inject a detour, so the straight segment between
   two kept waypoints still crosses a mullion and Regulated Pure Pursuit
   reports "collision ahead" and stops (patience exceeded → ABORTED). The
   robot never physically hits anything. A planner-side detour (or a
   navigation re-design) is future work and out of Phase-4 scope — which is
   why `glass_obstacles` is **excluded from the benchmark matrix** (plan
   change 2026-05-16; the static occupancy map also does not paint the
   interior mullions).

3. **U-turn overshoot / drift.** At strip ends the robot sometimes
   overshoots the 180° turn. Root cause is the low M4 real-time factor
   combined with the ~6 Hz lidar feeding a controller that would run at
   10 Hz on real hardware; mitigated (not eliminated) by
   `rotate_to_heading_min_angle 0.30` and the dense waypoint trail.

## Platform (Apple Silicon M4)

4. **Low Gazebo RTF.** Headless Ignition on M4 runs well below 1.0 RTF, so
   the controller frequency was reduced to 5 Hz and `desired_linear_vel` to
   0.15 m/s. On real hardware these would be 10 Hz / higher. All Phase-4
   durations are therefore reported in **sim-clock seconds**, not
   wall-clock.

5. **No GUI / RViz.** XQuartz cannot create an OpenGL/GLX context on Apple
   Silicon, so Gazebo runs headless and visualisation is via Foxglove
   Studio over WebSocket. This shaped the whole workflow (see
   [RUNNING.md](RUNNING.md) "Why this Foxglove workflow exists").

## Sensor / model

6. **Lidar height ≈ wall height.** Both the lidar and the frame walls are
   at ~0.10 m, so some rays graze the wall top and produce irregular
   costmap speckle. A URDF fix (raise the lidar joint origin by ~0.05 m)
   would help but is a perception/URDF change, out of Phase-4 conservative
   scope. The metrics collision threshold (0.05 m) is tight enough that
   only genuine near-contact is counted, and the 1.0 s clear-hysteresis
   prevents speckle from inflating the collision count.

7. **Foxglove lidar decay is long** — the 3-D panel keeps the lidar trail
   visible for a while. Purely cosmetic; does not affect Nav2 or metrics.

## 2-D abstraction (by design, not a defect)

8. The robot does not climb a real vertical pane; the world is
   zero-gravity with a planar glass surface ("vertical-surface kinematics
   modelled via a 2-D planar abstraction"). Zero friction causes a small
   linear-distance overshoot. This is an intentional modelling choice
   defended in the report — see [architecture.md](architecture.md).

## Phase-4 consistency caveat

9. **Bounds drift risk.** The metrics coverage bounds, the
   `coverage_planner` inset bounds, and the Nav2 map come from three files
   in two packages (no shared source — a shared file was rejected as too
   invasive for the conservative Phase-4 scope). They are kept consistent
   via the canonical
   [`worlds_manifest.yaml`](../src/window_cleaner_evaluation/config/worlds_manifest.yaml)
   that `run_benchmark.sh` reads, plus cross-reference comments. If a
   world's geometry is edited, all of: the SDF surface size,
   `planning_params*.yaml`, the manifest, and a `gen_map.py` regeneration
   must be updated together.
