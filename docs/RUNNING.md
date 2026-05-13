# Running the Simulation

How to bring the autonomous window-cleaning robot simulation up and down after
the initial Phase 1 setup is complete.

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
2. `Open Connection` → **Foxglove WebSocket** (not Rosbridge — they are different protocols).
3. URL: `ws://localhost:8765` → **Open**.

If panels are empty, set up the default layout:

| Panel | Setting |
|---|---|
| 3D | Fixed frame `odom`. Enable topics `/robot_description`, `/robot/scan`, `/tf`. |
| Image | Topic `/robot/camera/image_raw`. |
| Teleop | Topic `/cmd_vel`, publish rate `10`. |

Save the layout (`Layout → Save` in Foxglove) so it auto-loads next time.

### 4. Teleoperation

**Option A (recommended): Foxglove Teleop panel.** Arrow buttons publish `/cmd_vel` directly; no extra terminal needed.

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

Useful perception debug topics (add as Image panels in Foxglove):

| Topic | Content |
|---|---|
| `/perception/debug_image` | Raw camera feed with frame counter overlay |
| `/perception/debug_frame` | Edge detection + detected glass boundary (blue rect) |
| `/perception/debug_dirt` | Dirty regions outlined in green |
| `/perception/glass_boundary` | `PolygonStamped` — glass corners in image coords |
| `/perception/dirty_regions` | `PoseArray` — dirty region centers |
| `/perception/dirty_areas` | `Float32MultiArray` — pixel areas per region |

Tune detection thresholds without rebuilding by editing
`src/window_cleaner_perception/config/perception_params.yaml` and restarting
`perception.launch.py`.

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

## Why this Foxglove workflow exists

XQuartz on Apple Silicon cannot create an OpenGL/GLX context, so Gazebo's
OGRE2 GUI and RViz2 both segfault at window creation. The project therefore
runs Gazebo headless (`-s --headless-rendering`), renders sensors off-screen,
and uses Foxglove Studio (native Mac app, WebSocket protocol) to visualise
ROS topics and publish teleop commands. Full rationale is in
[PROJECT_ROADMAP — AI Notes Phase 0 / Phase 1](../PROJECT_ROADMAP%20%281%29.md).
