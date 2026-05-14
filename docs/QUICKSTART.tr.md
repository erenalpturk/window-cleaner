# Hızlı Başlangıç

Sadece komutlar. Detay için: [RUNNING.tr.md](RUNNING.tr.md).

## 0. Tek seferlik

```bash
open -a Docker
cd /Users/erenalpturk/Desktop/Projects/Robotic
docker compose -f docker/docker-compose.yml build
```

## 1. Sim'i başlat (Terminal 1)

```bash
cd /Users/erenalpturk/Desktop/Projects/Robotic
docker compose -f docker/docker-compose.yml run \
  --service-ports --rm --name wc-sim ros2-dev \
  bash -c "source install/setup.bash && \
           ros2 launch window_cleaner_bringup sim.launch.py \
           rviz:=false gui:=false foxglove:=true"
```

`Server listening on port 8765` mesajını bekle.

## 2. Foxglove'u bağla

Foxglove Studio → **Open Connection** → **Foxglove WebSocket** → `ws://localhost:8765`.

## 2a. Panel düzeni (ilk seferinde)

Foxglove boş açılır. Panel eklemek için tab bar'daki **+** ikonuna bas → panel
türünü seç. Ekranı 2×2'ye bölmek için bir panele sağ tıkla → **Split panel
right** / **Split panel down**.

**Panel 1 — 3D** (sol-üst, robotu/dünyayı gör)

1. Panele tıkla → sağ-üst **⚙ (dişli)** ikonu → ayarlar solda açılır.
2. **Frame**:
   - Fixed frame: `map` (Nav2 yoksa `odom`)
   - Display frame: `base_link`
   - Follow mode: `Pose (position + attitude)`
3. **Scene → Label scale**: `0.14`
4. Sol kenardaki **Topics** listesinde göz ikonuyla AÇ:
   - `/robot_description`, `/tf`, `/robot/scan`
   - `/map`, `/global_costmap/costmap` (Nav2 çalışınca)
   - `/planning/coverage_path` (Line, width `0.05`), `/plan` (Line, width `0.03`)

> Nav2 / planning'i Foxglove bağlandıktan **sonra** başlattıysan, yeni topic'ler
> (`/map`, `/tf`, `/global_costmap/costmap`) listede görünmez. **Disconnect →
> Open Connection** ile yeniden bağlan, listede çıkarlar.

**Panel 2 — Image** (sağ-üst, kamera)

1. Panel ekle → **Image**.
2. **⚙** → **Topic**: `/robot/camera/image_raw`
   (perception için: `/perception/debug_frame` veya `/perception/debug_dirt`)

**Panel 3 — Teleop** (sol-alt, manuel sürüş)

1. Panel ekle → **Teleop**.
2. **⚙**:
   - Topic: `/cmd_vel`
   - Publish rate: `10`
   - Linear x max: `0.20`
   - Angular z max: `1.0`

**Panel 4 — Raw Messages** (sağ-alt, mission state)

1. Panel ekle → **Raw Messages**.
2. **⚙ → Topic**: `/control/mission_state`

**Kaydet:** üst menü **Layout → Save layout as…** → `wc-default`. Sonraki
oturumda **Layout → Open** ile geri yükle.

## 3. Perception (Terminal 2)

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 launch window_cleaner_perception perception.launch.py
```

## 4. Nav2 (Terminal 3)

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 launch window_cleaner_bringup nav2.launch.py
```

`Managed nodes are active` mesajını bekle.

## 5. Planning — otonom kaplama (Terminal 4)

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 launch window_cleaner_planning planning.launch.py
```

## Kod değişikliğinden sonra rebuild

```bash
docker compose -f docker/docker-compose.yml run --rm ros2-dev \
  bash -c "colcon build --symlink-install"
```

## Kapatma

Terminal 1'de **Ctrl+C**. Ya da:

```bash
docker stop wc-sim
```






# 2. Build
docker compose -f docker/docker-compose.yml run --rm --service-ports ros2-dev bash -c \
  "cd /workspace && colcon build --symlink-install --packages-select window_cleaner_bringup"

# 3. Terminal 2: sim + nav2 (önceki gibi sleep 10 zinciriyle)

docker compose -f docker/docker-compose.yml run --rm --service-ports ros2-dev bash -c "
  source /opt/ros/humble/setup.bash &&
  source /workspace/install/setup.bash &&
  ros2 launch window_cleaner_bringup sim.launch.py 2>&1 | tee /workspace/sim.log &
  sleep 10 &&
  ros2 launch window_cleaner_bringup nav2.launch.py 2>&1 | tee /workspace/nav2.log &
  wait"


  docker compose -f docker/docker-compose.yml run --rm --service-ports ros2-dev bash -c "
  source /opt/ros/humble/setup.bash &&
  source /workspace/install/setup.bash &&
  ros2 launch window_cleaner_bringup sim.launch.py world:=/workspace/install/window_cleaner_worlds/share/window_cleaner_worlds/worlds/glass_obstacles.sdf 2>&1 | tee /workspace/sim.log &
  sleep 10 &&
  ros2 launch window_cleaner_bringup nav2.launch.py 2>&1 | tee /workspace/nav2.log &
  wait
"



# 4. Terminal 3: coverage_planner

docker compose -f docker/docker-compose.yml exec ros2-dev bash -c "
  source /opt/ros/humble/setup.bash &&
  source /workspace/install/setup.bash &&
  ros2 run window_cleaner_planning coverage_planner --ros-args --params-file /workspace/install/window_cleaner_planning/share/window_cleaner_planning/config/planning_params.yaml"

docker compose -f docker/docker-compose.yml exec ros2-dev bash -c "
  source /opt/ros/humble/setup.bash &&
  source /workspace/install/setup.bash &&
  ros2 run window_cleaner_planning coverage_planner --ros-args --params-file /workspace/install/window_cleaner_planning/share/window_cleaner_planning/config/planning_params_obstacles.yaml
"


# 5. Terminal 4: path_follower

docker compose -f docker/docker-compose.yml exec ros2-dev bash -c "
  source /opt/ros/humble/setup.bash &&
  source /workspace/install/setup.bash &&
  ros2 run window_cleaner_planning path_follower --ros-args -p use_sim_time:=true
"


