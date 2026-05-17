# Hızlı Başlangıç

Sadece komutlar. Detay, panel ayarları ve sorun giderme için:
[RUNNING.tr.md](RUNNING.tr.md).

> **Varsayılan sürüş modu Nav2 değil.** Plan değişikliği (2026-05-16, roadmap
> Faz 3 AI Notes) ile sürüşü deterministik `waypoint_follower` yapıyor; Nav2
> repoda **gelişmiş/alternatif mod** olarak kalıyor (en altta). Aşağıdaki akış
> varsayılan (basit, Nav2'siz) moddur.

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

`Server listening on port 8765` mesajını bekle. Robot SW köşede
`(-2.15, -1.15, 0.05)` spawn eder; sim statik `map→odom` TF yayınlar
(`map_odom_tf:=true`, varsayılan), Foxglove'da Fixed frame `map` kalabilir.

## 2. Foxglove'u bağla

Foxglove Studio → **Open Connection** → **Foxglove WebSocket** →
`ws://localhost:8765`.

## 2a. Panel düzeni (ilk seferinde)

Foxglove boş açılır. Panel eklemek için tab bar'daki **+** ikonuna bas → panel
türünü seç. Ekranı 2×2'ye bölmek için bir panele sağ tıkla → **Split panel
right** / **Split panel down**.

**Panel 1 — 3D** (sol-üst, robotu/dünyayı gör)

1. Panele tıkla → sağ-üst **⚙ (dişli)** ikonu → ayarlar solda açılır.
2. **Frame**: Fixed frame `map`, Display frame `base_link`,
   Follow mode `Pose (position + attitude)`
3. **Scene → Label scale**: `0.14`
4. Sol kenardaki **Topics** listesinde göz ikonuyla AÇ:
   - `/robot_description`, `/tf`, `/robot/scan`
   - `/planning/glass_boundary_viz` (cam çerçevesi — varsayılan modda
     `/map` OccupancyGrid yoktur)
   - `/planning/coverage_path` (Line, width `0.05`)

> Topic'leri Foxglove bağlandıktan **sonra** başlattıysan listede görünmez.
> **Disconnect → Open Connection** ile yeniden bağlan.

**Panel 2 — Image** (sağ-üst, kamera)

1. Panel ekle → **Image**.
2. **⚙** → **Topic**: `/robot/camera/image_raw`
   (perception için: `/perception/debug_frame` veya `/perception/debug_dirt`)

**Panel 3 — Teleop** (sol-alt, manuel sürüş)

1. Panel ekle → **Teleop**.
2. **⚙**: Topic `/cmd_vel`, Publish rate `10`, Linear x max `0.20`,
   Angular z max `1.0`

**Panel 4 — Raw Messages** (sağ-alt, mission state)

1. Panel ekle → **Raw Messages**.
2. **⚙ → Topic**: `/control/mission_state`

**Kaydet:** üst menü **Layout → Save layout as…** → `wc-default`. Sonraki
oturumda **Layout → Open** ile geri yükle.

## 3. Otonom kaplama — varsayılan mod (Nav2'siz)

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

Robot her strip'i sürer, strip geçişlerinde temiz iki aşamalı U-dönüşü yapar;
`/control/mission_state` WAITING → RUNNING → DONE.

## 4. Perception (opsiyonel, ayrı terminal)

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 launch window_cleaner_perception perception.launch.py
```

## 5. Cleaning controller (opsiyonel, ayrı terminal)

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 launch window_cleaner_control control.launch.py
```

## Benchmark (Faz 4, gözetimsiz ~30-50 dk)

```bash
docker compose -f docker/docker-compose.yml run --service-ports --rm \
  --name wc-bench ros2-dev \
  bash -c "source install/setup.bash && \
    bash install/window_cleaner_evaluation/share/window_cleaner_evaluation/scripts/run_benchmark.sh --all --runs 3"
```

## Kod değişikliğinden sonra rebuild

```bash
docker compose -f docker/docker-compose.yml run --rm ros2-dev \
  bash -c "colcon build --symlink-install"
```

`--symlink-install` sayesinde launch / URDF / SDF / Python kaynakları
hot-link; sonraki düzenlemeler için rebuild gerekmez (yeni paket / entry
point hariç).

## Kapatma

Terminal 1'de **Ctrl+C**. Ya da başka terminalden:

```bash
docker stop wc-sim
```

## Gelişmiş / alternatif mod: Nav2

Varsayılan akış Nav2 kullanmaz. Nav2 referans akışını (NavFn + Regulated
Pure Pursuit + özel BT) bilerek çalıştırmak istersen tam komutlar
[RUNNING.tr.md](RUNNING.tr.md) "Planning katmanını çalıştırma — gelişmiş
Nav2 modu" bölümünde. Özet: sim'i `map_odom_tf:=false` ile başlat →
`nav2.launch.py` → `coverage_planner` → `path_follower`.
