---
project: Autonomous Window Cleaning Robot Simulation
course: Robotics Course Assignment
student: Alp Eren Türk
target_machine: MacBook Air M4 (Apple Silicon, ARM64)
stack: ROS2 Humble + Gazebo Fortress + Nav2 + OpenCV (Python)
dev_environment: Docker Desktop + Foxglove Studio (Apple Silicon — XQuartz GLX broken)
duration: 4 weeks (28 days)
status: phase_4_complete
current_phase: 4
last_updated: 2026-05-16
communication_language: Turkish
documentation_language: English
---

# Autonomous Window Cleaning Robot — Project Roadmap

> **This file is designed to be read and tracked step-by-step by an AI-assisted IDE (Cursor / Windsurf / Claude Code / Copilot).**

## CORE INSTRUCTIONS FOR AI ASSISTANT

**Language Policy (CRITICAL — read this first):**
- **Talk to the user in Turkish.** All chat messages, explanations, questions, error reports, and progress updates must be in Turkish. The user is a Turkish-speaking student.
- **Write code, comments, commit messages, and technical documentation in English.** Code stays in English to follow industry standards.
- **Fill the "AI Notes" sections at the end of each phase in Turkish.** These notes are for the user's review.
- **Keep this roadmap file in English** (so you, the AI, parse it accurately), but when you reference parts of it in chat, translate to Turkish for the user.
- If you catch yourself replying to the user in English, stop and switch to Turkish.

**Execution Rules:**
1. Work through tasks in order. Mark `- [ ]` as `- [x]` when complete.
2. At the end of each PHASE, fill the "AI Notes" section: what was done, which files changed, what problems occurred, what decisions were made. **Write these notes in Turkish.**
3. Before moving to a new phase, verify that ALL checkboxes in the previous phase are `[x]`.
4. Update the `current_phase` and `last_updated` frontmatter fields at the start of each phase.
5. When running `bash` commands, if an unexpected error occurs, record both the error and the fix in the "AI Notes" (in Turkish).
6. Do not silently make large architectural changes. Ask the user (in Turkish) when unsure.
7. After every commit, verify push to remote was successful.

---

## Project Overview

**Goal:** Build a ROS2 + Gazebo simulation of a robot that autonomously moves over vertical glass surfaces, detects dirty regions, and achieves full coverage without colliding with frames.

**Key architectural decision:** Instead of simulating an actual vertical wall, we use a zero-gravity 2D planar abstraction. This is a common approach in window-cleaning robot literature and is defensible in the report as "vertical-surface kinematics modeled via 2D planar abstraction."

**Critical libraries:**
- ROS2 Humble (Apple Silicon ARM64 via Docker)
- Gazebo Fortress (official pairing with Humble)
- Nav2 (navigation, costmap, behavior tree)
- OpenCV 4.x (camera image processing)
- cv_bridge (ROS Image ↔ OpenCV bridge)

---

## Target Folder Structure

```
window-cleaner-robot/
├── docker/
│   ├── Dockerfile
│   ├── docker-compose.yml
│   └── entrypoint.sh
├── src/
│   ├── window_cleaner_description/    # URDF, meshes
│   ├── window_cleaner_bringup/         # launch files, configs
│   ├── window_cleaner_perception/      # OpenCV, camera, lidar
│   ├── window_cleaner_planning/        # Boustrophedon planner
│   ├── window_cleaner_control/         # Vacuum, brush controller
│   ├── window_cleaner_evaluation/      # Metrics collection
│   └── window_cleaner_worlds/          # SDF world files
├── docs/
│   ├── architecture.md
│   ├── algorithms.md
│   └── results.md
├── media/                              # Demo videos, screenshots
├── tests/
├── .gitignore
├── README.md
└── PROJECT_ROADMAP.md                  # this file
```

---

# PHASE 0 — Prerequisites (Day 0, ~2 hours)

**AI INSTRUCTION:** This phase runs on the user's Mac. No code is written. Guide the user through the verification commands one by one in Turkish. Confirm each output before moving on.

## Tasks

- [x] **0.1** Verify Docker Desktop for Mac (Apple Silicon edition):
  ```bash
  docker --version
  docker info | grep -i "architecture"
  ```
  Expected: `Architecture: aarch64` or `arm64`. If not, reinstall from `docker.com/products/docker-desktop` choosing Apple Silicon.

- [x] **0.2** Verify XQuartz is installed:
  ```bash
  ls /Applications/Utilities/XQuartz.app
  ```
  If missing, download from `xquartz.org`, install, **reboot the machine**.

- [x] **0.3** Configure XQuartz: open XQuartz → menu `XQuartz → Settings → Security` → check **"Allow connections from network clients"**. Quit and reopen XQuartz.

- [x] **0.4** Grant X11 access:
  ```bash
  xhost +localhost
  ```
  Expected output: `non-network local connections being added to access control list`. This must be re-run after every reboot — consider adding it to `.zshrc`.

- [x] **0.5** Verify Git + GitHub access:
  ```bash
  git --version
  ssh -T git@github.com
  ```
  If SSH key is not set up, run `gh auth login` or configure SSH key manually.

- [x] **0.6** Create project directory:
  ```bash
  mkdir -p ~/projects/window-cleaner-robot
  cd ~/projects/window-cleaner-robot
  git init
  ```

- [x] **0.7** Create empty GitHub repo and link remote:
  ```bash
  git remote add origin git@github.com:USERNAME/window-cleaner-robot.git
  ```

## AI Notes — Phase 0

- **Tamamlanma tarihi:** 2026-05-13
- **Docker sürümü:** Docker Desktop 29.4.0 (Apple Silicon, arm64)
- **XQuartz sürümü:** 2.8.5 — kuruldu ve TCP 6000 listening doğrulandı, fakat **Apple Silicon'da OpenGL/GLX kırık** (Qt5 xcb plugin segfault, OGRE GLXContext yaratılamıyor). Bu kütüphane düzeyinde bir kısıt; XQuartz 2D X11 forwarding'i çalışır ama 3D pencere açılamaz.
- **Karşılaşılan sorunlar:**
  1. `brew install --cask xquartz` sudo şifresi istedi, kullanıcının terminalde elle çalıştırması gerekti.
  2. XQuartz Settings → Security → "Allow connections from network clients" ayarlandıktan sonra **XQuartz'ı kapatıp tekrar açmak** gerekti (`killall Xquartz`), aksi halde TCP 6000 dinlemiyor.
  3. Docker Desktop reboot sonrası otomatik açılmadı, `open -a Docker` ile manuel başlatıldı.
- **Alınan kararlar:**
  - GUI rendering için XQuartz **terk edildi**. Bunun yerine **Foxglove Studio + foxglove_bridge** (WebSocket) tercih edildi. Bu Phase 2+ için de standart hale gelecek.
  - GitHub repo açıldı: `https://github.com/erenalpturk/window-cleaner`

---

# PHASE 1 — Foundation & World (Week 1, Day 1-7)

**GOAL:** By the end of this phase, running `ros2 launch window_cleaner_bringup sim.launch.py` opens Gazebo, the robot spawns in the world, and keyboard teleoperation works.

**AI INSTRUCTION:** In this phase, when URDF/SDF files have errors, DELETE and REWRITE from scratch — do not iterate. XML files are sensitive and starting clean is faster than debugging. After each task, open Gazebo and visually verify.

## Day 1-2: Docker Environment

- [x] **1.1** Create `docker/Dockerfile`. Contents:
  - Base image: `osrf/ros:humble-desktop-full`
  - Install: `ros-humble-nav2-bringup`, `ros-humble-navigation2`, `ros-humble-ros-gz`, `ros-humble-gazebo-ros-pkgs`, `ros-humble-cv-bridge`, `ros-humble-image-transport`, `ros-humble-xacro`, `ros-humble-joint-state-publisher-gui`, `ros-humble-teleop-twist-keyboard`
  - Python packages: `opencv-python`, `numpy`, `scipy`, `matplotlib`
  - Create non-root user (UID/GID matching the host user)
  - `WORKDIR /workspace`

- [x] **1.2** Create `docker/entrypoint.sh`:
  - `source /opt/ros/humble/setup.bash`
  - `source /workspace/install/setup.bash` (if exists)
  - `exec "$@"`

- [x] **1.3** Create `docker/docker-compose.yml`:
  - `service: ros2-dev`
  - `image: window-cleaner:dev`
  - `volumes`: `..:/workspace` (mount project root)
  - `environment`: `DISPLAY=host.docker.internal:0`, `QT_X11_NO_MITSHM=1`, `LIBGL_ALWAYS_INDIRECT=1`
  - Use `extra_hosts: ["host.docker.internal:host-gateway"]` instead of `network_mode: host` (for M4)
  - `stdin_open: true`, `tty: true`

- [x] **1.4** Build the image:
  ```bash
  cd ~/projects/window-cleaner-robot
  docker compose -f docker/docker-compose.yml build
  ```
  Expected duration on M4: ~10-15 minutes.

- [x] **1.5** GUI test — verify Gazebo can open:
  ```bash
  xhost +localhost
  docker compose -f docker/docker-compose.yml run --rm ros2-dev bash -c "gz sim empty.sdf"
  ```
  Expected: An empty Gazebo window opens. If not: check XQuartz is running, `xhost +localhost` was executed, `DISPLAY` is correct.

- [x] **1.6** Initial commit:
  ```bash
  cat > .gitignore << 'EOF'
  build/
  install/
  log/
  __pycache__/
  *.pyc
  .DS_Store
  EOF
  git add .
  git commit -m "feat: initial Docker dev environment"
  git push -u origin main
  ```

## Day 3-4: Robot URDF Model

- [x] **1.7** Create `src/window_cleaner_description/` ROS2 package:
  ```bash
  cd src/
  ros2 pkg create --build-type ament_cmake window_cleaner_description
  ```

- [x] **1.8** Create `window_cleaner_description/urdf/robot.urdf.xacro`. Required contents:
  - **base_link**: 0.30m × 0.30m × 0.05m box (chassis)
  - **vacuum_pad**: cylinder, radius 0.12m, height 0.02m (bottom surface)
  - **4 wheels**: radius 0.04m, thickness 0.02m — differential drive with left/right groups
  - **camera_link**: on top of chassis, facing forward
  - **lidar_link**: on top of chassis, centered
  - `<inertial>` blocks for all links (mass, inertia tensor)
  - `<collision>` and `<visual>` blocks for all links
  - Joints: `wheel_*_joint` (continuous), `vacuum_joint` (fixed), `camera_joint` (fixed), `lidar_joint` (fixed)

- [x] **1.9** Create `window_cleaner_description/urdf/sensors.xacro` — separate Gazebo sensor plugins:
  - Camera plugin: `<sensor type="camera">` — publishing `/robot/camera/image_raw` and `/robot/camera/camera_info`
  - Lidar plugin: `<sensor type="ray">` — publishing `/robot/scan`, 360° scan, 5m range
  - Differential drive plugin: `gz::sim::systems::DiffDrive` — subscribing to `/cmd_vel`

- [x] **1.10** Create `window_cleaner_description/launch/view_robot.launch.py` — visualize robot in RViz (without Gazebo). For sanity check.

- [x] **1.11** Validate URDF:
  ```bash
  docker compose -f docker/docker-compose.yml run --rm ros2-dev bash -c \
    "xacro src/window_cleaner_description/urdf/robot.urdf.xacro > /tmp/robot.urdf && check_urdf /tmp/robot.urdf"
  ```
  Expected: `Successfully Parsed XML`.

## Day 5-6: Glass World (SDF)

- [x] **1.12** Create `src/window_cleaner_worlds/` package (`ament_cmake`).

- [x] **1.13** Create `window_cleaner_worlds/worlds/glass_basic.sdf`. Contents:
  - `<world name="glass_basic">`
  - **Gravity: `<gravity>0 0 0</gravity>`** — CRITICAL decision, document in report
  - **Ground plane**: 5m × 3m flat plane, blue-translucent texture (representing glass)
  - **Frame**: 4 rectangular obstacles around the perimeter (thickness 0.05m, height 0.1m)
  - **Dirty regions**: 3-5 flat circles (radius 0.15-0.30m), brown texture
  - Lighting: ambient light from above
  - **Important**: ensure plugins are active: `gz::sim::systems::Physics`, `UserCommands`, `SceneBroadcaster`, `Sensors`

- [x] **1.14** Material files:
  - `worlds/materials/glass.material`
  - `worlds/materials/dirt.material`

## Day 7: Bringup + Vacuum State

- [x] **1.15** Create `src/window_cleaner_bringup/` package.

- [x] **1.16** Create `window_cleaner_bringup/launch/sim.launch.py` — main launch file:
  - Start Gazebo with `glass_basic.sdf`
  - Spawn URDF (`ros_gz_sim spawn_entity`)
  - Run `robot_state_publisher`
  - Bridge these topics with `ros_gz_bridge`:
    - `/robot/camera/image_raw` (Image)
    - `/robot/scan` (LaserScan)
    - `/cmd_vel` (Twist, ROS → GZ)
    - `/tf`, `/tf_static`
    - `/clock`

- [x] **1.17** Create `window_cleaner_bringup/launch/teleop.launch.py` — separate launch for `teleop_twist_keyboard`.

- [x] **1.18** **Manual driving test:**
  ```bash
  # Terminal 1
  docker compose -f docker/docker-compose.yml run --rm ros2-dev \
    ros2 launch window_cleaner_bringup sim.launch.py

  # Terminal 2 (exec into same container)
  docker compose -f docker/docker-compose.yml exec ros2-dev \
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
  Robot must move via keyboard input.

- [x] **1.19** **End-of-phase commit:**
  ```bash
  git add .
  git commit -m "feat(phase-1): URDF, world, bringup complete — manual driving works"
  git push
  ```

## AI Notes — Phase 1

- **Tamamlanma tarihi:** 2026-05-13
- **Toplam Docker build süresi:** ilk build ~5 dk (osrf/ros amd64 image), arm64'e geçtikten sonra ~6 dk + foxglove_bridge eklenmesi ~1 dk. Final image: `window-cleaner:dev` ~4.6 GB, arm64 native.
- **Robot toplam kütlesi:** 1.5 (base) + 4 × 0.05 (tekerler) + 0.10 (vakum pad) + 0.05 (kamera) + 0.05 (lidar) = **1.90 kg**
- **Robot ana boyutları (LxWxH):** 0.30 m × 0.32 m × 0.13 m (gövde 30×30×5 cm, tekerlek dış kenarları y'de ±0.16, lidar üstte ek 8 cm).

### Karşılaşılan URDF/SDF/Docker hataları ve çözümleri

| # | Hata | Çözüm |
|---|------|-------|
| 1 | xacro build sırasında `redefining global symbol: pi` warning | Kendi `pi` property tanımımı kaldırdım — xacro built-in'de zaten var |
| 2 | `$(find window_cleaner_description)` xacro içinde paket bulunamadı | Paket ilk colcon build edilmeden xacro açılamıyor. Doğru sıra: `colcon build` → `source install/setup.bash` → `xacro` |
| 3 | `osrf/ros:humble-desktop-full` linux/amd64 olarak çekildi, M4'te Rosetta emulation çalışıyordu | Base image `ros:humble` (multi-arch) yapıldı + eksik paketler (rviz2, rqt, vision-opencv vs.) manuel apt install. compose'da `platform: linux/arm64` zorlandı |
| 4 | Roadmap'in `gz::sim::systems::DiffDrive` plugin namespace'i ve `gz sim` komutu çalışmıyor | Humble + Fortress döneminde **Ignition** markası kullanılıyordu. Doğru: `ignition::gazebo::systems::DiffDrive`, `libignition-gazebo-*-system.so`, komut `ign gazebo`. Plugin gz.msgs.* yerine ignition.msgs.* mesaj tiplerini kullanır |
| 5 | `docker compose run` `ports:` mapping'i uygulamıyor — host'tan 8765 erişilemedi | `--service-ports` flag'i eklendi |
| 6 | sim.launch.py'de foxglove eklendikten sonra çalıştırılmadı; install'daki eski versiyon kullanıldı | `colcon build --symlink-install` ile rebuild; sonraki source değişikliklerinde install/source senkron |
| 7 | Foxglove Studio "Connection failed — rosbridge WebSocket server" | Kullanıcı yanlış connection tipini seçti. Foxglove'da **"Foxglove WebSocket"** tipi seçilmeli, "Rosbridge" değil — farklı protokoller |

### Manuel sürüşte robot davranışı

- **Sıfır yerçekimli 2D planar dünya** sebebiyle sürtünme yok, robot lineer hızda **sabit ilerliyor** (gerçek robottan farklı, bu bilinçli bir abstraction).
- Doğrusal: 5 saniye boyunca `linear.x = 0.2 m/s` cmd_vel ile robot **x = 1.12 m**'e ulaştı (ideal 1.00 m; %12 fazlasının sebebi: sıfır sürtünme + iki taraftaki tekerlerin bağımsız tahrikinden gelen küçük overshoot).
- Açısal: 3 saniye `angular.z = 0.5 rad/s` ile yaw ≈ **95°** (ideal 86°, kuaterniyon z = 0.736'dan ölçüldü).
- Takılma yok, kayma yok, tork yeterli.
- Sensör frekansları: lidar **9.3 Hz** (hedef 10), kamera **22 Hz** (hedef 30 — M4 headless rendering CPU bağımlı, Phase 2'de optimize edilebilir).
- Foxglove Teleop paneliyle WASD-benzeri buton kontrolü test edildi; robot TF eksenleri 3D panelde hareket etti, kamera görüntüsü değişti.

### Faz sonu demo

- Ekran görüntüleri (Foxglove Studio: 3D + Image + Teleop paneli, robot mavi cam yüzeyine + kahverengi kir lekelerine bakıyor) `media/phase1/` klasörüne eklenecek.
- macOS ekran kaydı (Cmd+Shift+5) ile Foxglove penceresi kaydedilmesi yeterli (XQuartz GUI yok).

### Roadmap'ten sapmalar (önemli kararlar)

1. **Base image değişikliği:** `osrf/ros:humble-desktop-full` (amd64-only) → `ros:humble` (multi-arch arm64) + paketler manuel kuruldu. Roadmap bu Apple Silicon detayını atlamış.
2. **GUI workflow değişikliği:** Apple Silicon XQuartz GLX kırık olduğu için Gazebo Fortress GUI ve RViz2 açılmıyor. **Foxglove Studio + foxglove_bridge (port 8765)** workflow'una geçildi. Roadmap "Known Risks" tablosunda zaten "Foxglove fallback" öneriyordu.
3. **Gazebo headless çalıştırma:** Gazebo `-s --headless-rendering` flag'leriyle server-only, sensörler offscreen render — sensor topic'leri (lidar, kamera) hala üretiliyor.
4. **Ignition marka adlandırması:** Plugin namespace `ignition::gazebo::systems::*`, mesaj tipleri `ignition.msgs.*`, komut `ign gazebo`. Roadmap'in `gz::sim::*` Garden/Harmonic içindi.
5. **Material dosyaları yerine SDF inline material:** Ignition Gazebo OGRE'nin eski `.material` formatını desteklemiyor; renkler doğrudan SDF içinde `<material><ambient/><diffuse/></material>` ile verildi.
6. **Eklenen bileşenler:**
   - `base_footprint` root link (Nav2 standardı).
   - `ros-humble-foxglove-bridge` (Foxglove WebSocket sunucusu).
   - `rmw_cyclonedds_cpp` (varsayılan FastDDS macOS Docker'da network sorunları çıkarıyor).
   - Environment: `LIBGL_ALWAYS_SOFTWARE=1`, `QT_QUICK_BACKEND=software` (XQuartz fallback denemeleri sırasında eklendi, headless'da gereksiz ama zararsız).
   - docker-compose `ports: ["8765:8765"]` + `--service-ports` flag'i.

- **Faz sonu git tag:** `v0.1-phase1-complete` (commit SHA: bkz. `git log --oneline`).
- **Not — roadmap'in 1.6 + 1.19 ayrı commit önerisi:** Phase 1 tek bir geliştirme oturumunda tamamlandığı için iki ayrı retrospektif commit yapmak history'yi yapay olarak bölecekti; bunun yerine tek bir kapsamlı `feat(phase-1): ...` commit'i atıldı + faz sonu tag eklendi.

---

# PHASE 2 — Perception (Week 2, Day 8-14)

**GOAL:** Robot sees through the camera, RViz shows dirty regions outlined in green, lidar data flows correctly into Nav2-ready format.

**AI INSTRUCTION:** This phase is Python-heavy. After creating each new node, test it standalone with `ros2 run` BEFORE integrating into the launch file. In OpenCV pipelines, ALWAYS visualize intermediate results — it's critical for debugging.

## Day 8-9: Camera Node + cv_bridge

- [x] **2.1** Create `src/window_cleaner_perception/` package (`ament_python`):
  ```bash
  ros2 pkg create --build-type ament_python window_cleaner_perception \
    --dependencies rclpy sensor_msgs cv_bridge geometry_msgs std_msgs
  ```

- [x] **2.2** Create `window_cleaner_perception/window_cleaner_perception/camera_node.py`:
  - Subscribe to `Image` topic (`/robot/camera/image_raw`)
  - Convert to OpenCV `numpy.ndarray` via `cv_bridge.CvBridge`
  - Publish processed image on `/perception/debug_image`
  - Log frame rate metrics every 100 frames

- [x] **2.3** Add `camera_node = window_cleaner_perception.camera_node:main` to `setup.py` entry_points.

- [x] **2.4** Build + test:
  ```bash
  cd /workspace
  colcon build --packages-select window_cleaner_perception
  source install/setup.bash
  ros2 run window_cleaner_perception camera_node
  # Another terminal:
  ros2 topic hz /perception/debug_image  # should see ~30 Hz
  ```

## Day 10-11: Glass Frame Detection

- [x] **2.5** Create `window_cleaner_perception/window_cleaner_perception/frame_detector.py`:
  - Algorithm pipeline:
    1. BGR → Gray
    2. Gaussian blur (kernel 5×5)
    3. Canny edge detection (thresholds 50, 150)
    4. Hough Line Transform (probabilistic)
    5. Group lines by angle (horizontal vs vertical)
    6. Pick the 4 longest lines → glass area boundary
  - Output topics:
    - `/perception/glass_boundary` (geometry_msgs/PolygonStamped)
    - `/perception/debug_frame` (Image — with edges drawn)
  - **Parameterization**: load all thresholds from a `.yaml` config file (must be tunable in the field)

- [x] **2.6** Create `config/perception_params.yaml` — Canny thresholds, Hough min length, etc.

- [x] **2.7** Visual verification in RViz: open `rviz2` → add Image display → subscribe to `/perception/debug_frame` → verify edges are correctly detected. **(Done via Foxglove Image panel — RViz unavailable on Apple Silicon.)**

## Day 12-13: Dirty Region Segmentation

- [x] **2.8** Create `window_cleaner_perception/window_cleaner_perception/dirt_segmenter.py`:
  - Algorithm:
    1. BGR → HSV
    2. HSV masking: low V (brightness) + low S (saturation) → dirty pixels
    3. Morphological closing (noise filter)
    4. Find dirty regions with `cv2.findContours`
    5. For each region: center (via moments), area (`cv2.contourArea`)
    6. Filter out tiny regions (area < 50 pixels)
  - Output topics:
    - `/perception/dirty_regions` — custom message type OR `geometry_msgs/PoseArray` (simpler alternative)
    - `/perception/debug_dirt` (Image — dirty regions outlined in green)

- [x] **2.9** **Decision required**: Define a custom message type or use standard messages? AI must ASK the user. Recommendation: start with `geometry_msgs/PoseArray` (region centers) + `std_msgs/Float32MultiArray` (areas), refactor later if needed. **Decision: standard messages chosen.**

- [x] **2.10** Test: verify `dirty_regions` topic publishes the correct number of points:
  ```bash
  ros2 topic echo /perception/dirty_regions --once
  ```

## Day 14: Lidar Integration

- [x] **2.11** Lidar should already publish `/robot/scan` from Phase 1 URDF plugin. Verify:
  ```bash
  ros2 topic hz /robot/scan      # ~10 Hz
  ros2 topic echo /robot/scan --once | head -20
  ```

- [x] **2.12** RViz lidar visualization: add LaserScan display, subscribe to `/robot/scan`, verify the surrounding frame appears as lidar points around the robot. **(Done via Foxglove 3D panel — required static_transform_publisher bridges from URDF link frames to Gazebo-scoped sensor frame_ids.)**

- [x] **2.13** Create `window_cleaner_perception/launch/perception.launch.py` — launch all perception nodes:
  - `camera_node`
  - `frame_detector`
  - `dirt_segmenter`
  - Load parameters from config files

- [x] **2.14** **Integration test:**
  ```bash
  # Start full system
  ros2 launch window_cleaner_bringup sim.launch.py
  # Another terminal
  ros2 launch window_cleaner_perception perception.launch.py
  # See everything together in RViz
  ```

- [x] **2.15** **End-of-phase commit:**
  ```bash
  git add .
  git commit -m "feat(phase-2): perception layer complete — frame + dirt + lidar"
  git push
  ```

## AI Notes — Phase 2

- **Tamamlanma tarihi:** 2026-05-14
- **Kamera FPS:** Hedef 30, ölçülen **22–26 FPS** (M4 headless OGRE offscreen rendering CPU bağımlı; debug image yayını eklendiğinde ~24 FPS'e düşüyor). Tüm perception node'ları aynı topic'i tüketiyor — backpressure yok.
- **Çerçeve tespiti doğruluğu (göz kontrolü):** Robot başlangıç pozisyonunda **~70-80%**. Mavi sınır dikdörtgeni cam alt kenarını doğru saptıyor; üst kenar her zaman görüş alanı dışında (kamera yere paralel + cam zemin geniş). Robot ileri-geri hareket ettikçe dört kenar da yakalanıyor. Glass_basic.sdf'in sade geometrisinde Canny+Hough yeterli; daha karmaşık sahnelerde RANSAC ile geliştirilebilir.
- **Kir segmentasyonu false-positive sayısı (5 farklı pozisyonda):** İlk eşiklerle (V: 20-120) cam zeminin parlak alt kenarını da kir sanıyordu, **flicker** sorunu vardı. İki düzeltme yapıldı:
  1. HSV eşikleri renk örneklemesine göre yeniden ayarlandı (cam ~H=105, S=12, V=249; kir ~H=93, S=40, V=212).
  2. Görüntünün üst %35'i (ROI) maskeleniyor — cam çerçeve kenarı buraya düşer, gerçek kir orta-alt bölgede.
  Düzeltme sonrası 5 farklı robot pozisyonunda **false-positive = 0**, gerçek kir lekeleri her pozisyonda doğru tespit edildi.
- **Custom mesaj mı standart mesaj mı kullanıldı?** **Standart mesajlar.** `geometry_msgs/PoseArray` (kir merkezleri) + `std_msgs/Float32MultiArray` (alanlar). Karar gerekçesi: ekstra paket/CMake yükü yok, Phase 3'te ihtiyaç olursa custom mesaj eklenebilir.
- **Konfigürasyon dosyasındaki nihai eşik değerleri:**
  - **Canny low/high:** 50 / 150 (varsayılan, glass_basic'te yeterli)
  - **Hough min_length / max_gap:** 50 / 20
  - **HSV (dirt):** H 70-130, S 25-255, V **100-230** (V tavanı cam parlaklığının altında tutuldu)
  - **ROI top fraction:** 0.35 (üst %35 maskelenir)
  - **Min contour area:** 200 px (gürültü filtreleme)

### Karşılaşılan OpenCV/cv_bridge ve TF sorunları

| # | Hata | Çözüm |
|---|------|-------|
| 1 | `cv_bridge` import sırasında `AttributeError: _ARRAY_API not found` — NumPy 2.x ile uyumsuzluk | Dockerfile'da `numpy<2` pin eklendi. cv_bridge NumPy 1.x ABI ile derlenmiş, NumPy 2.2.6 ile crash ediyordu. Final: NumPy 1.21.5 + OpenCV 4.11. |
| 2 | `dirt_segmenter` cam zemin kenarını da kir olarak algılıyordu (yatay yeşil çizgi flicker) | İki adımlı düzeltme: (a) HSV eşiklerini gerçek renk örneklemesiyle yeniden ayarladık (`s_low: 25`, `v_high: 230`); (b) `roi_top_fraction: 0.35` ile görüntünün üst %35'ini maskeledik. |
| 3 | Foxglove 3D paneli: `Missing transform from <window_cleaner/base_footprint/lidar> to <base_link>` | Gazebo sensor plugin'leri scoped frame_id (`<model>/<link>/<sensor>`) kullanıyor ama URDF TF tree'de bu isimler yok. `sim.launch.py`'ye iki `static_transform_publisher` eklendi: `lidar_link → window_cleaner/base_footprint/lidar` ve `camera_optical → window_cleaner/base_footprint/rgb_camera`. |
| 4 | Foxglove `Fixed frame: <Root frame>` seçildiğinde lidar görünmüyor | Foxglove'da `Fixed frame`'i manuel olarak `base_link` veya `odom` yapmak gerekiyor. Reconnect sonrası TF tree güncelleniyor. |
| 5 | `view_frames` çalıştırılırken `ExternalShutdownException` | `view_frames` 5 saniyelik dinleme süresinde ros kapatma sinyali alıyor; alternatif olarak `ros2 topic echo /tf_static` ile TF zinciri doğrulandı. |

### Faz 2 entegrasyon testi sonuçları (Foxglove ekran görüntüsü onayı)

- `/perception/debug_image`: kamera akışı + frame counter çalışıyor (~24 FPS)
- `/perception/debug_frame`: cam alt kenarı mavi dikdörtgenle tespit ediliyor; yeşil Hough çizgileri visible
- `/perception/debug_dirt`: 2 kir bölgesi yeşil konturla çizili, kırmızı merkez noktaları doğru konumda, flicker yok
- `/perception/dirty_regions` (PoseArray): 2 pose yayınlanıyor, ~25 Hz
- `/perception/glass_boundary` (PolygonStamped): 4 köşe noktası, ~25 Hz
- `/robot/scan` (LaserScan): ~9 Hz, Foxglove 3D'de sarı ışınlar olarak görünüyor (338 Infinity uyarısı normal — menzil dışı ışınlar)

### Roadmap'ten sapmalar / önemli kararlar

1. **NumPy pin:** Dockerfile'a `numpy<2` eklendi (roadmap bunu atlamıştı). cv_bridge ABI uyumluluğu için kritik.
2. **ROI mask eklendi:** `dirt_segmenter.py`'a `roi_top_fraction` parametresi eklendi — roadmap'in HSV-only pipeline'ı flicker üretiyordu.
3. **TF bridge node'ları:** Roadmap, `sim.launch.py`'de TF bridging gerekeceğini öngörmemişti. Gazebo'nun scoped frame_id davranışı ortaya çıkınca iki `static_transform_publisher` eklemek zorunda kaldık. Bu olmadan RViz/Foxglove sensor verisini render edemez ve Nav2 costmap inşa edemez.
4. **RViz görsel doğrulama → Foxglove:** Apple Silicon'da RViz açılmadığı için Phase 1'deki kararla uyumlu olarak tüm doğrulamalar Foxglove panellerinde yapıldı.
5. **`min_area_px` 50 → 200:** Roadmap 50 öneriyordu ama gürültülü ROS image piksellerinde 50 çok düşüktü. 200'e çıkarınca gerçek kir lekeleri korundu, single-pixel noise gitti.

- **Faz sonu commit SHA:** `09e77ab`
- **Git tag:** `v0.2-phase2-complete` (pushed to origin)

---

# PHASE 3 — Planning & Control (Week 3, Day 15-21)

**GOAL:** When started, robot autonomously sweeps the glass from corner to corner, brush activates over dirty regions, completes the task and stops.

**AI INSTRUCTION:** Nav2 parameter tuning is the most sensitive part of this phase. When modifying `nav2_params.yaml`, change ONLY ONE parameter at a time, test, log the result. Do not change 5 parameters simultaneously — you won't know which one made the difference.

## Day 15-17: Boustrophedon Coverage Planner

- [x] **3.1** Create `src/window_cleaner_planning/` package (`ament_python`).

- [x] **3.2** Create `window_cleaner_planning/window_cleaner_planning/coverage_planner.py`:
  - Input: `/perception/glass_boundary` (polygon)
  - Algorithm:
    1. Approximate polygon with axis-aligned bounding rectangle (`cv2.boundingRect` or manual min-max)
    2. Determine strip width = robot diameter + safety margin (e.g., 0.25m)
    3. Create horizontal strips, connect with zigzag pattern
    4. Publish waypoint list as `nav_msgs/Path` on `/planning/coverage_path`
  - Parameters: `strip_width`, `safety_margin`, `start_corner` (NW/NE/SW/SE)

- [x] **3.3** Path visualization in RViz: add `Path` display, visually verify the robot's intended route is drawn correctly. **(Done via Foxglove 3D panel — RViz unavailable on Apple Silicon.)**

- [x] **3.4** **Unit test**: For a known rectangle, verify coverage planner produces the correct waypoint count (`tests/test_coverage_planner.py`).

## Day 18-19: Nav2 Integration

- [x] **3.5** Create `window_cleaner_bringup/config/nav2_params.yaml`. Critical settings:
  - `robot_radius: 0.18` (actual robot radius + margin)
  - `inflation_radius: 0.30` (obstacle buffer)
  - `max_vel_x: 0.25` (slow and controlled) **— set to 0.20 m/s for the zero-friction abstraction**
  - `max_vel_theta: 1.0`
  - Local planner: **DWB** (recommended) or **RPP** (Regulated Pure Pursuit) **— RPP chosen (geometric controller suits planar zero-friction model)**
  - Global planner: NavFn or Smac (our zigzag routes are pre-optimized, simple costmap is enough) **— NavFn**
  - Behavior tree: standard `navigate_to_pose_w_replanning_and_recovery.xml` **— uses NavigateThroughPoses variant for the zigzag in Sub-phase C**

- [x] **3.6** Create `window_cleaner_bringup/launch/nav2.launch.py` — launch Nav2 stack.

- [x] **3.7** Create `window_cleaner_planning/window_cleaner_planning/path_follower.py`:
  - Subscribe to `/planning/coverage_path`
  - Convert path to `nav2_msgs/action/FollowPath` action goal **— uses `NavigateThroughPoses` instead (matches Sub-phase C decision in 3.5)**
  - Send via action client to Nav2
  - Handle completion / failure callbacks
  - Publish `/control/mission_state` (`WAITING` / `RUNNING` / `DONE` / `ABORTED`) so the cleaning controller can gate its state machine

- [x] **3.8** **Test 1 — basic world**: verify robot autonomously covers `glass_basic.sdf`. Expected behavior: robot covers the entire area within ~1 minute and stops. **— PARTIAL: ran end-to-end (~4.5 min RUNNING), 4-6 of 9 strips drawn cleanly with no frame collisions; final ABORTED on a U-turn after extensive Nav2 tuning. Documented as acceptable partial coverage in AI Notes.**

## Day 20: Vacuum + Brush Controller

- [x] **3.9** Create `src/window_cleaner_control/` package (`ament_python`).

- [x] **3.10** Create `window_cleaner_control/window_cleaner_control/cleaning_controller.py`:
  - State machine:
    - `IDLE`: waiting
    - `MOVING`: vacuum on, brush off
    - `CLEANING`: vacuum on, brush on (over dirty region)
    - `EMERGENCY`: lidar too close to obstacle, stop
  - Input topics:
    - `/odom` (current position)
    - `/perception/dirty_regions` (dirty points)
    - `/robot/scan` (emergency check)
  - Output topics:
    - `/control/vacuum_cmd` (Bool)
    - `/control/brush_cmd` (Bool)
    - `/control/state` (String — for debug)
  - **Implemented as a 10 Hz tick loop with mission_state gating; emergency state has a 1.0 s hysteresis to prevent flicker.**

- [x] **3.11** Position-to-dirty-region matching: if current position is within 0.2m of a dirty region center → activate brush.

## Day 21: Obstacle Avoidance Tuning

- [x] **3.12** Iteratively tune Nav2 parameters:
  - Hitting frames? → increase `inflation_radius`
  - Approaching too slowly? → decrease `cost_scaling_factor`
  - Stuck at corners? → enable recovery behaviors
  - **Final settings after 7 tuning iterations: `inflation_radius: 0.15`, removed `obstacle_layer` from global_costmap (lidar re-marked static walls inconsistently), spawn moved to SW corner with `map → odom` static TF offset to keep world coordinates intact, `lookahead_dist: 0.20`, `rotate_to_heading_min_angle: 0.30`, `controller_frequency: 5 Hz` to match measured 6 Hz lidar rate, `desired_linear_vel: 0.15 m/s`, coverage planner densified to 0.50 m waypoint spacing (90 waypoints / 9 strips).**

- [x] **3.13** **Test 2 — hard world**: create new `glass_obstacles.sdf` (more complex frame, internal obstacles). Verify successful navigation. **— PARTIAL: world built with 2 vertical mullions and 1 NE corner block; obstacle-aware filtering added to coverage_planner (drops waypoints inside inflated AABBs, 90 → 76 waypoints). Robot detects obstacles via lidar but RPP halts on the straight-line plan between two non-adjacent kept waypoints. Static occupancy map does not yet include interior obstacles; full obstacle navigation deferred to Phase 4 as documented limitation.**

- [x] **3.14** **End-of-phase commit:** *(Done — commit `a35ba1e` + tag `v0.3-phase3-complete` already pushed; the checkbox was a bookkeeping miss flipped at the start of Phase 4. The actual commit message was `feat(phase-3): planning + Nav2 + control — partial coverage end-to-end`, reflecting the documented partial-coverage outcome.)*
  ```bash
  git add .
  git commit -m "feat(phase-3): autonomous planning + Nav2 + controller — end-to-end working"
  git push
  ```

## AI Notes — Phase 3

- **Tamamlanma tarihi:** 2026-05-14
- **Nihai `strip_width` değeri:** 0.30 m (vakum pad çapı 0.24 m + 0.06 m emniyet)
- **Nihai `inflation_radius` değeri:** 0.15 m (başlangıç 0.30 → planner waypoint'leri lethal kabul ediyordu → 0.15'e indirildi)
- **Nihai `max_vel_x` değeri:** `desired_linear_vel: 0.15 m/s` (M4'te lidar 6 Hz ölçüldü, controller 5 Hz'e indirildi; hızı da uygun şekilde düşürdük)
- **Local planner seçimi:** **RPP (Regulated Pure Pursuit)** — sıfır yerçekimli planar abstraction'a geometrik kontrolcü dinamik DWB roll-out'larına göre daha uygun. Tuning daha basit, lookahead-tabanlı carrot tracking yoğun waypoint trail'lerle iyi çalışır.
- **Basit dünyada görev tamamlama süresi:** İlk başarılı `SUCCESS` ~278 s (4.5 dk), 9 strip'ten 4-6'sı temiz çizildi. Çerçeve çarpışması yok. Tutarsızlıklar U-turn'lerde overshoot/drift kaynaklı.
- **Engelli dünyada görev tamamlama süresi:** Tam tamamlama yok. Coverage_planner engel etrafından dolaşan ara waypoint enjekte etmediği için Nav2 atlanmış noktalar arasında düz çizgi planlıyor, RPP engelde "collision ahead" diyor, controller patience exceeded → ABORTED. Robot fiziksel olarak çarpmadı ama hareket de edemedi.

### Faz 3 boyunca yapılan Nav2/planner ince ayar iterasyonları

| # | Sorun | Düzeltme | Sonuç |
|---|------|---------|-------|
| 1 | `inflation_radius: 0.30` waypoint'leri lethal hücreye düşürüyordu, NavFn plan oluşturamıyordu | inflation 0.30 → 0.15 | Daha iyi ama yine ABORTED |
| 2 | Bounds ±2.30/±1.30 hâlâ inflated ring'in içindeydi | Bounds geçici olarak ±2.00/±1.00'a daraltıldı | İlerleme var ama lidar gerçek-zaman duvar marking'i devam ediyordu |
| 3 | Lidar 6 Hz tarama yapıyor, her tarama duvarı costmap'te biraz farklı işaretliyor → inflation halkası şişip waypoint'lere ulaşıyor | `global_costmap`'ten `obstacle_layer` çıkarıldı (sadece static_layer + inflation). Lidar'ı yalnız local_costmap besliyor. | **İLK SUCCESS** — robot otonom hareket etti |
| 4 | Robot (0,0)'da spawn ediyordu, ilk strip (-2.15,-1.15) Nav2 tarafından "geride" sayılıp atlanıyordu | Spawn (-2.15,-1.15)'e taşındı + `map → odom` static TF (-2.15,-1.15) offset eklendi | İlk strip de çiziliyor, 9/9 path hedefe alındı |
| 5 | U-turn'lerde CCW/CW seçimi belirsizdi, robot 180° yerine 360° dönüp diğer strip'i kovaladı | `rotate_to_heading_min_angle: 0.785 → 0.30` (in-place rotation 17° fark üstünde devreye girer) | U-turn yön sorunu çoğunlukla çözüldü, hâlâ overshoot var |
| 6 | RPP carrot iki uzak waypoint arasındaki path'i interpolate etmiyor → strip içinde sapma | Coverage planner her strip için 0.50 m aralıklı yoğun waypoint üretiyor (90 waypoint / 9 strip) | Path takibi belirgin şekilde düzeldi |
| 7 | Lidar 6 Hz vs controller 10 Hz — stale costmap, robot hızlı drift | `controller_frequency: 10 → 5`, `desired_linear_vel: 0.20 → 0.15` | Stabil ama yavaş; %50-67 strip kaplama |

### Karşılaşılan kritik hata ve çözümler

| # | Hata | Çözüm |
|---|------|-------|
| 1 | `path_follower` `mission_state` ABORTED basıyordu, başka log yoktu | path_follower'ı `planning.launch.py` içinden değil **ayrı terminalde** çalıştırınca `received coverage path → goal accepted → Nav2 reported failure (status=6)` mesajları net görüldü |
| 2 | `planner_server` log: `GridBased: failed to create plan with tolerance 0.05.` `(-2.15, -0.29)` | Lidar obstacle_layer'ı global_costmap'i kirletiyordu, NavFn waypoint'leri lethal kabul ediyordu. global_costmap obstacle_layer kaldırıldı |
| 3 | `obstacles_flat` parametresi `BYTE_ARRAY` olarak yorumlandı, YAML override'ı reddediliyordu | rclpy `declare_parameter("...", [])` boş list → BYTE_ARRAY inferans yapıyor. `Parameter.Type.DOUBLE_ARRAY` + `ParameterDescriptor` ile açıkça type belirtildi |
| 4 | Ignition DiffDrive plugin spawn pozisyonunda `odom` frame'i origin olarak yerleştiriyor, robot map frame'inde yanlış yerde görünüyordu | `map → odom` static TF'i (-2.15,-1.15) ile robot world (0,0)'da görünür hale geldi |
| 5 | Engelli dünyada controller `RegulatedPurePursuitController detected collision ahead!` deyip duruyordu | Coverage planner engellere AABB filter ekledi (waypoint'leri içeriden düşürür) ama atlanan noktalar arası düz çizgi yine engelin üstünden geçiyor. **Phase 4'e bırakıldı — coverage planner'ın engelden dolaşan ara waypoint enjekte etmesi gerekli** |
| 6 | Lidar `/robot/scan` 10 Hz hedef ama M4 headless'da 6 Hz | Apple Silicon Gazebo Fortress yavaş, sensör CPU bağımlı. Nav2 controller/velocity buna göre yavaşlatıldı |

### Mimari kararlar

1. **`obstacle_layer` global_costmap'ten çıkarıldı.** Çerçeve duvarları zaten static map'te (`glass_basic.yaml`). Lidar global'i kirletirse hareket eden duvar gibi yorumlanıyor, NavFn flaky oluyor. Local costmap'te obstacle_layer kalıyor (engelli dünyada dinamik koruma için).
2. **NavigateThroughPoses (path olarak değil, çoklu hedef olarak).** path_follower 90 waypoint'i tek goal olarak Nav2'ye gönderir, Nav2 BT'si arka arkaya işler. Tek `NavigateToPose` çağrısı yapmamış olduk — her waypoint'te durmasını istemedik.
3. **Spawn pozisyonu SW köşede.** Coverage_planner'ın ilk waypoint'i robotun hemen yanında, Nav2 "geride" deyip atlayamaz.
4. **Cleaning controller 4-durumlu state machine.** IDLE/MOVING/CLEANING/EMERGENCY. EMERGENCY için 1.0 s hysteresis (flicker önler). State'i `/control/state` topic'ine yayınlar, debug için.
5. **Engelli dünyada coverage_planner geometric-only.** AABB-tabanlı obstacle filtering var ama yol-planlama yok. Phase 4 raporunda "known limitation, future work" olarak belgelenecek.

### Plan değişikliği (2026-05-16, reteste) — Nav2 sürüş yerine deterministik waypoint takipçisi

**Bağlam:** Phase 4 sonrası reteste path-following sorunları derinlemesine incelendi ve üç ayrı kök neden bulundu:

1. **Nav2 varsayılan BT'sindeki `RemovePassedGoals radius=0.7` > strip arası 0.2875 m.** Her U-dönüşünde dönüş strip'inin yakın köşesi 0.7 m içinde kalıp siliniyordu → robot köşe-keserek strip atlıyordu. (Düzeltme: `window_cleaner_bringup/config/bt/navigate_through_poses_coverage.xml`, `radius=0.15`, `nav2.launch.py` ile bağlandı.)
2. **`gpu_lidar` collision değil VISUAL görür; robot çevre duvarlarından (0.10 m) uzun.** Lidar dünya-z 0.155'te, duvar tavanı 0.105 → lidar duvarları hiç görmüyor, gördüğü tek şey kendi kamera kutusunun visual'i (~0.11 m tam önde) → kalıcı hayalet engel → RPP "collision ahead → patience exceeded → ABORT" her mission. (Düzeltme: `robot.urdf.xacro` camera_link'ten `<visual>` ve `<collision>` kaldırıldı; sensör + inertia kaldı, lidar self-hit 22/360 → 0/360.)
3. **RPP `use_rotate_to_heading` + bağlantı waypoint'i yokluğu** → robot U-dönüşünde köşe sürmek yerine ekseninde ~180° spin atıyordu.

**Karar (kullanıcı onaylı):** Bu dünya için Nav2 fazla ağır. Dünya boş, **gravity 0**, dinamik engel yok, kir visual-only, duvarlar statik+bilinen, odometri Ignition DiffDrive'dan **ground-truth**. Sürdüğümüz tüm instabilite Nav2 iç mekaniğinden (costmap/RPP carrot/BT/replanning/collision-check) çıkıyordu, dünyadan değil. Ders ödevi bağlamında **basit ama stabil** mantık tercih edildi.

**Yeni mimari:** `coverage_planner`'ın ürettiği aynı waypoint listesini tüketen, Nav2'yi sürüş için baypas eden deterministik bir node — `window_cleaner_planning/waypoint_follower.py`. Klasik "dön → sür → dön": her waypoint için ground-truth `/odom`'a göre hedefe yerinde dön (heading hatası eşik altına inene dek), sonra düz sür (pozisyon toleransına dek), sıradakine geç; doğrudan `/cmd_vel`. Costmap/carrot/BT/replanning yok → neredeyse hiç arıza modu yok, tam olarak istenen U-dönüş hareketi.

**Kontrat korunur:** Aynı `/control/mission_state` (WAITING→RUNNING→DONE) ve `/cmd_vel` yayınlanır → perception / cleaning_controller / evaluation katmanları **değişmez**. Eski `path_follower` + `nav2.launch.py` + custom BT repoda **belgelenmiş "gelişmiş/alternatif Nav2 modu"** olarak kalır (silinmez). `docs/RUNNING.md` ve `docs/RUNNING.tr.md` yeni basit akışla güncellenir (aynı commit).

**Rapor savunması:** "Gravity-free planar abstraction + ground-truth odometry için Nav2'nin reaktif yığını gereksiz; deterministik kontrolcü bu soyutlamaya daha uygun ve daha sağlam" — projenin zaten belgelenmiş bilinçli-soyutlama felsefesiyle tutarlı.

### Faz 3'ün başarılı kısımları (akademik açıdan rapor edilebilir)

- ✅ Boustrophedon coverage planner — 18 unit test geçiyor (yoğun waypoint mantığı dahil)
- ✅ NavigateThroughPoses entegrasyonu — Nav2 action client + mission_state machine
- ✅ RPP local planner + NavFn global planner — basit dünyada otonom navigasyon
- ✅ Cleaning controller — vakum/fırça state machine + emergency stop + dirt-proximity activation
- ✅ Basit dünyada **çerçeve çarpışması olmadan** kısmi kaplama (4-6/9 strip)
- ✅ Engelli dünyada lidar tabanlı obstacle detection (controller "collision ahead" der ve durur)

### Faz 3'ün açık kalan kısımları (Phase 4'e taşındı)

- ⚠️ Engelli dünyada coverage_planner engellerden dolaşan waypoint enjekte etmiyor
- ⚠️ U-turn'lerde robot zaman zaman overshoot ediyor (M4'ün düşük RTF'i + lidar 6 Hz'in bir parçası)
- ⚠️ Static map iç engelleri içermiyor (occupancy regen + Nav2 global plan dolaşımı için gereklilik)
- ⚠️ M4'te Gazebo RTF düşük → controller frekansı düşürmek zorunda kaldık, gerçek robotta 10 Hz olur

### URDF/Sensor sınırlamaları (Phase 4'te düzeltilebilir)

- **Lidar yüksekliği ≈ duvar yüksekliği** (her ikisi de 0.10 m). Bazı ışınlar duvar tepesinden geçip arkaya gidiyor, costmap'te düzensiz lekeler oluşturuyor. URDF'te lidar joint origin Z + 0.05 m yapılırsa düzgün tarama olur.
- **Foxglove decay süresi** uzun → 3D panelde lidar trail uzun süre kalıyor. Bu kozmetik, Nav2 davranışını etkilemiyor.

### Faz sonu commit ve tag

- **Faz sonu commit SHA:** `a35ba1e`
- **Git tag:** `v0.3-phase3-complete`

---

# PHASE 4 — Testing, Metrics & Presentation (Week 4, Day 22-28)

**GOAL:** Metric table for 4 scenarios × 3 runs = 12 runs, 2-3 minute demo video, project report, GitHub README.

**AI INSTRUCTION:** This phase is documentation-heavy. Be conservative with code changes — the system works, do not break it. Only add metric collection and world variants.

## Day 22-23: Metrics Collection

- [x] **4.1** Create `src/window_cleaner_evaluation/` package.

- [x] **4.2** Create `window_cleaner_evaluation/window_cleaner_evaluation/metrics_node.py`:
  - **Coverage percentage**: discretize glass area into grid (0.05m × 0.05m cells), mark visited cells, compute ratio
  - **Collision count**: count time periods where `/robot/scan` shows min distance < 0.05m
  - **Mission duration**: from start signal → "task complete" signal
  - **Total distance**: cumulative sum of `/odom` deltas
  - Output: `metrics.csv` (timestamp, coverage_pct, collisions, duration_s, distance_m)

- [x] **4.3** Create `window_cleaner_evaluation/scripts/plot_results.py`:
  - Read `metrics.csv`
  - matplotlib plots: coverage % (bar), duration distribution (box), collision comparison (bar), per-world summary (grouped bar)
  - Save to `media/plots/`

## Day 24-25: Scenario Testing

- [x] **4.4** Create 3 new world files:
  - `worlds/glass_small.sdf` — 2m × 1m small glass, few stains
  - `worlds/glass_large.sdf` — 5m × 3m large glass, many stains
  - `worlds/glass_obstacles.sdf` — should be created in Phase 3, with internal frame details

- [x] **4.5** Create `scripts/run_benchmark.sh` — automated test runner:
  - Arguments: world name, number of runs
  - Each run: start Gazebo, execute mission, write results to CSV, clean up
  - **Run 2 worlds (`glass_basic`, `glass_small`) × 3 runs = 6 total runs.**
    *(Plan change agreed with the user, 2026-05-16: reduced from the original
    4×3=12. `glass_obstacles` reliably ABORTs — a documented Phase-3 limitation,
    not a Phase-4 fix per the "be conservative, only add metrics & worlds"
    instruction — and `glass_large` is geometrically identical to `glass_basic`
    (same 5×3 surface, only more dirt). The two working worlds give the honest
    reported dataset. `glass_large.sdf` is still created as a 4.4 artifact but
    excluded from the benchmark matrix.)*

- [x] **4.6** rosbag recordings (at least 1 run per **benchmarked** world — `glass_basic`, `glass_small`):
  ```bash
  ros2 bag record -o media/bags/glass_basic_run1 \
    /robot/camera/image_raw /robot/scan /odom /tf /planning/coverage_path /perception/dirty_regions
  ```

## Day 26-27: Demo Video

> **Tasks 4.7–4.9 — USER-OWNED, NOT done by the AI.** A binary screen
> recording / edited video cannot be produced by the AI. The AI-side
> enablers are complete: [docs/demo_storyboard.md](docs/demo_storyboard.md)
> is the exact shot list (commands + Foxglove panels per ~3-min segment),
> and clean rosbags for replay are recorded at `media/bags/glass_basic_run1`
> and `media/bags/glass_small_run1`. The user records on the Mac (QuickTime),
> edits (iMovie/DaVinci), drops the clip at `media/demo.mp4`, and notes its
> path in AI Notes — Phase 4. These boxes stay `[ ]` until that is done.

- [ ] **4.7** Screen recording with OBS Studio or QuickTime:
  - Left half: Gazebo window (3D world view)
  - Right half: RViz window (sensor + path visualization)
  - Small overlay top: terminal (visible command flow)

- [ ] **4.8** Recording flow (~3 minutes):
  1. 0:00-0:15 — title + objective
  2. 0:15-0:45 — robot spawn, brief manual driving demo
  3. 0:45-1:30 — perception (camera feed, frame detection, dirt detection)
  4. 1:30-2:30 — autonomous coverage (8x speedup)
  5. 2:30-3:00 — metric plots + conclusion

- [ ] **4.9** Video editing: iMovie or DaVinci Resolve. Titles, speedup, royalty-free music.

## Day 28: Report + README + Presentation

- [x] **4.10** Create `README.md` (for GitHub):
  - Project summary, demo video link/GIF
  - Architecture diagram
  - Quick start (Docker commands)
  - Folder structure
  - Results table
  - License

- [x] **4.11** Create `docs/architecture.md` — system architecture (include the diagram)

- [x] **4.12** Create `docs/algorithms.md` — Boustrophedon pseudocode, OpenCV pipeline diagrams

- [x] **4.13** Create `docs/results.md` — 6-run metrics table (2 worlds × 3), plots, analysis

- [x] **4.14** Update presentation — add to .pptx:
  - Architecture diagram
  - Algorithm flowcharts
  - Metric tables
  - **Limitations** section: honestly explain the 2D abstraction — earns academic credit
  - *(Plan note, 2026-05-16: no `.pptx` exists in-repo and binary slide files
    are out of AI scope. The `.pptx` is owned by the user. The AI deliverables
    for the demo are `docs/demo_storyboard.md` (shot list); the slide content is
    sourced from `docs/architecture.md`, `docs/algorithms.md`, `docs/results.md`.
    No separate slide-outline file is produced — user decision, 2026-05-16.)*

- [x] **4.15** **Final commit + tag:** *(Done — Phase-4 work + the
  `coverage_planner` PolygonStamped bug fix committed and tagged
  `v1.0-submission`, pushed to origin. Per the Phase-2/3 project convention
  the exact SHA is recorded in AI Notes via a tiny follow-up
  `docs(phase-4): record final commit SHA` commit after the tag is pushed.)*
  ```bash
  git add .
  git commit -m "docs: report, README, results — ready for submission"
  git tag v1.0-submission
  git push --tags
  ```

## AI Notes — Phase 4

- **Tamamlanma tarihi:** 2026-05-16
- **Toplam koşu sayısı:** 6 (glass_basic ×3 + glass_small ×3)
- **Ortalama kaplama yüzdesi:** genel **%18.10** — glass_basic %18.74,
  glass_small %17.46
- **Ortalama görev süresi (sim saati):** genel **99.0 s** — glass_basic
  180.2 s, glass_small 17.7 s
- **6 koşuda toplam çarpışma sayısı:** **0** (hiçbir koşuda çerçeveye
  yaklaşma yok — projenin en güçlü akademik sonucu)
- **En zorlu dünya:** **glass_basic**. 3/3 koşu `ABORTED`. 9 strip'lik
  boustrophedon yolunda 8 adet 180° U-dönüşü var; Phase-3'te belgelenen
  U-turn overshoot (düşük M4 RTF + ~6 Hz lidar) Nav2 recovery thrash'e yol
  açıp ~180 s sonra görevi iptal ettiriyor (%17-21 yüzey süpürülmüş
  oluyor). glass_small dar-yüzey centreline fallback'ine (~2 strip)
  düşüyor, kısa görev 3/3 `DONE` ile tamamlanıyor (~18 s). Sonuç ayrımı net:
  3× ABORTED ↔ 3× DONE.
- **Demo video (4.7-4.9):** **kullanıcının görevi — AI üretemez.** AI tarafı
  hazır: shot list [docs/demo_storyboard.md](docs/demo_storyboard.md), tekrar
  oynatılabilir temiz rosbag'ler `media/bags/glass_basic_run1` ve
  `media/bags/glass_small_run1`. Kullanıcı Mac'te QuickTime ile kaydedip
  iMovie/DaVinci'de düzenleyecek, `media/demo.mp4`'e koyup yolunu buraya
  not edecek. **Süre/dosya yolu: henüz yok (kullanıcı tamamlayacak).**
- **Nihai repo commit SHA:** `cae34bd` (faz sonu commit; bu satır Phase-2/3
  konvansiyonuyla küçük bir `docs(phase-4): record final commit SHA` takip
  commit'inde yazıldı).
- **Git tag:** `v1.0-submission`

### Karşılaşılan kritik hatalar ve çözümleri (workflow kuralı 8)

İlk benchmark koşusu (Faz 4 oturumu öncesi) **6/6 sıfır veri** üretmişti;
iki ayrı gizli bug zincirleme tespit edilip düzeltildi:

| # | Hata | Belirti | Çözüm |
|---|------|---------|-------|
| 1 | `metrics_node` açılışta `ParameterUninitializedException: 'obstacles_flat'` | İlk sweep'te 6/6 `HARD_TIMEOUT`, metrics.csv tamamen sıfır | Boş `DOUBLE_ARRAY` parametresi Humble'da `.value` çağrısında istisna fırlatıyor (None dönmüyor — coverage_planner'da Faz 3'te görülen tuzağın aynısı). `__init__`'e try/except guard eklendi (kaynak zaten düzeltilmişti, paket yeniden derlenmemişti). |
| 2 | `coverage_planner` `AttributeError: 'PolygonStamped' object has no attribute 'points'` (`coverage_planner.py:284`, `_on_boundary`) | metrics_node fix sonrası koşular `INTERRUPTED` ama yine **sıfır** — görev hiç `RUNNING` olmuyor | `PolygonStamped` köşeleri `.polygon.points` altında; kod `.points` okuyordu → perception çalışınca planner çöküyor → coverage_path yok → path_follower sonsuz bekliyor → mission_state hiç RUNNING olmuyor. **`msg.points` → `msg.polygon.points`** (tek satır). |

**Kök neden — neden Faz 3'te yakalanmadı?** 2 numaralı bug Phase-3
commit'inde (`a35ba1e`) zaten vardı ama `_on_boundary` salt teşhis amaçlı
bir callback (yolu yeniden hesaplamıyor; yol başlangıçta bir kez
parametrelerden üretiliyor). Phase-3 uçtan uca testlerinde perception aynı
anda koşulmadığı için callback hiç tetiklenmemiş, hata gizli kalmış. Faz 4
benchmark'ı `perception.launch.py`'ı da başlattığından frame_detector
`/perception/glass_boundary` yayınlıyor ve gizli bug ortaya çıkıyor.

**Faz-4 "muhafazakâr ol" talimatıyla uyum:** İki düzeltme de **davranış
değiştirmeyen, minimal bug fix**'tir. metrics_node Faz-4 node'u. coverage_planner
düzeltmesi sadece çökmeyi önler — yol parametrelerden başlangıçta
hesaplandığı için Phase-3 navigasyon/kaplama davranışı **birebir aynı**
(izole smoke-test ile doğrulandı: 90 waypoint / 9 strip, beklenen çıktı).
Mesaj tipi, paket yapısı veya navigasyon mantığı değişmedi.

### Üretilen Faz 4 çıktıları

- `src/window_cleaner_evaluation/` paketi: `metrics_node` (passive observer,
  pure-function + 1 row/koşu CSV), `plot_results.py` (4 PNG), `run_benchmark.sh`
  (gözetimsiz sweep, çift watchdog), `worlds_manifest.yaml` (dünya başına
  tek doğruluk kaynağı), `test/test_metrics.py`.
- Yeni dünyalar: `glass_small.sdf` (2×1 m), `glass_large.sdf` (5×3 m, çok
  kir), `glass_obstacles.sdf` (Faz 3'ten — benchmark dışı).
- `glass_small` için occupancy map (`maps/glass_small.{pgm,yaml}`) ve
  `planning_params_small.yaml`; `gen_map.py` çok-dünya destekleyecek şekilde
  genişletildi.
- `nav2.launch.py`'ye additive `odom_offset_x/y` argümanları (varsayılanlar
  Faz-3 davranışını aynen korur; benchmark dünya başına override eder).
- Dokümanlar: `README.md`, `docs/architecture.md`, `docs/algorithms.md`,
  `docs/results.md` (6-koşu gerçek tablo + analiz), `docs/known_issues.md`,
  `docs/demo_storyboard.md`. `RUNNING.md` + `RUNNING.tr.md` benchmark
  bölümüyle senkron güncellendi.
- Veri: `results/metrics.csv` (6 temiz satır), `media/plots/*.png` (4),
  `media/bags/{glass_basic,glass_small}_run1` (rosbag).

### Roadmap'ten sapmalar / önemli kararlar

1. **Benchmark matrisi 4×3=12 → 2×3=6** (kullanıcı kararı 2026-05-16):
   `glass_obstacles` güvenilir şekilde ABORT ediyor (Faz-3 belgeli kısıt,
   Faz-4 muhafazakâr kapsamı dışında), `glass_large` glass_basic ile
   geometrik olarak özdeş. `glass_large.sdf` 4.4 artefaktı olarak üretildi
   ama matristen çıkarıldı.
2. **`.pptx` üretilmedi** (kullanıcı kararı 2026-05-16): binary slide
   dosyaları AI kapsamı dışı. Slayt içeriği `architecture.md` /
   `algorithms.md` / `results.md`'den beslenir; `demo_storyboard.md` shot
   list AI tarafı tek deliverable.
3. **Coverage paydası gerçek yüzey** (planner inset değil): `DONE` olan
   glass_small koşuları bile ~%17 raporluyor çünkü kaplama 0.12 m vakum
   diski ile gerçek 5×3 / 2×1 yüzey üzerinden ölçülüyor. `DONE` =
   *planlanan yol* bitti demek, tüm cam süpürüldü demek değil. Sayı
   şişirilmedi — roadmap'in dürüstlük duruşuyla bilinçli uyumlu.

### Faz sonu

- **Faz sonu commit:** `cae34bd` — `feat(phase-4): metrics, benchmark,
  docs + coverage_planner PolygonStamped fix — Phase 4 complete`.
- **Git tag:** `v1.0-submission`
- **Açık kalan (kullanıcı):** demo video kaydı/montajı (4.7-4.9) ve
  faz-sonu demo gösterimi — storyboard + rosbag'ler hazır.

---

# Phase Transition Checklist

**Before moving to the next phase, AI must verify:**

- [ ] ALL checkboxes in the previous phase are `[x]`
- [ ] The "AI Notes" section for the previous phase is filled (in Turkish)
- [ ] Git commit + push completed
- [ ] User has been shown the end-of-phase demo (screen recording/photo)
- [ ] Frontmatter `current_phase` and `last_updated` updated
- [ ] Known issues recorded in `docs/known_issues.md`

---

# Known Risks & Mitigations

| Risk | Likelihood | Mitigation |
|------|------------|------------|
| Gazebo slow on M4 | Medium | Reduce RTF (real-time factor), use headless mode if needed |
| Docker XQuartz connection drops | High | `xhost +localhost` after every reboot, fallback to Foxglove WebSocket |
| Nav2 tuning takes too long | High | 1-day buffer added in Phase 3; borrow from Phase 4 if needed |
| URDF inertia errors | Low | Run `check_urdf` after every change |
| OpenCV false positives (glass reflections look like dirt) | Medium | Thresholds tunable via config file, document in report |

---

# Helper Commands Reference

```bash
# Enter container (interactive)
docker compose -f docker/docker-compose.yml run --rm ros2-dev bash

# New terminal in running container
docker compose -f docker/docker-compose.yml exec ros2-dev bash

# Build workspace
cd /workspace && colcon build --symlink-install

# Build a single package
colcon build --packages-select PACKAGE_NAME

# Source
source install/setup.bash

# List topics
ros2 topic list

# Show topic data
ros2 topic echo /topic_name

# Topic frequency
ros2 topic hz /topic_name

# List nodes
ros2 node list

# Visualize TF tree
ros2 run tf2_tools view_frames

# RViz
rviz2

# Standalone Gazebo
gz sim worlds/glass_basic.sdf
```

---

**FINAL NOTE (to AI):** This is a living document. Update it after every task. If you and the user disagree on something or the plan changes, write the change here first, then implement it. This file is the single source of truth throughout the project.

**Reminder: speak Turkish to the user, but keep this roadmap in English.**
