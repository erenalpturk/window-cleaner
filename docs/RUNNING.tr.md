# Simülasyonu Çalıştırma

Otonom cam temizleme robotu simülasyonunun her oturumda nasıl açılıp
kapatılacağı. Faz 1 ilk kurulumu zaten tamamlandığı varsayılır.

> İngilizce sürümü: [RUNNING.md](RUNNING.md)
>
> **Bu dosyayı `docker/`, `src/window_cleaner_bringup/launch/`, port
> map'lemeleri, env değişkenleri ve harici araç gereksinimleriyle
> senkron tut.** Sim'in nasıl başlatıldığını etkileyen herhangi bir
> değişiklik yapıyorsan, aynı commit içinde bu dosyayı da güncelle.

## Ön gereksinimler (tek seferlik)

| Araç | Amaç | Kurulum |
|---|---|---|
| Docker Desktop (Apple Silicon) | Container runtime | https://www.docker.com/products/docker-desktop |
| Foxglove Studio (Apple Silicon) | 3D + kamera görselleştirme + teleop arayüzü | https://foxglove.dev/download |
| Homebrew | Foxglove'u `brew install --cask foxglove-studio` ile (opsiyonel) | https://brew.sh |

XQuartz **gerekli değil** — Apple Silicon XQuartz OGRE2 / RViz için
çalışan bir GLX context yaratamadığından, proje Foxglove WebSocket
kullanıyor.

İlk seferlik image build:

```bash
cd /Users/erenalpturk/Desktop/Projects/Robotic
docker compose -f docker/docker-compose.yml build
```

M4'te ilk seferinde ~10 dakika, sonraki Docker layer cache hit'lerinde
~1 dakika sürer.

## Sim'i Açma (her oturum)

### 1. Docker Desktop'ı başlat

```bash
open -a Docker
```

Menü çubuğundaki balina ikonunun animasyonu durana kadar bekle.

### 2. Sim'i başlat

```bash
cd /Users/erenalpturk/Desktop/Projects/Robotic

docker compose -f docker/docker-compose.yml run \
  --service-ports --rm --name wc-sim ros2-dev \
  bash -c "source install/setup.bash && \
           ros2 launch window_cleaner_bringup sim.launch.py \
           rviz:=false gui:=false foxglove:=true"
```

**Engelli dünyada** (Faz 3 zorlu dünya testi) çalıştırmak için `world:=`
argümanı ekleyin:

```bash
docker compose -f docker/docker-compose.yml run \
  --service-ports --rm --name wc-sim ros2-dev \
  bash -c "source install/setup.bash && \
           ros2 launch window_cleaner_bringup sim.launch.py \
           world:=/workspace/install/window_cleaner_worlds/share/window_cleaner_worlds/worlds/glass_obstacles.sdf \
           rviz:=false gui:=false foxglove:=true"
```

Robot **SW köşede** `(-2.15, -1.15, 0.05)` spawn olur — böylece coverage
planner'ın ilk waypoint'i robotun hemen yanındadır, Nav2 ilk strip'i
"geride" deyip atlamaz. `nav2.launch.py` `map → odom` static TF'ini
buna göre kaydırır ki `map` frame'i yine dünya koordinatlarıyla aynı
kalsın.

Bu komut container içinde şunları başlatır:

| Süreç | Görevi |
|---|---|
| `ign gazebo -s --headless-rendering` | Fizik + sensör render (GUI penceresi yok) |
| `robot_state_publisher` | URDF'den TF tree |
| `ros_gz_sim create` | Robotu dünyaya spawn eder |
| `ros_gz_bridge` | ROS ↔ Gazebo topic çevirisi |
| `foxglove_bridge` | `0.0.0.0:8765` üzerinde WebSocket sunucusu |

Sim hazır olduğunda log şu satırı gösterir:

```
[foxglove_bridge-5] [INFO] ... Server listening on port 8765
```

Önemli flag'ler:

- `--service-ports` — **şart**. Bu flag olmadan `docker compose run`,
  compose.yml'deki `ports:` alanını yok sayar ve 8765 Mac host'una
  ulaşmaz.
- `--name wc-sim` — ikinci bir terminalin `docker exec` ile aynı
  container'a bağlanmasını sağlar.
- `rviz:=false gui:=false foxglove:=true` — Apple Silicon varsayılanı.
  Çalışan OpenGL'i olan Linux host'ta tersini yapabilirsin.

### 3. Foxglove Studio'ya bağlan

1. Foxglove Studio'yu aç.
2. `Open Connection` → **Foxglove WebSocket** (Rosbridge **değil** —
   farklı protokollerdir).
3. URL: `ws://localhost:8765` → **Open**.

URL'nin yanındaki durum noktası bridge bağlantıyı kabul ettiğinde
yeşile döner. Kırmızı kalırsa, en alttaki Sorun Giderme'ye bak.

Bağlandıktan sonra bir sonraki bölümdeki layout'u kur.

### 4. Foxglove panel düzeni

Bu projenin standart layout'u. Bir kez kur, sonra **Layout → Save layout
as…** ile kaydet — sonraki oturumda otomatik yüklenir.

Foxglove boş başlar. Panel eklemek için: tab bar'daki **+** ikonuna
tıkla, ya da var olan bir panele sağ tıkla → **Split panel**
(sağ / aşağı) ve açılan menüden panel türünü seç.

Önerilen layout: 2×2 grid — sol-üst **3D**, sağ-üst **Image**,
sol-alt **Teleop**, sağ-alt **Raw Messages**. İstediğin gibi
yerleştirebilirsin; önemli olan aşağıdaki panel ayarları.

#### Panel 1 — 3D (sim dünyası + robot + path + costmap)

1. Panel ekle → **3D**.
2. Panele tıkla, sonra sağ-üstündeki **dişli (⚙) ikonuna** bas.
   Ayarlar sol kenar çubuğunda açılır.
3. **Frame** altında ayarla:
   - **Fixed frame**: `map` (Nav2 henüz çalışmıyorsa `odom`).
   - **Display frame**: `base_link`.
   - **Follow mode**: `Pose (position + attitude)` — kamera robotu
     takip etsin.
4. **Scene** altında:
   - **Label scale**: `0.14` (varsayılan 1.0 değeri TF etiketlerini
     robotun üzerini kaplayacak kadar büyük yapar).
   - İstersen **Show labels: Off** ile etiketleri tamamen kapat.
5. Sol kenardan **Topics**'i aç ve şu topic'lerin yanındaki göz ikonunu
   AÇIK yap:
   - `/robot_description` — URDF mesh'ini render eder.
   - `/tf` — her link için TF eksenleri.
   - `/robot/scan` (LaserScan) — lazer noktaları.
   - `/map` (OccupancyGrid) — statik cam haritası (beyaz iç, siyah
     çerçeve duvarları). Faz 3+.
   - `/global_costmap/costmap` (OccupancyGrid) — Nav2'nin duvar
     etrafındaki inflation halkası. Faz 3+.
   - `/planning/coverage_path` (Path) — boustrophedon zikzak overlay.
     Faz 3+. Alt ayarlarında: **Type: Line**, **Line width: 0.05**.
   - `/plan` (Path) — Nav2'nin mevcut segment için ürettiği canlı
     global plan. Faz 3+. **Type: Line**, **Line width: 0.03**, farklı
     bir renk.

#### Panel 2 — Image (kamera akışı + perception debug overlay'leri)

1. Panel ekle → **Image**.
2. Panel ayarlarında (dişli ikonu):
   - **Topic**: ham akış için `/robot/camera/image_raw`, ya da
     perception debug topic'lerinden biri:
     - `/perception/debug_image` — frame counter overlay
     - `/perception/debug_frame` — Hough çizgileri + cam sınırı
     - `/perception/debug_dirt` — kir konturları yeşilde
   - Birden fazla görüntüyü aynı anda izlemek için birden fazla Image
     paneli ekleyebilirsin.

#### Panel 3 — Teleop (ekran üstü pad ile manuel sürüş)

1. Panel ekle → **Teleop**.
2. Panel ayarlarında:
   - **Topic**: `/cmd_vel`
   - **Publish rate**: `10` Hz
   - **Linear x max**: `0.20` m/s (Nav2 RPP üst sınırına eşitle)
   - **Angular z max**: `1.0` rad/s
3. Ekrandaki ok butonları basılı tutulduğunda Twist mesajı yayınlar.

#### Panel 4 — Raw Messages (mission state, parametre değerleri vs.)

1. Panel ekle → **Raw Messages**.
2. **Topic**: `/control/mission_state` — otonom kaplama sırasında
   WAITING → RUNNING → DONE arasında geçer (Faz 3+).
3. Başka bir topic izlemek için Topic alanını değiştir. Faydalı
   olanlar:
   - `/perception/dirty_regions` — tespit edilen kir merkezlerinin
     PoseArray'i
   - `/odom` — robotun anlık pozu
   - `/rosout` — tüm node logları

#### (Opsiyonel) Manuel goal-pose yayınlama (Faz 3+)

Otonom planner'ı beklemeden tek bir Nav2 hedefi göndermek için:

1. Panel ekle → **Publish**.
2. Ayarlar:
   - **Topic**: `/goal_pose`
   - **Message schema**: `geometry_msgs/msg/PoseStamped`
   - **Editing mode**: `On`.
3. JSON editöründe doldur (örnek — cam'in ortası):
   ```json
   {
     "header": {"frame_id": "map"},
     "pose": {
       "position":    {"x": 1.5, "y": 0.5, "z": 0},
       "orientation": {"x": 0,   "y": 0,   "z": 0, "w": 1}
     }
   }
   ```
   `frame_id: "map"` alanı **zorunlu** — Nav2 boş frame'leri reddeder.
4. **Publish** butonuna bas. Robot oraya gider.

> Foxglove'un "3D'ye tıklayıp publish" modu da var ama mevcut Foxglove
> sürümünde topic remap'i tutarsız çalışıyor (varsayılan
> `/move_base_simple/goal`, `/goal_pose` değil). Yukarıdaki Publish
> paneli güvenilir yol.

#### Layout'u kaydet

**Layout → Save layout as…** → örn. `wc-default` adıyla kaydet. Bir
sonraki oturumda **Layout → Open** ile geri yüklersin.

### 5. Teleoperation (manuel sürüş)

**Seçenek A (önerilen): Foxglove Teleop paneli** — yukarıda
[Panel 3](#panel-3--teleop-ekran-%C3%BCst%C3%BC-pad-ile-manuel-s%C3%BCr%C3%BC%C5%9F)'te zaten
yapılandırıldı. Sürmek için ok butonlarına basılı tut.

**Seçenek B: ikinci terminalde klavye.**

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Tuşlar: `i` ileri, `,` geri, `j`/`l` döndür, `k` dur, `q`/`z` hızı
artır/azalt.

## Kod değişikliklerinden sonra rebuild

`src/` içindeki düzenlemeler (URDF, SDF, launch dosyaları, Python
node'lar):

```bash
docker compose -f docker/docker-compose.yml run --rm ros2-dev \
  bash -c "colcon build --symlink-install"
```

`--symlink-install` sayesinde launch / URDF / SDF / Python kaynakları
`install/` dizinine sembolik link'lenir. Bu tek build'den sonra, bu
dosyalardaki değişiklikler sadece launch'ı yeniden başlatınca
algılanır — yeniden build gerekmez. Yeni paket, yeni entry point veya
C++ değişiklikleri ise yine build gerektirir.

Dockerfile / image değişiklikleri:

```bash
docker compose -f docker/docker-compose.yml build
```

## Kapatma

Launch terminalinde: **Ctrl+C** (graceful — SIGINT'i sim, bridge ve
Foxglove'a propagate eder).

Launch terminali kapalıysa başka bir terminalden:

```bash
docker stop wc-sim
```

## Perception katmanını çalıştırma (Faz 2+)

Önce sim'i başlat, sonra ikinci terminalde:

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 launch window_cleaner_perception perception.launch.py
```

Bu üç node başlatır: `camera_node`, `frame_detector`, `dirt_segmenter`.

Görsel olarak doğrulamak için
[Foxglove panel düzeni](#4-foxglove-panel-d%C3%BCzeni)'ndeki Image
panelini aç ve **Topic**'i sırasıyla `/perception/debug_image` (ham
frame counter), `/perception/debug_frame` (tespit edilen cam sınırı)
ve `/perception/debug_dirt` (kir konturları yeşilde) arasında değiştir.

Yeniden build etmeden eşik değerlerini ayarlamak için
`src/window_cleaner_perception/config/perception_params.yaml`'i düzenle
ve `perception.launch.py`'yi yeniden başlat.

## Nav2'yi çalıştırma (Faz 3 Sub-phase B+)

Nav2 stack'i statik bir occupancy grid (`maps/glass_basic.pgm`)
kullanıyor ve `map` frame'i ile global/local costmap'leri yayınlıyor.
Identity `map → odom` static TF'inin sahibi de o — AMCL kullanılmıyor
çünkü `/odom` Ignition DiffDrive plugin'inden ground-truth geliyor ve
robot world (0, 0)'da spawn oluyor.

Bu, Sub-phase B'nin testi: Nav2'yi tek başına çalıştırıp manuel olarak
`/goal_pose` ile robotun bir hedefe gittiğini doğrulamak. Sub-phase C
ve sonrası için Nav2 zaten Planning launch akışının bir parçası
(aşağıdaki bölüme bak).

**1. Sim çalışıyor olsun.** [§"Sim'i başlat"](#2-simi-başlat) bölümündeki
komutu Terminal 1'de çalıştır.

**2. İkinci terminalde Nav2 stack'i başlat:**

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 launch window_cleaner_bringup nav2.launch.py
```

Lifecycle manager'ın şu log satırlarını yazdığını görmelisin (sırasıyla
~3-5 saniye içinde):

```
Configuring map_server
Activating map_server
... Configuring controller_server
... Activating bt_navigator
Managed nodes are active
```

**3. Foxglove'da doğrula.** [§"Foxglove panel düzeni"](#4-foxglove-panel-düzeni)'ndeki
layout açıksa, 3D panelinde:

- **`/map`** topic'i — beyaz cam alanı, siyah çerçeve duvarları görünür
- **`/global_costmap/costmap`** — çerçeve duvarlarının etrafında renkli
  inflation halkası
- Robot world (0,0)'da, cam'in tam ortasında

**4. Manuel hedef gönder.** [§"(Opsiyonel) Manuel goal-pose yayınlama"](#opsiyonel-manuel-goal-pose-yayınlama-faz-3)
bölümündeki Publish panelini kullan. Robot oraya planlayıp gitmeli.

SDF dünyayı düzenledikten sonra haritayı yeniden üret:

```bash
python3 src/window_cleaner_bringup/maps/gen_map.py
```

## Planning katmanını çalıştırma (Faz 3 Sub-phase C+)

Planning katmanında iki node var:

| Node | Topic / Action | Görevi |
|---|---|---|
| `coverage_planner` | `/planning/coverage_path` yayınlar (latched) | Boustrophedon zikzağı bir kez başlangıçta üretir. |
| `path_follower` | `navigate_through_poses` action'ını çağırır; `/control/mission_state` yayınlar | Path'i Nav2'ye verir; WAITING / RUNNING / DONE / ABORTED durumlarını raporlar. |

**Tam otonom kaplama** için üç terminal gerekiyor: sim, Nav2, planning.

**Terminal 1 — Sim:** [§"Sim'i başlat"](#2-simi-başlat)'taki komutu çalıştır.

**Terminal 2 — Nav2:** [§"Nav2'yi çalıştırma"](#nav2yi-çalıştırma-faz-3-sub-phase-b)'daki
komutu çalıştır. **`Managed nodes are active` mesajını bekle.**

**Terminal 3 — Coverage planner:**

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 run window_cleaner_planning coverage_planner \
  --ros-args --params-file /workspace/install/window_cleaner_planning/share/window_cleaner_planning/config/planning_params.yaml
```

Engelli dünya için bu params dosyasını obstacle-aware varyantla değiştir:

```bash
ros2 run window_cleaner_planning coverage_planner \
  --ros-args --params-file /workspace/install/window_cleaner_planning/share/window_cleaner_planning/config/planning_params_obstacles.yaml
```

`coverage_planner` boustrophedon path'i `TRANSIENT_LOCAL` durability ile
bir kez yayınlar, böylece sen `path_follower`'ı yeniden başlatırken
çalışmaya devam edebilir. Basit dünya için beklenen log satırı:

```
Published coverage path with 90 waypoints (9 strips, spacing=0.50m) ...
```

Engelli dünyada bu sayı daha düşük olur (~76) çünkü inflated engel AABB
içine düşen waypoint'ler atılır.

**Terminal 4 — Path follower:**

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 run window_cleaner_planning path_follower --ros-args -p use_sim_time:=true
```

`path_follower` yoğun waypoint trail'ini tek bir `NavigateThroughPoses`
goal'ü olarak gönderir; robot pattern'i sürer ve action
`STATUS_SUCCEEDED` döndüğünde durur. Mission state geçişleri terminalde
görünür:

```
received coverage path with 90 waypoints; waiting for action server...
sending NavigateThroughPoses goal with 90 poses
goal accepted by Nav2
mission_state -> RUNNING
... (uzun çalışma) ...
Nav2 reported SUCCESS — coverage complete
mission_state -> DONE
```

> **Niye `planning.launch.py` yerine ayrı terminaller?** Faz 3 debug'ında
> üç launch'ı shell `&` ile bundle'lamak `path_follower`'ın log'larını
> `foxglove_bridge` çıktısının altında boğdu. `coverage_planner` ve
> `path_follower`'ı `ros2 run` ile ayrı terminallerde çalıştırmak her
> node'un log'unu izole tutar ve birini Nav2'yi veya sim'i yeniden
> başlatmadan restart edebilirsin. `planning.launch.py` toplu kullanım
> için hâlâ kurulu.

Foxglove'da ilerlemeyi izlemek için
[§"Foxglove panel düzeni"](#4-foxglove-panel-düzeni)'ndeki layout'u kullan:

- **3D paneli** canlı planı (`/plan`), tüm kaplama yolunu
  (`/planning/coverage_path`), costmap halkasını ve robotun strip'ler
  arasında hareketini gösterir.
- **Raw Messages paneli** `/control/mission_state` topic'inde action
  ilerledikçe `WAITING → RUNNING → DONE` geçişlerini gösterir.

Planner'ı **tek başına** (Nav2 olmadan, sadece path görselleştirmek
için) çalıştırmak istersen sadece `planning.launch.py`'yi başlat —
ama kendi static `map → odom` TF'ini eklemen ya da 3D panelin Fixed
frame'ini `odom` yapman gerek (Nav2 olmadığı için TF orada olmayacak).

## Cleaning controller'ı çalıştırma (Faz 3 Sub-phase D+)

Cleaning controller 4-durumlu state machine (IDLE / MOVING / CLEANING /
EMERGENCY) çalıştırır ve `/control/vacuum_cmd`, `/control/brush_cmd`,
`/control/state` topic'lerini yayınlar. `/control/mission_state` üzerinden
(path_follower'dan) gate'lenir, `/odom`'a dirt-proximity matching için,
`/robot/scan`'a emergency stop için abone olur.

```bash
docker exec -it wc-sim bash
source install/setup.bash
ros2 launch window_cleaner_control control.launch.py
```

Beşinci terminalde state geçişlerini izle:

```bash
ros2 topic echo /control/state
```

Tek bir mission boyunca beklenen sekans:

```
IDLE       (mission_state == WAITING)
MOVING     (mission_state -> RUNNING, robot kir merkezlerinden uzakta)
CLEANING   (robot pose cache'lenmiş bir kir merkezine 0.20 m içinde — fırça AÇIK)
MOVING     (robot kir alanından çıktı)
...
IDLE       (mission_state -> DONE)
```

`EMERGENCY` lidar minimum range `0.08 m`'nin altına düştüğünde tetiklenir
ve lane `1.0 s` boyunca açık kaldıktan sonra temizlenir (hysteresis flicker
önler). EMERGENCY'de controller `/cmd_vel`'i sıfırlayıp Nav2'yi ezer.

Yeniden build etmeden bound'ları, strip width'i veya başlangıç köşesini
ayarlamak için
`src/window_cleaner_planning/config/planning_params.yaml`'i düzenle ve
`planning.launch.py`'yi yeniden başlat.

## Benchmark'ı çalıştırma (Faz 4)

Faz 4 `window_cleaner_evaluation` paketini ekler: pasif bir `metrics_node`
(misyon başına bir CSV satırı), gözetimsiz `run_benchmark.sh` taraması ve
çevrimdışı `plot_results.py`.

**Benchmark matrisi:** `glass_basic` + `glass_small`, her biri 3 koşu = 6
koşu (plan değişikliği 2026-05-16 — `glass_obstacles` güvenilir şekilde
ABORT ediyor, belgelenmiş bir Faz-3 sınırlaması; `glass_large` geometrik
olarak `glass_basic` ile aynı). Birçok koşu ABORTED/kısmi biter — bu bir
başarısızlık değil, dürüst veri setinin kendisidir.

Tüm taramayı **tek** komutla çalıştır (tek uzun-ömürlü container; script
koşular arasında Gazebo/DDS'i süreçleri öldürerek sıfırlar). Tamamen
gözetimsizdir (M4'te ~30–50 dk):

```bash
docker compose -f docker/docker-compose.yml run --service-ports --rm \
  --name wc-bench ros2-dev \
  bash -c "source install/setup.bash && \
    bash install/window_cleaner_evaluation/share/window_cleaner_evaluation/scripts/run_benchmark.sh --all --runs 3"
```

Seçenekler: `--world glass_basic|glass_small`, `--runs N`, `--timeout S`
(koşu başına sabit duvar-saati, varsayılan 480), `--bag` (her dünyanın 1.
koşusunda rosbag kaydı → `media/bags/`, görev 4.6).

Çıktılar:

* `results/metrics.csv` — koşu başına bir satır: `timestamp,world,
  run_index,mission_result,coverage_pct,collisions,duration_s,distance_m`
  (commit edilir).
* `results/benchmark_summary.txt`, `results/logs/<world>_run<N>/*.log` —
  ABORT'u sonradan teşhis için süreç-başına log (git-ignore).
* `media/plots/*.png` — kaplama / süre / çarpışma / özet grafikleri,
  sonda otomatik üretilir (commit edilir).

Tek misyon, manuel (debug): normal Faz-3 yığınını başlat (sim → nav2 →
perception → planning → control) ve `metrics_node` ekle:

```bash
docker compose -f docker/docker-compose.yml exec wc-sim bash -c \
  "source install/setup.bash && \
   ros2 launch window_cleaner_evaluation metrics.launch.py run_id:=1"
```

Bir dünya SDF'ini düzenledikten sonra Nav2 haritalarını yeniden üret (her
dünyanın `.pgm` + `.yaml`'ını yazar; `glass_basic` grid içeriği tasarım
gereği değişmez):

```bash
docker compose -f docker/docker-compose.yml exec wc-sim bash -c \
  "python3 src/window_cleaner_bringup/maps/gen_map.py"
```

`nav2.launch.py` artık additive `odom_offset_x` / `odom_offset_y`
argümanları alıyor (varsayılan `-2.15` / `-1.15`, yani 5×3 dünyalar için
değişmemiş). Benchmark, 2×1 yüzeyi sabit-kodlu offset'e sığmayan
`glass_small` için dünya-başına SW spawn'ı geçer.

Mevcut bir CSV'den istediğin zaman yeniden grafik üret:

```bash
docker compose -f docker/docker-compose.yml exec wc-sim bash -c \
  "python3 install/window_cleaner_evaluation/share/window_cleaner_evaluation/scripts/plot_results.py"
```

## Topic özet tablosu

| Topic | Tip | Hz | Yön |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | 1000 | GZ → ROS |
| `/cmd_vel` | `geometry_msgs/Twist` | yayınlandıkça | ROS → GZ |
| `/odom` | `nav_msgs/Odometry` | 30 | GZ → ROS |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | değiştikçe | GZ → ROS |
| `/joint_states` | `sensor_msgs/JointState` | yüksek | GZ → ROS |
| `/robot/scan` | `sensor_msgs/LaserScan` | ~9 | GZ → ROS |
| `/robot/camera/image_raw` | `sensor_msgs/Image` | ~22 | GZ → ROS |
| `/robot/camera/camera_info` | `sensor_msgs/CameraInfo` | ~22 | GZ → ROS |
| `/robot_description` | `std_msgs/String` | latched | sadece ROS |
| `/perception/debug_image` | `sensor_msgs/Image` | ~22 | sadece ROS |
| `/perception/debug_frame` | `sensor_msgs/Image` | ~22 | sadece ROS |
| `/perception/debug_dirt` | `sensor_msgs/Image` | ~22 | sadece ROS |
| `/perception/glass_boundary` | `geometry_msgs/PolygonStamped` | ~22 | sadece ROS |
| `/perception/dirty_regions` | `geometry_msgs/PoseArray` | ~22 | sadece ROS |
| `/perception/dirty_areas` | `std_msgs/Float32MultiArray` | ~22 | sadece ROS |
| `/planning/coverage_path` | `nav_msgs/Path` | latched (1) | sadece ROS |
| `/map` | `nav_msgs/OccupancyGrid` | latched | sadece ROS (map_server) |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | 1 | sadece ROS |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | 2 | sadece ROS |
| `/goal_pose` | `geometry_msgs/PoseStamped` | yayınlandıkça | Foxglove → Nav2 |
| `/plan` | `nav_msgs/Path` | hedef başına | Nav2 → ROS |
| `/control/mission_state` | `std_msgs/String` | 1 + değişimde | path_follower → ROS |
| `/control/state` | `std_msgs/String` | 10 | cleaning_controller → ROS |
| `/control/vacuum_cmd` | `std_msgs/Bool` | 10 | cleaning_controller → ROS |
| `/control/brush_cmd` | `std_msgs/Bool` | 10 | cleaning_controller → ROS |

## Sorun giderme

| Belirti | Çözüm |
|---|---|
| Foxglove "Connection failed — rosbridge WebSocket server…" | Yanlış bağlantı tipi. **Foxglove WebSocket** seç, Rosbridge değil. |
| Host'ta 8765 portu kapalı (`nc -z localhost 8765` → CLOSED) | Sim `--service-ports` olmadan başlatılmış. Flag'le yeniden başlat. |
| `Cannot connect to the Docker daemon` | Docker Desktop çalışmıyor. `open -a Docker` ve bekle. |
| `gz sim ...` komutu bulunamıyor | `ign gazebo` kullan — Humble, Ignition Gazebo 6 (Fortress) ile eşleşiyor. |
| xacro çalışırken `package 'window_cleaner_...' not found` | Önce `colcon build --symlink-install`, sonra `source install/setup.bash`. |
| Kamera / lidar topic'leri sessiz | `ign gazebo` süreci ölmüş — launch terminalinde `Aborted` / `segfault` ara. |
| `docker exec wc-sim …` "No such container" diyor | Sim hiç başlamadı veya zaten durdu. Bring-up komutunu yeniden çalıştır. |
| 3D panel Fixed-frame dropdown'ında `map` yok | Nav2 henüz aktive olmadı. `Managed nodes are active` mesajını bekle, sonra Foxglove'da disconnect + reconnect ile TF tree'yi yenile. |
| 3D panel topic toggle'ları çalışmıyor (göz ikonları gri kalıyor) | Topic henüz yayınlanmıyor. Container içinde `ros2 topic list` ile kontrol et. |
| Publish panel butonu gri / "frame_id required" diyor | JSON'daki `header.frame_id` boş. `"map"` yaz. |
| TF etiketleri robotun üstünü kaplıyor | 3D panel ayarları → **Scene → Label scale: 0.14**, ya da **Show labels: Off**. |

## Bu Foxglove workflow'u neden var

Apple Silicon'da XQuartz OpenGL/GLX context yaratamadığı için Gazebo'nun
OGRE2 GUI'si ve RViz2 ikisi de pencere oluşturma anında segfault eder.
Bu yüzden proje Gazebo'yu headless çalıştırıyor (`-s --headless-rendering`),
sensörleri off-screen render ediyor ve ROS topic'lerini görselleştirmek
+ teleop yayınlamak için Foxglove Studio'yu (native Mac uygulaması,
WebSocket protokolü) kullanıyor. Tam gerekçe:
[PROJECT_ROADMAP — AI Notes Faz 0 / Faz 1](../PROJECT_ROADMAP%20%281%29.md).
