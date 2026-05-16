#!/usr/bin/env bash
# Phase 4 benchmark runner (tasks 4.5 + 4.6).
#
# Runs the full autonomous mission for each benchmarked world N times,
# letting metrics_node write one CSV row per run, then plots the results.
# Designed to be UNATTENDED — every run is bounded by metrics_node's sim
# watchdog AND an outer wall-clock timeout, so the sweep always terminates
# with a complete metrics.csv even when missions ABORT or wedge Gazebo
# (the documented Phase-3 behaviour).
#
# Run it as the command of ONE long-lived container (fresh DDS/Gazebo state
# is reset between runs by killing processes, not the container):
#
#   docker compose -f docker/docker-compose.yml run --service-ports --rm \
#     --name wc-bench ros2-dev \
#     bash -c "source install/setup.bash && \
#       bash install/window_cleaner_evaluation/share/window_cleaner_evaluation/scripts/run_benchmark.sh --all --runs 3"
#
# Usage: run_benchmark.sh [--world NAME|--all] [--runs N] [--timeout S] [--bag]
#   --all (default)  all worlds in the manifest's benchmark_worlds list
#   --world NAME      a single world (glass_basic | glass_small)
#   --runs N          runs per world (default 3)
#   --timeout S       hard wall-clock seconds per run (default 480)
#   --bag             record a rosbag on run 1 of each world (task 4.6)

set -u
set -o pipefail
# NOTE: deliberately NO `set -e` — an ABORTED mission is an expected,
# recorded outcome; the loop must continue.

# --------------------------------------------------------------------------- #
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="$(cd "$SCRIPT_DIR/../config" && pwd)"
MANIFEST="$CONFIG_DIR/worlds_manifest.yaml"
METRICS_PARAMS="$CONFIG_DIR/metrics_params.yaml"
PLOT="$SCRIPT_DIR/plot_results.py"

WORKSPACE="${WORKSPACE:-/workspace}"
RESULTS_DIR="$WORKSPACE/results"
LOG_ROOT="$RESULTS_DIR/logs"
CSV="$RESULTS_DIR/metrics.csv"
FLAG="$RESULTS_DIR/.run_complete"
SUMMARY="$RESULTS_DIR/benchmark_summary.txt"
BAG_DIR="$WORKSPACE/media/bags"
PLOT_DIR="$WORKSPACE/media/plots"
CSV_HEADER="timestamp,world,run_index,mission_result,coverage_pct,collisions,duration_s,distance_m"

RUNS=3
TIMEOUT=480
WANT_BAG=0
WORLD_ARG="--all"

log()  { echo "[$(date +%H:%M:%S)] $*" | tee -a "$SUMMARY"; }
die()  { echo "FATAL: $*" >&2; exit 1; }

# --------------------------------------------------------------------------- #
while [ $# -gt 0 ]; do
  case "$1" in
    --all)     WORLD_ARG="--all"; shift ;;
    --world)   WORLD_ARG="$2"; shift 2 ;;
    --runs)    RUNS="$2"; shift 2 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    --bag)     WANT_BAG=1; shift ;;
    *) die "unknown argument: $1" ;;
  esac
done

command -v ros2 >/dev/null 2>&1 || die "ros2 not on PATH — did you 'source install/setup.bash'?"
[ -f "$MANIFEST" ] || die "manifest not found: $MANIFEST"
cd "$WORKSPACE" || die "cannot cd to workspace $WORKSPACE"
mkdir -p "$RESULTS_DIR" "$LOG_ROOT" "$BAG_DIR" "$PLOT_DIR"

# --------------------------------------------------------------------------- #
# Manifest helpers (PyYAML ships with the ROS image).
manifest_worlds() {
  python3 -c "import yaml,sys;print(' '.join(yaml.safe_load(open(sys.argv[1]))['benchmark_worlds']))" "$MANIFEST"
}
load_world() {  # exports W_SDF W_MAP W_PARAMS W_SX W_SY W_GMINX W_GMAXX W_GMINY W_GMAXY
  local w="$1"
  eval "$(python3 - "$MANIFEST" "$w" <<'PY'
import sys, yaml
m = yaml.safe_load(open(sys.argv[1]))
w = m['worlds'][sys.argv[2]]
print(f"W_SDF={w['sdf']}")
print(f"W_MAP={w['map']}")
print(f"W_PARAMS={w['planner_params']}")
print(f"W_SX={w['spawn_x']}")
print(f"W_SY={w['spawn_y']}")
print(f"W_GMINX={w['glass_min_x']}")
print(f"W_GMAXX={w['glass_max_x']}")
print(f"W_GMINY={w['glass_min_y']}")
print(f"W_GMAXY={w['glass_max_y']}")
PY
)"
}

# --------------------------------------------------------------------------- #
cleanup_processes() {
  # ros2 launch + ign double-fork, so kill by name. SIGINT first (graceful —
  # lets metrics_node's finally-block flush a row), then SIGKILL stragglers.
  pkill -INT -f 'metrics_node'                                  2>/dev/null
  pkill -INT -f 'ros2 bag record|rosbag2'                       2>/dev/null
  sleep 2
  pkill -INT -f 'coverage_planner|path_follower|cleaning_controller' 2>/dev/null
  pkill -INT -f 'camera_node|frame_detector|dirt_segmenter'     2>/dev/null
  sleep 2
  pkill -INT -f 'lifecycle_manager|map_server|controller_server|planner_server|bt_navigator|behavior_server|smoother_server' 2>/dev/null
  pkill -INT -f 'parameter_bridge|robot_state_publisher|static_transform_publisher' 2>/dev/null
  pkill -INT -f 'ign gazebo|ruby .*ign|gz sim'                  2>/dev/null
  sleep 3
  pkill -KILL -f 'metrics_node|coverage_planner|path_follower|cleaning_controller|camera_node|frame_detector|dirt_segmenter|lifecycle_manager|nav2|parameter_bridge|robot_state_publisher|static_transform_publisher|ign gazebo|ruby .*ign|gz sim|ros2 bag record|rosbag2' 2>/dev/null
  # Clear leftover DDS shared memory / ros tmp so the next run starts clean.
  rm -rf /dev/shm/* 2>/dev/null
  rm -rf /tmp/.ros  2>/dev/null
  sleep 2
  return 0
}
trap 'echo "[trap] cleaning up..."; cleanup_processes' EXIT INT TERM

wait_for_topic() {  # topic, timeout_s
  local topic="$1" deadline=$(( $(date +%s) + ${2:-90} ))
  while [ "$(date +%s)" -lt "$deadline" ]; do
    if timeout 5 ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      return 0
    fi
    sleep 3
  done
  return 1
}
wait_for_log() {  # file, pattern, timeout_s
  local file="$1" pat="$2" deadline=$(( $(date +%s) + ${3:-120} ))
  while [ "$(date +%s)" -lt "$deadline" ]; do
    [ -f "$file" ] && grep -q "$pat" "$file" 2>/dev/null && return 0
    sleep 3
  done
  return 1
}

write_synthetic_row() {  # world, run, result
  [ -s "$CSV" ] || echo "$CSV_HEADER" >> "$CSV"
  echo "$(date -Iseconds),$1,$2,$3,0.00,0,0.0,0.000" >> "$CSV"
}

# --------------------------------------------------------------------------- #
run_one() {  # world, run_index
  local world="$1" run="$2"
  local rl="$LOG_ROOT/${world}_run${run}"
  mkdir -p "$rl"
  rm -f "$FLAG"
  load_world "$world"

  log "=== $world run $run — spawn=($W_SX,$W_SY) sdf=$W_SDF ==="
  cleanup_processes
  sleep 3

  ros2 launch window_cleaner_bringup sim.launch.py \
      world:="$WORKSPACE/$W_SDF" x:="$W_SX" y:="$W_SY" z:=0.05 \
      gui:=false rviz:=false foxglove:=false \
      > "$rl/sim.log" 2>&1 &
  if ! wait_for_topic /odom 120; then
    log "  /odom never appeared — sim failed to start; HARD_TIMEOUT row"
    write_synthetic_row "$world" "$run" "HARD_TIMEOUT"
    cleanup_processes; sleep 5; return
  fi

  ros2 launch window_cleaner_bringup nav2.launch.py \
      map:="$WORKSPACE/$W_MAP" \
      odom_offset_x:="$W_SX" odom_offset_y:="$W_SY" \
      > "$rl/nav2.log" 2>&1 &
  wait_for_log "$rl/nav2.log" "Managed nodes are active" 120 \
    || log "  (nav2 'active' banner not seen — continuing; path_follower waits up to 30 s for the action server)"

  ros2 launch window_cleaner_perception perception.launch.py \
      > "$rl/perception.log" 2>&1 &
  sleep 3

  # metrics_node BEFORE the planner so it catches the first RUNNING.
  ros2 run window_cleaner_evaluation metrics_node --ros-args \
      --params-file "$METRICS_PARAMS" \
      -p use_sim_time:=true \
      -p world_name:="$world" -p run_index:="$run" \
      -p results_dir:="$RESULTS_DIR" \
      -p glass_min_x:="$W_GMINX" -p glass_max_x:="$W_GMAXX" \
      -p glass_min_y:="$W_GMINY" -p glass_max_y:="$W_GMAXY" \
      > "$rl/metrics.log" 2>&1 &
  sleep 2

  if [ "$WANT_BAG" -eq 1 ] && [ "$run" -eq 1 ]; then
    ros2 bag record -o "$BAG_DIR/${world}_run1" \
        /robot/camera/image_raw /robot/scan /odom /tf /tf_static \
        /planning/coverage_path /perception/dirty_regions \
        /control/mission_state /control/state \
        > "$rl/bag.log" 2>&1 &
  fi

  ros2 run window_cleaner_planning coverage_planner --ros-args \
      --params-file "$WORKSPACE/$W_PARAMS" -p use_sim_time:=true \
      > "$rl/planner.log" 2>&1 &
  sleep 2
  ros2 run window_cleaner_planning path_follower --ros-args \
      -p use_sim_time:=true > "$rl/follower.log" 2>&1 &
  ros2 launch window_cleaner_control control.launch.py \
      > "$rl/control.log" 2>&1 &

  # Wait for metrics_node to finalise (it owns the truth: DONE/ABORTED via
  # mission_state, or its own sim TIMEOUT watchdog). The flag file is robust
  # where `ros2 topic echo` would hang on a dead DDS.
  local deadline=$(( $(date +%s) + TIMEOUT )) result=""
  while [ "$(date +%s)" -lt "$deadline" ]; do
    if [ -f "$FLAG" ]; then result="$(cat "$FLAG" 2>/dev/null)"; break; fi
    sleep 5
  done
  if [ -z "$result" ]; then
    log "  HARD wall-clock timeout (${TIMEOUT}s) — metrics_node did not finalise"
    # SIGINT metrics_node so its finally-block writes an INTERRUPTED row;
    # if even that fails, synthesise one so the CSV stays complete.
    pkill -INT -f 'metrics_node' 2>/dev/null
    sleep 8
    if [ -f "$FLAG" ]; then result="$(cat "$FLAG")"; else
      result="HARD_TIMEOUT"; write_synthetic_row "$world" "$run" "$result"
    fi
  fi
  log "  -> result=$result"

  cleanup_processes
  sleep 5
}

# --------------------------------------------------------------------------- #
if [ "$WORLD_ARG" = "--all" ]; then
  WORLDS="$(manifest_worlds)"
else
  WORLDS="$WORLD_ARG"
fi
[ -n "$WORLDS" ] || die "no worlds resolved"

log "benchmark start — worlds=[$WORLDS] runs=$RUNS timeout=${TIMEOUT}s bag=$WANT_BAG"
for world in $WORLDS; do
  for run in $(seq 1 "$RUNS"); do
    run_one "$world" "$run"
  done
done

log "benchmark done — generating plots"
python3 "$PLOT" --csv "$CSV" --outdir "$PLOT_DIR" \
  >> "$SUMMARY" 2>&1 || log "  (plot_results failed — run it manually on $CSV)"
log "ALL DONE. CSV=$CSV  plots=$PLOT_DIR"
echo "==== metrics.csv ===="; cat "$CSV" 2>/dev/null || true
