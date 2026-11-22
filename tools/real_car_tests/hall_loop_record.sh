#!/usr/bin/env bash
# Hall-loop recording for an offline slam_toolbox map (hall_loop_map.py), 23 Nov. HAND-PUSH path (the
# fallback); with teleop the runner's own bag (run_test.py --test teleop) is the recording.
#
# Stack (no VESC actuation; each in its own terminal, ROS sourced as in SESSION_HANDOFF section 7):
#   A: ros2 launch f1tenth_bringup observe_only.launch.py enable_vesc:=true
#        LiDAR + RF2O (+ live SLAM, loop closing off) + read-only VESC telemetry (/sensors/core, /odom_vesc_observe;
#        vesc_driver command inputs remapped to /observe_only_unused/*). observe_only does NOT start the IMU.
#   B: ros2 launch f1tenth_imu imu.launch.py          (car STILL ~10 s: gyro calibration; the node exits on motion)
#   C: tools/real_car_tests/hall_loop_record.sh       (this script)
#
# 1. Checks for ~5 s: /scan >= 25 Hz (~30), /imu/data_raw >= 90 Hz (~100) and not frozen, /odom_rf2o >= 10 Hz
#    (~20). Refuses with the reason if one is missing (22 Nov: LiDAR driver crashed once with exit -6; the IMU was
#    not started by observe_only).
# 2. Records /scan /imu/data_raw /odom_rf2o /tf /tf_static, plus whatever of /sensors/core /odom_vesc_observe
#    /control/status /t1b_stop_converter/status /manual/drive /sensors/servo_position_command is published (the C1
#    EKF replay needs /sensors/core + /imu/data_raw + /odom_rf2o), zstd per file, into
#    ~/f1tenth_hw_evidence/hall_loop_<YYYYmmdd_HHMMSS>/bag. Never /drive or /commands/* (the mux counts their
#    subscribers and latches ACTUATOR_GRAPH_OWNERSHIP_FAULT, 14 Nov).
# Ctrl+C stops the recording cleanly.
#   hall_loop_record.sh [--check-only] [--out-root DIR]
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
OUT_ROOT="${HOME}/f1tenth_hw_evidence"
CHECK_ONLY=0
while [ $# -gt 0 ]; do
  case "$1" in
    --check-only) CHECK_ONLY=1 ;;
    --out-root) OUT_ROOT="$2"; shift ;;
    -h|--help) sed -n '2,26p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done
command -v ros2 >/dev/null || { echo "REFUSED: ros2 not found: source /opt/ros/humble/setup.bash and install/setup.bash"; exit 2; }

REQUIRED="/scan /imu/data_raw /odom_rf2o /tf /tf_static"
OPTIONAL="/sensors/core /odom_vesc_observe /control/status /t1b_stop_converter/status /manual/drive /sensors/servo_position_command"
echo "Checking sensors for 5 s (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}, ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}) ..."
set +e
CHECK_OUT="$(python3 "$HERE/hall_loop_map.py" --sensor-check $OPTIONAL)"
RC=$?
set -e
echo "$CHECK_OUT" | grep -v '^OPTIONAL_PRESENT' || true
if [ $RC -ne 0 ]; then
  echo "REFUSED: fix the sensors above, then run this script again (nothing recorded)."
  exit 2
fi
PRESENT="$(echo "$CHECK_OUT" | sed -n 's/^OPTIONAL_PRESENT //p')"
TOPICS="$REQUIRED $PRESENT"
for t in $TOPICS; do
  case "$t" in /drive|/drive_calibrated|/commands/*) echo "REFUSED: $t must never be recorded (mux subscriber count)"; exit 2 ;; esac
done
[ $CHECK_ONLY -eq 1 ] && { echo "Check only: would record $TOPICS"; exit 0; }

OUT="$OUT_ROOT/hall_loop_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
{
  echo "hall_loop recording $(date -Iseconds)"
  echo "git $(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo '?')"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}"
  echo "topics: $TOPICS"
  echo "$CHECK_OUT"
} > "$OUT/info.txt"

cat <<EOF

=====================================================================================
 RECORDING -> $OUT/bag
   1. Car on the START TAPE MARK, hands off: keep it STILL for 10 s (start reference).
   2. Push 2 laps SLOWLY (walking pace, ~0.3-0.5 m/s), smooth turns, keep the LiDAR clear of your body.
   3. Stop exactly ON the start tape mark (same heading), hands off, still 5 s.
   4. Ctrl+C here.
 Then on the laptop:  python3 tools/real_car_tests/hall_loop_map.py $OUT
=====================================================================================
EOF
# shellcheck disable=SC2086
ros2 bag record -o "$OUT/bag" --compression-mode file --compression-format zstd $TOPICS > "$OUT/bag.log" 2>&1 &
BAG=$!
trap 'kill -INT $BAG 2>/dev/null || true' INT TERM
wait $BAG || true
wait $BAG 2>/dev/null || true
trap - INT TERM
echo "Recording stopped: $OUT ($(du -sh "$OUT/bag" 2>/dev/null | cut -f1))"
echo "Next (laptop, after copying the folder): python3 tools/real_car_tests/hall_loop_map.py $OUT"
