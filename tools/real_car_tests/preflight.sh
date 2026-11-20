#!/usr/bin/env bash
# Read-only preflight for real-car tests. Publishes nothing; only short-lived subscribers
# (ros2 topic echo, and the 2 s IMU check below).
#   preflight.sh clean    before launching: no ROS process may be running (ros2 daemon allowed)
#   preflight.sh stack    test stack running, runner NOT yet started
#   preflight.sh stack t1b   same, with converter:=t1b (it alone publishes brake and current)
# Exit code 0 = OK, 1 = mismatch (do not continue).
source /opt/ros/humble/setup.bash
source "$HOME/f1tenth_uj/install/setup.bash"
set -u  # after sourcing: the ROS setup scripts use unset variables
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

mode="${1:-}"
fail=0

procs=$(pgrep -af "ros2|vesc|ackermann|mux|rf2o|slam|explor|ekf" \
        | grep -v -e "ros2cli.daemon" -e "pgrep" -e "preflight.sh")
echo "== processes (ros2 daemon hidden)"
echo "${procs:-<none>}"

if [ "$mode" = "clean" ]; then
    if [ -n "$procs" ]; then echo "FAIL: ROS processes still running"; exit 1; fi
    echo "OK: clean"; exit 0
fi
[ "$mode" = "stack" ] || { echo "usage: $0 clean|stack [t1b]"; exit 2; }
variant="${2:-}"
want_bc=0; [ "$variant" = "t1b" ] && want_bc=1

echo "== nodes"
nodes=$(ros2 node list -a 2>/dev/null)
echo "$nodes"
# F-14 (resolved 14 Nov): rf2o's CLaserOdometry2DNode holds a CLaserOdometry2D member that is
# itself an rclcpp::Node, and the launch remap __node:=rf2o_laser_odometry renames both. So ONE
# rf2o process shows the name twice. Allowed only in exactly that case; any other duplicate fails.
rf2o_procs=$(pgrep -fc "rf2o_laser_odometry/rf2o_laser_odometry_node")
dups=$(echo "$nodes" | sort | uniq -c | awk '$1 > 1 {print $1, $2}')
while read -r count name; do
    [ -z "$name" ] && continue
    if [ "$name" = "/rf2o_laser_odometry" ] && [ "$count" = 2 ] && [ "$rf2o_procs" = 1 ]; then
        echo "OK   duplicate $name x2 from one rf2o process (inner CLaserOdometry2D node, F-14)"
    else
        echo "FAIL duplicate node name: $name x$count (rf2o processes: $rf2o_procs)"; fail=1
    fi
done <<< "$dups"

# topic  expected_publishers  expected_subscribers ("-" = do not check)
check() {
    local info pubs subs
    info=$(ros2 topic info "$1" 2>/dev/null)
    pubs=$(echo "$info" | awk '/Publisher count/ {print $3}')
    subs=$(echo "$info" | awk '/Subscription count/ {print $3}')
    pubs=${pubs:-0}; subs=${subs:-0}
    local ok=OK
    if [ "$2" != "-" ] && [ "$pubs" != "$2" ]; then ok=FAIL; fi
    if [ "$3" != "-" ] && [ "$subs" != "$3" ]; then ok=FAIL; fi
    printf "%-4s %-28s pub %s (want %s)  sub %s (want %s)\n" "$ok" "$1" "$pubs" "$2" "$subs" "$3"
    [ "$ok" = OK ] || fail=1
}
echo "== actuator chain"
check /drive                       1 1
check /commands/motor/speed        1 1
check /commands/servo/position     1 1
check /commands/motor/brake        $want_bc 1
check /commands/motor/current      $want_bc 1
check /commands/motor/duty_cycle   0 1
check /commands/motor/position     0 1
echo "== mux inputs (runner not started yet: nobody publishes)"
check /manual/drive                0 1
check /joy                         0 1
check /control/request_mode        0 1
check /exploration/drive           0 -
echo "== mux status"
# `ros2 topic echo` truncates strings at 128 chars unless --full-length, and --once ends
# with a '---' document separator: take the first YAML document.
timeout 3 ros2 topic echo --once --full-length /control/status 2>/dev/null \
    | python3 -c "import sys,json,yaml; doc=next(yaml.safe_load_all(sys.stdin.read())); \
s=json.loads(doc['data']); \
print({k: s.get(k) for k in ('mode','reason','real_actuation_enabled','max_speed_mps','max_steer_rad', \
'front_valid_fraction','nearest_front_m','scan_reason','drive_subscriber_count','motor_subscriber_count')})" \
    || { echo "FAIL: no /control/status"; fail=1; }

echo "== IMU (/imu/data_raw, 2 s; 20 Nov: BMI160 froze after one I2C timeout, rate stayed 100 Hz)"
python3 - <<'PY' || fail=1
import os, sys, time
sys.path.insert(0, os.path.expanduser('~/f1tenth_uj/tools/real_car_tests'))
from run_test import ImuWatch   # same rule as the runner's arm refusal
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
rclpy.init()
node = rclpy.create_node('preflight_imu_check')
watch, n = ImuWatch(), [0]
def on_imu(m):
    g, a = m.angular_velocity, m.linear_acceleration
    watch.add(time.monotonic(), (g.x, g.y, g.z, a.x, a.y, a.z))
    n[0] += 1
node.create_subscription(Imu, '/imu/data_raw', on_imu, qos_profile_sensor_data)
t_end = time.monotonic() + 2.0
while time.monotonic() < t_end:
    rclpy.spin_once(node, timeout_sec=0.05)
problem = watch.status(time.monotonic())
node.destroy_node()
rclpy.shutdown()
print('%-4s /imu/data_raw %d msgs in 2 s: %s' % ('FAIL' if problem else 'OK', n[0], problem or 'changing samples'))
sys.exit(1 if problem else 0)
PY

[ $fail = 0 ] && echo "OK: stack" || echo "FAIL: fix the above before running"
exit $fail
