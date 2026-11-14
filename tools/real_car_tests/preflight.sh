#!/usr/bin/env bash
# Read-only preflight for real-car tests. Publishes nothing, starts no node.
#   preflight.sh clean    before launching: no ROS process may be running (ros2 daemon allowed)
#   preflight.sh stack    test stack running, runner NOT yet started
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
[ "$mode" = "stack" ] || { echo "usage: $0 clean|stack"; exit 2; }

echo "== nodes"
nodes=$(ros2 node list -a 2>/dev/null)
echo "$nodes"
dups=$(echo "$nodes" | sort | uniq -d)
if [ -n "$dups" ]; then echo "FAIL: duplicate node names:"; echo "$dups"; fail=1; fi

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
check /commands/motor/brake        0 1
check /commands/motor/current      0 1
check /commands/motor/duty_cycle   0 1
check /commands/motor/position     0 1
echo "== mux inputs (runner not started yet: nobody publishes)"
check /manual/drive                0 1
check /joy                         0 1
check /control/request_mode        0 1
check /exploration/drive           0 -
echo "== mux status"
timeout 3 ros2 topic echo --once /control/status 2>/dev/null \
    | python3 -c "import sys,json,yaml; s=json.loads(yaml.safe_load(sys.stdin.read())['data']); \
print({k: s.get(k) for k in ('mode','reason','real_actuation_enabled','max_speed_mps','max_steer_rad', \
'front_valid_fraction','nearest_front_m','scan_reason')})" \
    || { echo "FAIL: no /control/status"; fail=1; }

[ $fail = 0 ] && echo "OK: stack" || echo "FAIL: fix the above before running"
exit $fail
