# Test terminals reached over Wi-Fi SSH (e.g. phone hotspot): keep ROS 2 / Cyclone DDS on the Jetson's loopback.
# 22 Nov: launching the stack while SSH ran over the hotspot froze every SSH session (DDS discovery and traffic on
# the Wi-Fi link). USB SSH (usb0) needs none of this: use the normal setup there.
#   source ~/f1tenth_uj/tools/real_car_tests/local_env.sh      (in EVERY terminal of the session, before ros2)
# Effect: nothing leaves the Jetson, so laptop RViz / ros2 on the laptop see nothing. Mixed terminals (some with,
# some without) do not see each other (the runner then says "No /control/status").
(return 0 2>/dev/null) || { echo "local_env.sh must be sourced: source $0"; exit 1; }
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1
unset CYCLONEDDS_URI                      # ~/cyclonedds_jetson.xml pins Wi-Fi peers; loopback discovery uses multicast
# Cyclone discovers over loopback multicast; without the MULTICAST flag on lo it fails with a participant-index
# error. The flag does not survive a reboot.
ip link show lo | grep -q MULTICAST || sudo ip link set lo multicast on
ros2 daemon stop >/dev/null 2>&1          # the CLI daemon keeps the old discovery settings; it restarts by itself
echo "local DDS: ROS_LOCALHOST_ONLY=1, CYCLONEDDS_URI unset, lo $(ip link show lo | grep -q MULTICAST && echo multicast on || echo 'multicast OFF')"
