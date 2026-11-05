# F1TENTH UJ — Fast path from real LiDAR to controlled physical exploration

**Date:** 2026-11-05; working branch `zaq/hardware-integration`. No physical actuation authorized or tested as part of this note.

## Latest results

From the Jetson ROS bag `~/f1tenth_hw_evidence/real_exploration_baseline_20261105_101514/rosbag`, duration 43.137 s, 1279 raw scans, 863 RF2O messages, 10 maps, 3021 TF messages, 1 static TF = 5174 total messages. The offline footprint experiment evaluated 256 scans with 0.50 m preview. It recorded median measured/unknown fractions across sampled full-body trajectory points:
- -0.150 rad: 0.9122 / 0.0859; measured near-or-occluded fraction 0.0018
- -0.075 rad: 0.8798 / 0.1202; near-or-occluded 0
- 0.000 rad: 0.8947 / 0.1053; near-or-occluded 0
- +0.075 rad: 0.8871 / 0.1129; near-or-occluded 0
- +0.150 rad: 0.8976 / 0.1005; measured near-or-occluded 0.0018

All had 0% snapshots with fully measured sampled points. Thus none is a validated collision-free candidate. These point samples and single planar LiDAR do not prove safety under motion.

## Fixed self-test issue

The old synthetic 3m-all-finite scan test incorrectly demanded *no unknown samples at all*. Two of 322 test points fall inside the algorithm's **2cm LiDAR-origin exclusion disk** and remain unknown by design. The code's actual conservative classification remains unchanged. The test now checks >99% measured fraction, preserves nonzero near-origin unknown and explicitly checks that `no_unmeasured_sampled_points` is false. Commit `459112f`. The corrected self-test and bag test **still require execution on Jetson** before being called passed.

## Practical live milestone added

`tools/exploration/live_shadow_paths.py`:
- Subscribes to the physical `/scan` (LaserScan sensor-data QoS).
- At ~5 Hz publishes 5 `nav_msgs/Path` visualizations on `/exploration/shadow_path_*`, respecting bicycle steering geometry.
- Publishes JSON `std_msgs/String` summary `/exploration/shadow_path_diagnostics`: per-candidate unknown/observed/occluded fractions, sample counts and stale-scan checks.
- **Never publishes any Ackermann, servo, VESC, arm, or enable command.** Visualization is a planning aid, **not motion certification**.
- Does not modify the existing physical drive interlock. ROS frame alignment and measured steering bounds remain unverified.

Use physical observed-only bringup `ros2 launch f1tenth_bringup observe_only.launch.py enable_vesc:=false` in one terminal. In another, pull the branch, then run:

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_uj/install/local_setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd ~/f1tenth_uj
git pull --ff-only origin zaq/hardware-integration
python3 -m py_compile tools/exploration/offline_swept_footprint.py tools/exploration/live_shadow_paths.py
python3 tools/exploration/offline_swept_footprint.py --self-test
python3 tools/exploration/live_shadow_paths.py --self-test
python3 tools/exploration/live_shadow_paths.py
```

In another sourced terminal verify `ros2 topic echo --once /exploration/shadow_path_diagnostics` and `ros2 topic list | grep /exploration/shadow_path_`. Use RViz2 Path displays and `Fixed Frame=base_link` to view the five candidate arcs. Keep motor drive disabled.

## Shortest safe path to first powered drive (hardware gates; not yet completed)

1. **E-stop/cutoff:** independent manually reachable hardware motor disable/cutoff with a second person at the stop and predictable reset; verify real VESC safe behavior independent of Jetson power or software. Without remote, cannot substitute Ctrl+C, a ROS node or a VESC USB disconnect for this.
2. **Jetson unexpected shutdown:** check wiring and supply under realistic load and verify motor shutdown on loss of control computer; persistent journaling is now configured but historical incident has not been diagnosed.
3. **VESC electrical limits:** current screenshots showed problematic unverified motor/battery/regen limits and 1000ms timeout with zero brake current; set *conservative* current/regen values for measured components with VESC Tool, read back them, document exact values. Do not use high limits from earlier screenshots by default.
4. **Actuator bench on elevated/secured wheels:** confirm neutral, forward polarity/ERPM, actual measured left/right maximum steering (including backlash), stale-command response, no auto-rearm. Avoid pushing software limits past physical binding.
5. **Supervised low-speed first movement:** only after steps 1–4: short straight motion in clear bounded area, battery power available, spotter at verified stop, slow velocity, limited travel distance. Compare actual distance/heading to VESC odometry and RF2O; map grows as physically moved.
6. **Autonomous exploration phases:** integrate a single validated command authority, stale LiDAR and map faults fail closed, unknown-space-aware map/footprint planner, measurable progress/lap tracking. Then enable longer first-lap exploration and map saving. Pure Pursuit and 1–5 m/s speed ramp belong to later validated racing, not first commissioning.

## Status

**Today demonstrated:** ROS bag capture; stationary LiDAR on hardware; SLAM map; high-rate sensor streams; offline candidate analysis; live visual path publisher *committed but not yet Jetson-tested*.

**Not demonstrated:** a validated moving obstacle-avoidance planner, emergency stop, safe powered drivetrain commissioning, reliable dynamic localization or autonomous physical track exploration.
