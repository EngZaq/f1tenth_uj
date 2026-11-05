# LakiBeam1 field reliability diagnosis | 2026-11-05

Branch: zaq/hardware-integration. This is an evidence-based hardware validation protocol, not clearance for real propulsion.

## Sources reviewed

- Richbeam official product specifications: https://www.richbeam.com/product/LakiBeam
- Richbeam upstream ROS 2 driver source: https://github.com/RichbeamTechnology/Lakibeam_ROS2_Driver/blob/main/src/lakibeam1_scan.cpp
- Richbeam official ROS 2 launch: https://github.com/RichbeamTechnology/Lakibeam_ROS2_Driver/blob/main/launch/lakibeam1_scan.launch.py
- LakiBeam user manual v1.8.1: https://cdn.robotshop.com/media/R/RCH/RBC-Rch-01/pdf/single-line-lidar-dtof-industry-grade-lakibeam1-user-manual.pdf
- ROS LaserScan contract: https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg

The assumed LakiBeam1 supports >=25 m at 90% reflectivity and >=15 m at 10% reflectivity, 270-degree FOV, 30 Hz, roughly 0.25-degree resolution at 30 Hz. THE MODEL MUST BE CONFIRMED BY LIVE DEVICE FIRMWARE READBACK, because L1S/L1L have different ranges.

## Proven upstream driver behavior

From Richbeam ROS 2 src/lakibeam1_scan.cpp:

    scan.range_min = 0.0;
    scan.range_max = 100.0;
    scan.ranges[i] = (float)scan_vec[i].dist / 1000;
    if (scan.ranges[i] == 0) {
        scan.ranges[i] = inf;
        scan.intensities[i] = 0;
    }

Therefore a ROS +inf means the received raw strongest-return distance was zero. It does NOT prove a clear region out to 25 m or beyond. Raw zero can result from out-of-range, weak/missing return, filtering, or environment. Those causes are not encoded in LaserScan. The 100 m range_max was hardcoded in software; it is NOT the published physical detection distance. Do not blanket replace +inf with 25 m.

Upstream also builds a 360-degree LaserScan angle vector from the number of received samples (angle step 360/N), even though the sensor optical FOV is 270 degrees; if UDP packets are lost or a scan is truncated, angular geometry may be wrong. It uses strongest echo only; the raw UDP packet also carries a last return. Verify the installed Jetson driver is the same source revision before patching.

In upstream C++, sensor configuration invocation is commented out: // scan_config(). Thus launch parameters for filter, scan window and scan frequency may NOT be applied to the device. Read back HTTP device status instead.

Manual specifies filter 0=off, LV1=smoothing, LV2=scan-shadow, LV3=both. A shadow filter may remove edge points. Actual filter MUST be read from device. No filter changes without an A/B recorded test.

## Actual observation

At ~30 Hz, 154 scans. Front +/-20 degrees: 118/160 finite ranges, 42/160 +inf, no measured object closer than 0.80 m; closest finite 1.247 m. Consistent 72-76% finite during sample. Earlier scan from previous surroundings reported an object at ~0.36 m, which is a separate measured obstacle condition.

The explorer is stopped by 90% finite coverage; lowering 80 cm to 20 cm DOES NOT resolve the actual rejection.

## Jetson read-only investigation

Stop the continuously published AUTO hold; keep enable_real_actuation false and motor safely isolated.

    cd ~/f1tenth_uj
    git pull --ff-only origin zaq/hardware-integration
    source /opt/ros/humble/setup.bash
    source install/local_setup.bash
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python3 tools/diagnostics/lakibeam_reliability_audit.py --self-test
    python3 tools/diagnostics/lakibeam_reliability_audit.py --seconds 10 --out ~/f1tenth_hw_evidence/lidar_open.json

This tool subscribes to /scan and issues device HTTP GET requests ONLY, to 192.168.198.2. It records actual model, motor rpm, scan rate, filter setting, scan window, scan length histogram (expected 1440 samples at 30 Hz), frequency/gaps, ROS timestamp age, and per-half-degree persistent/intermittent +inf sectors. No changes to LiDAR or VESC.

Read installed driver path:

    ros2 pkg prefix lakibeam1
    ps -eo pid,ppid,args | grep '[l]akibeam1_scan_node'
    ip route get 192.168.198.2
    ip -s link

Device status API documented by manufacturer:

    curl --noproxy '*' --max-time 3 http://192.168.198.2/api/v1/sensor/overview
    curl --noproxy '*' --max-time 3 http://192.168.198.2/api/v1/sensor/filter

Note: source and actual device may differ from upstream; the GET responses establish what is truly configured.

## Controlled stationary known-target test

With drive power isolated and body stationary, place a wide matte WHITE vertical board at 1 m from laser optical origin, crossing the horizontal scan plane. A width of at least 0.8 m covers +/-20 degrees at 1 m. Avoid shiny/mirrored material. Ensure no one is near a powered drivetrain. Capture:

    python3 tools/diagnostics/lakibeam_reliability_audit.py --seconds 10 --out ~/f1tenth_hw_evidence/lidar_white_1m.json

Move same board to 2 m, then optionally test a matte DARK board at 1 m, capturing each as separate output (rename --out). Also test target at physical front, +10 deg, -10 deg to verify frame sign and laser yaw; confirm known target detected at the expected ROS LaserScan angles.

Interpretation:
- With white wall, center bins should have finite ranges approximately 1/cos(angle) m if 0 degrees aligns with physical forward. Nearby ranges can vary due to sensor offset, surface and accuracy.
- White detected but black missing at same distance: material reflectivity / weak return, a potential unsafe blind spot.
- Persistent fixed missing wedge even with broad white target: mounting obstruction, scan window, laser yaw, or sensor filter.
- Intermittent holes at same stationary target: optical/filter instability, power or packet timing.
- Expected target at different ROS angle: physical laser orientation/inverted/offset or driver index mapping mismatch.
- Scan lengths other than 1440 at 30 Hz or irregular intervals: investigate UDP packet assembly and sample indexing. ROS 30 Hz alone does not ensure 1440 valid rays.
- Empty outdoor space with +inf could be beyond range but still cannot certify near-zero-reflectivity obstacles are absent.

Also check clean optical window, supply 9-36 V for external DC, correct Ethernet wiring, no network interface drops, and laser scan plane height/angle. Manufacturer notes possible +/-1 degree vertical laser emission error; small low obstacles may not intersect this planar scan.

## Engineering fix after controlled evidence

Retain raw /scan and preserve unknown returns as UNKNOWN. Only after confirming firmware model and scan geometry consider a separate /scan_normalized adapter with accurate model-dependent metadata and no invented ranges. Never treat +inf as 25/100m of verified free space. Compare Nav2 obstacle clearing semantics (inf_is_valid) before activation.

Replace global 90% finite percentage with a validated swept-path reliability policy based on angular dropout streaks, temporal measured evidence and calibrated vehicle footprint, only after controlled obstacle checks. Use multiple sensing sources for low/black/transparent obstacles. Keep stale-scan and near-obstacle stop conditions. Set speed-dependent clearance using measured braking/coasting plus hardware independent E-stop. Firmware motor current limits remain separately unverified.

Current progress: public manufacturer code and manual analyzed; diagnostic committed. The reason for this exact car's +inf sector remains UNDETERMINED until actual device GET status plus known target test. Full autonomous actuation not authorized by this diagnosis.


## Optional white-wall quantitative calibration

After the same read-only audit command has succeeded, use a broad matte white wall 1.0m perpendicular to physical vehicle forward, measured from the optical origin, with the laser plane intersecting the wall:

    python3 tools/diagnostics/lakibeam_reliability_audit.py --seconds 10 \
      --front-wall-m 1.0 --out ~/f1tenth_hw_evidence/lidar_white_wall_1m.json

The script reports +inf frequency per half-degree, per-angle mean finite distance, mean strongest-echo RSSI, and the center +/-10-degree wall distance residual relative to 1.0/cos(angle) metres. This tests actual scan orientation as well as intensity and ranging precision. No motion or device settings change. The +/-10° residual assumes the wall is genuinely perpendicular to laser frame 0°, so a large residual is not necessarily a sensor hardware defect.

A second white-wall 2m run is useful:

    python3 tools/diagnostics/lakibeam_reliability_audit.py --seconds 10 \
      --front-wall-m 2.0 --out ~/f1tenth_hw_evidence/lidar_white_wall_2m.json

Do NOT run this known-wall calibration with the vehicle driving toward the wall. If center returns disappear or the wall appears at a wrong bearing, inspect physical sensor yaw / ROS angle indexing and firmware scan window before changing explorer code.
