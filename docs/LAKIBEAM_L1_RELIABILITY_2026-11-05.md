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

## Evidence update: controlled board measurements, 2026-11-05

The device GET confirms model LakiBeam1, 30 Hz, 1800 RPM, 0.25-degree
angular resolution, 45..315 degree physical window, actual filter level 3,
min_angle=10, max_angle=170, window=90, neighbors=0.

User reports the second wall was at ACTUAL 1.95m, although the CLI was run
with 2.00m. Reported median residual -0.0327m under the wrong reference
becomes approximately +0.0175m under actual geometry, conditional on samples
really hitting the board. The original all-bin 0.1593m RMSE is NOT a proven
16cm instrument error: board edges, background and mixed aggregate bin ranges
can dominate the statistic. Forward finite fraction was 0.8801 for +/-10deg
and ~0.815 for +/-20deg; samples per scan 1440 all frames and scan ~30Hz.

The 1m experiment labeled lidar_open.json has 303 full 1440-point scans at
30.22Hz, front median finite fraction 0.9321, and known_wall_test=null
because the user omitted --front-wall-m 1.0. The original JSON retains
angle_data and can be analyzed offline.

Board geometry is essential: at 1.95m a perpendicular white target centered
on the 0 degree ray requires >=1.42m width to fill +/-20deg. At 1.0m,
the same FOV requires >=0.73m. For +/-10deg at 1.95m width >=0.69m.
Without measured board width the coverage falloff from 1m to 1.95m
cannot be called intrinsic sensor dropout.

Upstream manufacturer specification LakiBeam1: 25m @ 70% reflectivity,
at least 15m @ 10% reflectivity, typical +/-2cm range accuracy. The
100m LaserScan metadata is a hardcoded driver value, not an instrument
capability.

Reanalyze both EXISTING JSON files without restarting ROS or moving car:

    cd ~/f1tenth_uj
    git pull --ff-only origin zaq/hardware-integration
    python3 tools/diagnostics/reanalyze_lidar_wall.py --self-test
    python3 tools/diagnostics/reanalyze_lidar_wall.py --file       ~/f1tenth_hw_evidence/lidar_white_wall_2m.json --distance-m 1.95
    python3 tools/diagnostics/reanalyze_lidar_wall.py --file       ~/f1tenth_hw_evidence/lidar_open.json --distance-m 1.0

If the exact board width is measured, pass --board-width-m WIDTH to either
command. This crops the angular ROI to what the board should actually cover,
with a 2 degree edge margin. Without this value, +/-5deg is assumed
for screening ONLY. The inlier grouping uses +/-10cm as a diagnostic
background exclusion criterion, not as a sensor accuracy or driving limit.

Next engineering gates: actual board width and angle, known white/dark
target test, possible filter LV3 versus LV1 or LV0 controlled comparison
when user elects to change LiDAR settings; retain conservative unknown
handling until sensor's dark-object behavior and physical stopping system
are tested. Do NOT replace zero-return/+inf by fake 25m free-space data.

## 2026-11-05: Direct engineering fix for false-positive 90% forward dropout

Observed offline calibration:
- Actual white-board 1.95m: central +/-5 degrees **100% measured**, corrected perpendicular residual median **+0.0217m**, RMSE **0.0208m**.
- User's approx 1m test: central +/-5 degrees **100% measured**, perpendicular residual median **-0.0200m**, RMSE **0.0193m**.
- Across the full +/-20 degrees the 1.95m board test had missing echoes, but board width was not provided. Without width or angular coverage, this is not proof of an out-of-range or failed sensor. The 25m nominal range has no bearing on a near 1m or 1.95m *central return*.
- 303/303 frames have full 1440 angular samples, 30.2Hz, consistent timestamps. Real firmware filter level 3 is configured; no filter changes were required for the central measurement.

The immediate software defect was that the physical exploration **proposal** used the same global >=90% finite-ray requirement across +/-20 degrees as a motion precondition. That turns legitimate no-return rays outside a narrow target into an all-front STOP even if the measured central sector is clean.

Code now calls proposal_front_guard() from tools/exploration/front_scan_guard.py for **non-actuating exploration proposals**:
- Require >=90% finite measured points in the central +/-5 degrees with no central object below 0.80m.
- Independently reject ANY finite measured object closer than 0.80m anywhere in +/-20 degrees.
- All invalid/inf rays remain unknown, represented as zero in the existing exploration gap selection. Never convert +inf into a fabricated 25m distance.
- Simulator behavior unchanged.
- **IMPORTANT** This is ONLY a less misleading proposal-availability criterion. A central +/-5 degrees region does NOT span the physical car's full swept footprint, especially in turns. Therefore the separate real actuator mux STILL requires >=90% across +/-20 degrees and will not authorize movement merely because proposals are available. This patch does NOT certify safe driving.

Regression test:

    cd ~/f1tenth_uj
    git pull --ff-only origin zaq/hardware-integration
    python3 tools/exploration/test_proposal_front_guard.py
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select f1tenth_bringup
    source install/local_setup.bash

Restart ONE launch, for ROS graph examination only:

    ros2 launch f1tenth_bringup real_integrated.launch.py       enable_real_actuation:=false       enable_autonomous_actuation:=true       enable_joystick:=false

In a second sourced terminal:

    ros2 topic echo /exploration/drive --once --field drive
    ros2 topic echo /control/status --once --field data

A nonzero exploration proposal now becomes possible on scans with valid central measured gap while real /drive stays zero. If the explorer remains at zero, its once-per-second logs identify obstacle/gap constraints; do not disable the full-footprint/motor gate to force movement.

Further motor commissioning must check VESC firmware limits, actual stopping distance and an independent cutoff. For powered autonomy, the next safety improvement is a physically validated swept-path/unknown-return policy rather than merely reusing a narrow central proposal test for propulsion.
