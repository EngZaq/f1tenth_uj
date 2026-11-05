# F1TENTH UJ — Physical scan validity retest (2026-11-05 ~09:47 local)

**Disposition:** Shadow exploration software transport PASS, fail-closed response PASS, *usable exploration progress FAIL/NOT YET DEMONSTRATED*. NO actuator commands.

## Hardware observations (user-provided Jetson output)
- Boot history after enabling persistent journal shows only one aggregated record, starting 2026-07-29. Previous shutdown cause cannot be reconstructed from missing previous-boot journal; Jetson runtime at first check had just restarted.
- tegrastats ~09:48: SoC and CPU temperature ~48–50°C, reported `VDD_IN` ~5.2 **watts** idle. This is power draw, **not supply voltage**. Nothing here proves thermal shutdown or electrical supply stability; investigate abrupt power loss under load.
- LiDAR profiles: 1780 scans (~29.7 Hz); 1202 exploration proposals (~20 Hz); 1200 RF2O messages (~20 Hz); 12 map updates (~0.2 Hz).
- median front valid measured nearest range 2.34 m.
- Forward scan validity (±20° about laser forward): p10=0.80, median=0.8125, p90=0.8125, min=0.7875, max=0.825. **Every scanned frame below the previously required 0.90 valid-fraction threshold**.
- Ray classification in ±20° region for all 1780 scans:
  - 230,368 finite valid measurements (~80.89%).
  - 54,432 `+inf` (~19.11%).
  - zero NaN / negative infinity / below-min / above-max measurements.
  - Around 160 forward rays/scan (40° coverage).
- The controller correctly logged `front_insufficient_valid_returns` repeatedly, published speed=0 and steer=0. `proposal_violations={}`, `moving_proposals=0`; the independent audit reports PASS **because zero motion is fail-closed**, but this is not a movement-capable planner.
- Known map area 11.4225 m² at start and end (stationary); RF2O net displacement 0.00961 m, not ground-truth.
- JSON: `/home/ftenth/f1tenth_hw_evidence/exploration_shadow_retest_20261105_094728/decision_audit.json`.

## What is established and not established
The code discrepancy from the first run (allowing too few valid front beams) was fixed; the controller now blocks unknown forward data. Whether 19% +Inf represents open space beyond reliable returns, reflective surfaces, geometry of the environment, sensor optical limits, or driver encoding is **not known**. ROS LaserScan specifies that values outside range_min/range_max are discarded, while some ROS scan-conversion tools encode no-return beams as +Inf; **none of that by itself proves a corridor is free**. Do not change the 90% condition or reinterpret +Inf as guaranteed free space without inspecting spatial pattern and actual sensor behavior.

## New diagnostic on integration branch
`tools/exploration/lidar_angle_profile.py` (read-only):
- 5° bearing bins across -60° to +60°.
- +Inf and other invalid classifications per angular sector.
- closest valid finite return by sector.
- median fraction of valid near straight ahead (-5° to +5°).
- p50/p95 longest continuous invalid arc in -20°..+20°.
- report saved in JSON.
- `--self-test` checks classification without ROS.

### Run (keep `observe_only.launch.py enable_vesc:=false` active)
```bash
cd ~/f1tenth_uj
git pull --ff-only origin zaq/hardware-integration
source /opt/ros/humble/setup.bash
source ~/f1tenth_uj/install/local_setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 -m py_compile tools/exploration/lidar_angle_profile.py
python3 tools/exploration/lidar_angle_profile.py --self-test
OUT="$HOME/f1tenth_hw_evidence/lidar_angle_profile_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
python3 tools/exploration/lidar_angle_profile.py --duration 20 --output "$OUT/profile.json" | tee "$OUT/profile.log"
```

**Recommended controlled observation:** run one scan profile looking into current scene, another with a large matte, flat obstacle positioned around 1–2 m directly in front at LiDAR beam height; keep car stationary. Compare angular patterns. No physical movement required. Do not claim that open +Inf is proof of free space.

## Next engineering decision
Based on per-angle missing-data pattern, design a corridor-specific shadow proposal that requires:
- near-center and entire vehicle swept-footprint envelope sufficiently supported by measured returns,
- no measured obstacles in travel corridor,
- unknown rays excluded from certified free space (not blindly assigned maximum range),
- explicit stop on data staleness or poor coverage,
- measured steering range, independent stop, firmware stale-command behavior and power stability before any real actuation.
A future `strict_shadow_acceptance_pass` is necessary but insufficient; also require nonzero, valid proposals in a controlled clear environment to demonstrate feasible progress.

**Hardware drive interlock remains ACTIVE.**

## Angle-resolved evidence (user Jetson output ~09:55)

Full read-only profile, 593 scans, 1440 ranges/scan, `frame_id=laser`. Evidence JSON remains on Jetson at `~/f1tenth_hw_evidence/lidar_angle_profile_20261105_095538/profile.json`.

- Reported LaserScan metadata `range_min=0.0`, `range_max=100.0`. **Verified against Richbeam's official ROS 2 source**: `src/lakibeam1_scan.cpp` explicitly sets these constants and replaces every raw zero-distance entry with `+Inf`, setting its intensity to zero. Source: https://github.com/RichbeamTechnology/Lakibeam_ROS2_Driver/blob/main/src/lakibeam1_scan.cpp . Thus `+Inf` here means *raw distance zero/unmeasured*, not proven obstacle-free space and not evidence of a 100-meter effective LiDAR range.
- Median invalid fraction across forward +/-20° was **18.75%**, while the central +/-5° valid fraction was only **67.5%**.
- p50/p95 maximum consecutive missing angular arc in +/-20° was **1.25° / 1.50°**. The invalid rays appear fragmented over angle, not one large blind sector.
- Angular bin valid fractions: -15° to -10° =54.5%; -5°..0°=70.5%; 0°..+5°=65.0%; +5°..+10°=95.0%; +15°..+20°=96.9%.
- Closest finite observed 0°..+5°~5.189m, +15°..+20°~2.335m, +20°..+25°~1.169m, +50°..+55°~0.886m. These are **per-bin minima over 593 scans**, not a synchronized single-scan obstacle map. In ROS conventions **positive bearing is counter-clockwise**, nominally **left**, assuming the current `laser` alignment is accurate.
- The exact cause of zero distance is still unknown (surface reflectivity, angle, missed return, driver/data or environmental conditions). The accepted fail-closed controller is not yet producing autonomous progress.

## Next read-only diagnostic
The newly committed `tools/exploration/temporal_scan_coverage.py` compares per-angle measurement **coverage**, not free-space clearance, across 1, 2, 3, 5 and 10 consecutive scans, with no ROS publishers. Run **only with the car stationary**. It helps distinguish intermittent no-returns from persistent angular gaps. Historical readings cannot be used for physical collision avoidance without compensating for robot and obstacle motion and validating conservatism. A matte flat target near the central field of view can independently verify orientation and reveal whether +Inf decreases as a nearby surface becomes measurable.
