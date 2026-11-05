# F1TENTH UJ — Physical LiDAR Exploration Shadow Audit

**Observed 2026-11-05**, using real Jetson, LakiBeam LiDAR, RF2O and SLAM Toolbox; **no actuator commands**. Branch: `zaq/hardware-integration`.

## Results of first 60-second audit (before controller guard fix)

Evidence stored **on Jetson**, not in this repository:
`/home/ftenth/f1tenth_hw_evidence/exploration_shadow_20261105_092813/decision_audit.json`.

| Metric | Observed |
| --- | ---: |
| LiDAR messages | 1800 (~30 Hz) |
| Exploration proposals | 1202 (~20 Hz) |
| RF2O messages | 1200 (~20 Hz) |
| Maps | 12 (~0.20 Hz) |
| Peak absolute proposed speed | 0.075 m/s |
| Peak absolute proposed steering | 0.150 rad |
| Fraction zero-speed proposals | 0.0832 |
| Median nearest valid front beam | 1.274 m |
| Proposed motion with auditor-invalid forward scan | **1102** |
| Other proposal violations | none observed |
| RF2O net XY | 0.018534 m (not independently validated motion) |
| SLAM known mapped area start/end | 10.93 m² / 10.93 m² |
| Transport | PASS |
| Independent shadow acceptance | **FAIL** |

The occupancy grid was saved successfully:
`/home/ftenth/f1tenth_hw_evidence/exploration_shadow_20261105_092813/initial_map.pgm`
and `initial_map.yaml` (161 × 109 at 0.05 m/cell). As the robot was not moving autonomously, no coverage growth was expected.

**Root cause found in code:** Physical `exploration_controller_v2.py` previously required only >=3 valid forward beams and used the 15th percentile of valid forward ranges to decide clearance. The independent checker required >=90% valid beams in +/-20 degrees and checked the minimum valid distance. Thus the controller could propose motion when most of the forward view was unknown. The old controller condition was insufficiently conservative; the magnitude of the discrepancy does *not* establish that the LiDAR hardware was faulty.

## Changes after the failed audit — awaiting new Jetson test

1. `tools/exploration/front_scan_guard.py`: ROS-independent check. Reject missing/bad geometry, fewer than eight beams in forward +/-20°, fewer than 90% finite in-range returns, and any valid return nearer than 0.8 m. Invalidate the proposal rather than infer free space.
2. `tools/exploration/exploration_controller_v2.py`: in `platform:=physical` shadow mode, require the new forward-sector condition **before** computing an exploration proposal. Simulator branch retains preexisting behavior pending separate regression. The **physical drive interlock remains intact**.
3. `tools/exploration/test_front_scan_guard.py`: 11 no-ROS/no-actuator cases for full data, missing rays, NaNs, infinites, obstacles, threshold and invalid geometry.
4. `tools/exploration/physical_exploration_audit.py`: still an independent checker; now logs per-scan valid-fraction distribution, counts of NaN, +Inf, -Inf, below-min and above-max beams, plus whether *any* movement proposal was possible.

**Important:** The auditor's `strict_shadow_acceptance_pass` can be TRUE while all proposed speeds are 0.0; that is only fail-closed behavior, **not progress toward autonomous mapping**. Inspect `moving_proposals`, `front_scan_validity` and `forward_sector_beam_tallies` before claiming exploration readiness.

### Retest

Keep only the existing real sensor observation launch running, without VESC actuation:

```bash
cd ~/f1tenth_uj
git pull --ff-only origin zaq/hardware-integration
source /opt/ros/humble/setup.bash
source ~/f1tenth_uj/install/local_setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 -m py_compile tools/exploration/front_scan_guard.py \
 tools/exploration/exploration_controller_v2.py \
 tools/exploration/test_front_scan_guard.py \
 tools/exploration/physical_exploration_audit.py
python3 tools/exploration/test_front_scan_guard.py
```

Restart the explorer process (do not leave the older controller still running):

```bash
python3 tools/exploration/exploration_controller_v2.py --ros-args \
 -p platform:=physical -p enable_drive:=false \
 -p scan_topic:=/scan -p proposal_topic:=/exploration/drive \
 -p wheelbase:=0.26 -p speed:=0.15 -p max_steer:=0.15 \
 -p lookahead:=0.9 -p stop_distance:=0.8 \
 -p scan_timeout:=0.25 -p front_fov_deg:=120.0 \
 -p safety_bubble_m:=0.35 -p min_front_valid_fraction:=0.90
```

In a separate sourced terminal:

```bash
cd ~/f1tenth_uj
OUT="$HOME/f1tenth_hw_evidence/exploration_shadow_retest_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
python3 tools/exploration/physical_exploration_audit.py \
 --duration 60 --output "$OUT/decision_audit.json" | tee "$OUT/decision_audit.log"
```

Interpretation: `front_validity.fraction_below_90pct` and `forward_sector_beam_tallies` identify the real distribution of gaps; `proposal_violations` should be empty or explicitly explained; `moving_proposals=0` indicates the algorithm is still blocked by sensor validity or nearby obstacles. Do **not** weaken thresholds simply to obtain nonzero commands. First identify whether missing beams are `+Inf` for max-range/no-return according to the driver (or true dropout) by comparing readings against a known target at safe distances.

## Jetson spontaneous shutdown investigation

At 2026-11-05 09:25 the Jetson reported just-restarted uptime. `journalctl -b -1` failed because persistent journal was not enabled. `dmesg` showed thermal governor and cooling device *registration* during boot, which is routine and **not evidence that an overtemperature event occurred**. Abrupt shutdown cause remains **unknown**; examine power wiring, supply output under load and thermal data, and enable persistent logs for future incident capture.

## Physical drive remains blocked

An RC remote is not required for *future* autonomous exploration, but an **independent verified stop** is still required. Controller-level stale-command response and safe electrical limits must also be verified. Robot steering travel/backlash and wheel-speed signs/scale remain uncalibrated. No current software patch authorizes drive or removes the existing `platform:=physical` interlock.

**Status:** Controller safety discrepancy identified and corrected in shadow mode; await retest and real scan classification.
