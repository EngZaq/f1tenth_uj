# F1TENTH real mapping: fast fallback when SLAM loop closing distorts the track
**2026-11-08 | physical development branch `zaq/hardware-integration`**

## What is actually failing?
There are two independent meanings of *closed loop*:
1. SLAM loop closure is a pose-graph optimization that attempts to recognize places revisited. It can be disabled.
2. A *geometrically closed observed track* is a driveable ring of known-free cells surrounding a central obstacle/interior. Our mission requires this even when SLAM loop closure is disabled.

Moatasem's Nov 2026 simulator results show false-looking graph corrections with loop closure ON: ~1.6m / 24 degrees and map wall RMS ~0.477m versus ~0.100m with loop closure OFF, three runs per condition. Tightening the matcher still failed. Do not transfer the faulty loop-on setting to real driving. The physical system must independently validate its own mapping geometry.

## Conservative fast path for a map (after tomorrow's ball-joint repair)
1. Fix ball joint, measure steering direction/range and independently confirm propulsion cutoff. Do not operate the car while the steering linkage is compromised.
2. Bring up only LiDAR, one RF2O TF publisher and SLAM; no VESC motor commands.
3. Explicitly set `enable_slam_loop_closing:=false` and verify the **live** parameter.
4. Manually guide/push around one entire track loop at slow speed, then inspect the map in RViz. If the return seam is not observed, continue a second observation pass only if the map alignment remains plausible.
5. Save the raw occupancy map **even if the lap closure detector does not pass**. This is diagnostic evidence, *not an accepted racing map*. Do not trick the map finalizer by setting `closed=True`.
6. If the observed corridor closes, calculate the physical map centerline and validate actual vehicle footprint/turning radius before permitting autonomous racing.
7. If map deforms even with loop closing OFF, investigate RF2O yaw drift on turns, timestamp gaps, double TF authority, frame conventions and actual LiDAR x/y/yaw mounting. Do not "solve" a sensor/TF fault by changing optimizer parameters.

## Terminal setup and source overlay
```bash
cd ~/f1tenth_uj
git status --short
git fetch origin zaq/hardware-integration
git switch zaq/hardware-integration
git pull --ff-only origin zaq/hardware-integration
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select f1tenth_bringup
source install/local_setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 pkg prefix f1tenth_bringup
```
Each new terminal needs both `source` commands and the ROS domain/RMW environment. No `/scan` or `/odom_rf2o` topics exist until launch starts.

## Mapping-only launcher (NO actuation)
Once hardware steering is repaired and the car may be safely guided:
```bash
ros2 launch f1tenth_bringup observe_only.launch.py \
  enable_vesc:=false \
  enable_slam_loop_closing:=false
```
In a separately sourced terminal:
```bash
ros2 param get /slam_toolbox do_loop_closing
ros2 topic hz /scan
ros2 topic hz /odom_rf2o
ros2 topic info -v /map
```
Expect `do_loop_closing=false`. If any required input is absent, do not start the loop.

Record evidence before moving (in another sourced terminal):
```bash
mkdir -p ~/f1tenth_hw_evidence
ros2 bag record -o ~/f1tenth_hw_evidence/loopoff_$(date +%Y%m%d_%H%M%S) \
  /scan /odom_rf2o /map /tf /tf_static
```
After one guided lap while the original launcher is still active:
```bash
mkdir -p ~/f1tenth_hw_evidence/loopoff_map
ros2 run nav2_map_server map_saver_cli -t /map \
  -f ~/f1tenth_hw_evidence/loopoff_map/new_track_diagnostic
```
The YAML/PGM may be useful even if the topology gate fails, but then the map is explicitly diagnostic. Inspect whether the lane returns to the correct wall, whether occupied walls are doubled or smeared, and whether unknown cells remain across the intended track.

## Optional non-actuating full-lap gate
While mapping-only bringup is active, in another sourced terminal, start the same first-lap logic that the real autonomy stack uses:
```bash
cd ~/f1tenth_uj
python3 src/f1tenth_bringup/scripts/physical_first_lap.py \
  --ros-args -p min_travel_m:=22.0
```
**22 m is only the current default**; use measured lap length and do not relax departure, return, or map topology rules to claim completion. The monitor now reports `completion_blockers` in `/mission/physical_lap_progress` and the `physical_first_lap_progress.json` evidence file.
```bash
ros2 topic echo /mission/physical_lap_progress --once --field data
```
Possible blockers: NOT_LEFT_START_AREA, INSUFFICIENT_TRAVEL, INSUFFICIENT_DURATION, NOT_RETURNED_TO_START, HEADING_NOT_ALIGNED_AT_START, OBSERVED_CORRIDOR_NOT_CLOSED, NO_FRESH_MAP.

## Why an actual return may still fail the topological gate
`map_coverage.py` only counts actually observed free cells. It then erodes the free region using **0.30m clearance** and requires a fully surrounded unknown/occupied interior at least **0.5m²**. A corridor narrower than roughly **0.60m** can disappear under this erosion, and an incomplete wall/unknown gap can leave no enclosed hole. Changing this threshold without a proper measured body-clearance audit can be unsafe; do not forcibly set `closed=True`.

The first-lap node also requires (defaults): 22m RF2O odometry travel, 2.5m departure, 40s elapsed, <=0.85m return-to-start, <=48deg heading difference and fresh /map (<8s). For a real track of different perimeter, set the *measured* travel minimum appropriately; keep the geometric and freshness validation.

## Failure decision tree
- **Map clean, corridor closed, first-lap not complete:** inspect `completion_blockers` (odometry travel, heading/start return or threshold mismatch).
- **Map clean, corridor not topologically closed:** inspect free-space sampling, remaining unknown gap and 0.30m erosion. Observe a second pass; do NOT invent unknown free cells.
- **Map bends/jumps only when loop closing ON:** retain OFF as the short-term containment; teammate's results show matcher-threshold tightening alone is inadequate.
- **Map bends even when loop closing OFF:** verify exactly one `odom->base_link` TF, one `map->odom` TF, laser-to-base x/y/yaw alignment, RF2O turns and time stamps. A closed loop optimizer cannot correct a broken frame chain reliably.
- **No ROS topics/package:** source overlay, build once, then launch the mapping stack. An empty ROS graph without a launch is normal.

A partial map, simulated-map replacement, hand-drawn wall completion, or manually prebuilt track is a valid *engineering diagnostic/demonstration artifact* only when labeled honestly. None proves the actual unknown-track autonomous mapping objective.
