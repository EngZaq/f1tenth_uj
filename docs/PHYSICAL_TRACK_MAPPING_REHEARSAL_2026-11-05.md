# Physical-track map and route rehearsal — 2026-11-05

## Goal

Produce the **first real physical-track map** and nominal **body-audited centerline** with Jetson LiDAR + RF2O + SLAM, *without powered autonomous driving or modifying VESC firmware*. This is a manual-push lap of the physical robot, not an autonomous lap.

This advances the actual map -> topology gate -> map saving -> centerline extraction -> vehicle geometry audit pipeline originally built in simulation.

## What is new

- `physical_map_rehearsal.launch.py`: launch real LiDAR, LiDAR TF, RF2O odometry, SLAM Toolbox mapping with loop closing explicitly ON, the real first-lap watcher, and physical map finalizer. **NO VESC driver, Ackermann converter, joystick, explorer or physical drive mux.**
- `observe_only.launch.py` has a new optional `enable_slam_loop_closing` launch arg, default FALSE so previous use remains unchanged. Rehearsal sets TRUE.
- `physical_first_lap.py` now publishes `/mission/physical_lap_progress` and writes `~/f1tenth_hw_evidence/physical_first_lap_progress.json` every ~1 s with traveled distance, return-to-start distance, heading error, closed observed corridor, and completion eligibility.
- Rehearsal lap thresholds are conservative observation gates and are tunable from the launch: default travel >=12m, departure radius >1.5m, duration >=20s, return <0.85m, return yaw error <48deg, actual map corridor closed with >=0.30m clearance and a substantial interior. These thresholds do **not** certify that the physical robot can navigate this track safely. Original powered-intent integrated launch retains the original first-lap defaults >=22m, >2.5m, >=40s.
- Once complete, existing finalizer saves live SLAM `new_track.yaml` / `new_track.pgm`, extracts a closed centerline, then runs the existing real-body route optimizer. It writes `mission_status.json` with accepted/rejected results and step logs.

## Single workflow on Jetson

1. Stop existing integrated launch with Ctrl+C. **Physically isolate propulsion** before handling or manually pushing the robot. Power LiDAR and Jetson safely (do not assume a stopped ROS VESC driver makes motor electrically safe). The VESC is not used in this rehearsal.
2. Put car at the starting line of a real, closed course with visible LiDAR boundaries and enough width for the known physical dimensions; the course must have a clear loop around a substantial enclosed interior. Do not move it in public corridors.
3. Run one command:

```bash
cd ~/f1tenth_uj
git pull --ff-only origin zaq/hardware-integration
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select f1tenth_bringup
source install/local_setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch f1tenth_bringup physical_map_rehearsal.launch.py
```

4. In a separate sourced terminal, watch progress:

```bash
ros2 topic echo /mission/physical_lap_progress --field data
```

5. Slowly push the entire vehicle (with independent propulsion isolation) around a **closed physical track** while Jetson/LiDAR run. Return near the starting pose and heading. Keep scan range unobstructed and do not pick up and teleport the robot; RF2O only estimates motion from successive scans. A tiny circle around an isolated object is not representative of a race track. This is a mapping-only rehearsal and does not demonstrate autonomous driving.
6. When the existing geometry and observed map topology conditions pass, the watcher publishes `/mission/first_lap_complete=true`, then the finalizer saves and audits the actual track map and route. Output under a new timestamped `~/f1tenth_hw_evidence/physical_mapping_YYYYMMDD_HHMMSS` directory. `mission_status.json` ends in `COMPLETED_MAPPING_AND_PLANNING` only when all map/route checks actually accept. On failure, it writes `STOPPED` with a reason. Logs persist.

Optional quick result:
```bash
cat ~/f1tenth_hw_evidence/physical_first_lap_progress.json
find ~/f1tenth_hw_evidence -name mission_status.json -print
```

The diagnostic `/control/status` is **not** present in this rehearsal because the VESC mux is intentionally absent. Do not interpret this as an error.

## Important limits

- The real SLAM map can have poor loop closure, RF2O drift, false loop closures or topological artifacts; both route audit and later moving localization evidence are required. Loop closure enabled does not guarantee closure.
- Map features are observed with the real LiDAR, but there is no autonomous exploration, no wheel/motor speed validation and no proof of safe collision avoidance. A route accepted from a map remains nominal until localization, vehicle tracking, steering and stopping are physically validated.
- No VESC Tool settings are changed. Existing high and unverified motor/battery current values, long 1s timeout with zero timeout brake and missing independent physical cutoff still block powered ground autonomous driving. Do not bypass these to claim a milestone.
- >5m/s is a future design objective, not tested or configured physical vehicle speed. The known 8,414 ERPM/(m/s) scaling needs physical calibration; final firmware ERPM and safety limits depend on real hardware capabilities and track conditions.

## If completion does not trigger

Read the **one progress JSON** (not another LiDAR audit). Check `observed_corridor_closed`, `travel_m`, `return_distance_m`, `heading_return_error_deg`, and `map_age_s`. Don't lower obstacle clearance or convert unknown map cells to free just to produce a closed loop. If the real track is shorter than 12m, you may adjust only the **rehearsal** `minimum_travel_m` parameter after measuring the actual track length; the observed closed corridor remains mandatory.

No firmware or ROS motor clamp values were changed in this feature.
