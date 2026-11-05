# F1TENTH UJ — LakiBeam1(L) manufacturer manual findings

**Date:** 2026-11-05 (Jordan time)  
**Scope:** Interpret the user-provided `LakiBeam1(L) User Manual` (Richbeam, document RBDOC-UM-1001), compare with Jetson ROS 2 observations and official Richbeam ROS 2 driver. No settings were changed and no actuator commands were issued.

## Manufacturer specifications (manual p.9)

| Parameter | LakiBeam1 | LakiBeam1L |
|---|---|---|
| Nominal ranging at 70% reflectivity | 25 m | 40 m |
| Ranging at 10% reflectivity | >=15 m | >=20 m |

Both variants: direct time of flight, 270-degree horizontal scan, 10/20/25/30 Hz, 0.1 degree resolution at 10 Hz and 0.25 degree resolution at 20–30 Hz, typical +/-2 cm ranging accuracy, 940 nm Class 1 laser, typical <=2 W, 5V USB or 9–36V DC external feed (follow connection-specific manufacturer instructions).

At **30 Hz**, the observed **1440 beams/scan** is exactly compatible with 360/0.25 degrees in the ROS array, *but* the optical instrument has only 270 degrees useful angular coverage (manual p.29); it is wrong to treat every array entry as having useful field-of-view information.

**Do not interpret** `LaserScan.range_max=100.0 m` as real sensor range. Official driver hardcodes `range_min=0.0` and `range_max=100.0`, while the manufacturer lists 25 m / 40 m at 70% reflectivity depending on exact variant.

## No-return and filtering (manual pp.18–19, 21–24, 55–60)

Manual says MSOP packets carry distance plus RSSI (strongest and last returns); a zero value for last return indicates *no return data*. Richbeam ROS2 driver `src/lakibeam1_scan.cpp` explicitly converts a **zero raw distance** to **`+Inf`** in `/scan` and sets that beam intensity to zero. The output therefore means *unmeasured/no valid distance*, **not confirmed free space**. Official code reference: https://github.com/RichbeamTechnology/Lakibeam_ROS2_Driver/blob/main/src/lakibeam1_scan.cpp

Manual lists four onboard general filter modes (p.19):
- OFF / 0: no filter
- LV1 / 1: smoothing only
- LV2 / 2: scan-shadow (veiling-effect) filtering only
- LV3 / 3: smoothing + scan-shadow filters together

The official launch file sets default `filter` to 3. **Must read actual device settings** before assuming it is running filter=3. The manufacturer documents read-only APIs:
- `GET /api/v1/system/firmware` (model, serial, firmware versions; p.34)
- `GET /api/v1/sensor/overview` (scanning frequency, resolution, scan range, filter, target host; pp.42–43)
- `GET /api/v1/sensor/filter/level` (p.55)
- `GET /api/v1/sensor/filter/min_angle`, `GET /api/v1/sensor/filter/max_angle` (pp.56–57)

Read-only commands on Jetson, using *verified physical Ethernet* sensor address `192.168.198.2` (manual pp.14, 34, 42, 55):

```bash
for route in system/firmware sensor/overview sensor/filter/level \
             sensor/filter/min_angle sensor/filter/max_angle; do
    printf '\n=== %s ===\n' "$route"
    curl --max-time 4 -fsS "http://192.168.198.2/api/v1/$route" || true
    printf '\n'
done
```

These are GET only. **Do not issue PUT** to change filtering, scanner geometry or networking while mapping. If we later run an A/B comparison between the filter modes, record the baseline, keep the car stationary, change only one variable, retain the fail-closed physical interlock, and restore/verify the original configuration after the experiment.

## Laser footprint and mount effects (manual pp.27–30)

- Optical horizontal scan range: 45° to 315°, 270° total (device-local azimuth; confirm ROS driver zero/offset and actual mounting).
- Vertical emission alignment tolerance +/-1 degree relative to horizontal.
- Laser emission origin about 56.75 mm above the product bottom; the existing ROS `base_link -> laser` Z=0.150 m is a provisional external mounting estimate, not automatically the optical-origin measurement.
- Optical spot diameter approximately `6 mm + 8.5 mrad * range_m`: ~23 mm at 2 m, ~40 mm at 4 m; returns on narrow or edge objects can be inconsistent. Driver algorithm may remove apparent pointcloud shadows.

The vertical +/-1° tolerance corresponds to up to ~3.5 cm of height change at 2 m and ~8.7 cm at 5 m relative to an exactly horizontal beam. The figures are geometric estimates, not measured tilt of this particular unit. Nearby high-gloss floor, glass, thin poles, highly angled edges, occlusions, lens contamination or tilt may affect available returns; these are hypotheses to test, not conclusions.

Manufacturer pp.32–33 contain fault diagnosis and cover-lens maintenance guidance; follow optical-cleaning instructions only if lens is visibly dirty.

## What the physical ROS evidence establishes

Real Jetson temporal test 2026-11-05 10:02:03, saved at:
`~/f1tenth_hw_evidence/lidar_temporal_20261105_100203/coverage.json`

- 594 stationary scans; stable angular geometry.
- ±20° front median finite coverage: 1 scan=81.25%, 2=81.87%, 3=81.87%, 5=82.50%, 10=82.50%.
- Even ten scans did not achieve 90% measured angular coverage, indicating mostly persistent missing returns.
- p95 largest contiguous unmeasured arc: 1.5° (one scan), 1.25° (2/3/5/10 scans).
- Previous angle profile: center ±5° valid fraction 67.5%, with many short gaps; overall ±20° invalid fraction 18.75%.

At 2 m range, 1.25° spans approximately `2 m * tan(1.25°)=0.0436 m` transverse width. This does not prove what object sizes will be missed. +Inf gaps must not be converted into certified free space.

## Next controlled experiments, in order

1. **GET** firmware/model, filter mode, scan frequency and filter threshold metadata. No hardware changes.
2. Capture a baseline LiDAR angle and temporal-coverage profile looking at the existing scene.
3. Stationary controlled target test: large matte cardboard perpendicular to the LiDAR front ray at 1–2 m, *at measured laser height*, then repeat. Compare valid-return rates in ±5° and +/-20°; check true orientation. No motion commands.
4. If the target does not improve coverage, inspect optical window, scan azimuth offset/orientation, filter level, firmware configuration and driver packet handling.
5. Only after source diagnosis, consider an experiment altering **device filter mode one variable at a time**, while maintaining separate logs, restoring baseline and checking map/scan stability. Modifying internal sensor filtering can cause measurement artifacts and may increase false points.
6. Build a swept-footprint-aware local proposal evaluator using **current-scan measured obstacles plus explicit unknown space**, with conservative failure behavior; then revalidate in physical shadow mode and simulation. Maintain hardware actuation interlock.
7. Power stability, independent hardware emergency stop, motor firmware timeout, real steering angles and calibrated odometry remain **blocking gates before autonomous propulsion**.

**Conclusion:** The datasheet supports investigating internal filtering, surface/optical properties, sensor orientation and hardware geometry rather than lowering 90% validity to an arbitrary 80%. The current scan masks and 0.8 m stop-distance rule remain conservative *software diagnostics*, not proven physical collision safety.
