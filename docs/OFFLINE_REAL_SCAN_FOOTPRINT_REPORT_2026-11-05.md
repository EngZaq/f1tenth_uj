# F1TENTH UJ — Physical LiDAR baseline and offline footprint research

**2026-11-05, Jordan time.** Working branch `zaq/hardware-integration`. All experiments in this note are *read-only offline*; no physical actuator commands have been issued.

## Recorded physical ROS bag

Jetson-local path: `/home/ftenth/f1tenth_hw_evidence/real_exploration_baseline_20261105_101514/rosbag`.

- SQLite3 `rosbag_0.db3`, **18.8 MiB**, duration **43.137078477 s**.
- Total **5,174 messages**: `/scan` **1,279** (~29.65 Hz), `/odom_rf2o` **863** (~20.01 Hz), `/map` **10**, `/tf` **3,021**, `/tf_static` **1**.
- Recorder cleanly finalized; reported `metadata.yaml` present.
- Car remained stationary during data capture (no powered exploration). The bag was not uploaded to GitHub; evidence path is on the Jetson only.

## First offline front-edge comparison

`tools/exploration/offline_footprint_replay.py`, self-test PASS. Processed **256 subsampled scans** over ~43.0 s at `--every 5 --preview-m 0.50`.

- Old global +/-20° forward-scan >=90% finite-return rule approved **0.0039 (1/256)**.
- Forward valid fraction across processed scans: p10 **0.8750**, median **0.8812**, p90 **0.8875**.
- Front-edge-only candidate probe results:

| Steering angle (rad) | Median measured probe fraction | Median unknown | Fraction of scans with any measured hit | All samples measured fraction |
|---|---:|---:|---:|---:|
| -0.150 | 0.9636 | 0.0364 | 0.0000 | 0.0000 |
| -0.075 | 0.9091 | 0.0909 | 0.0000 | 0.0000 |
| 0.000 | 0.8909 | 0.1091 | 0.0000 | 0.0000 |
| +0.075 | 0.9636 | 0.0364 | 0.0000 | 0.0000 |
| +0.150 | 0.9818 | 0.0182 | 0.0000 | 0.0000 |

**Interpretation:** The old global front sector rule rejects nearly all candidate motion in this scene, whereas the raw measurement availability differs by steering direction. But none of the front-edge candidates achieved complete sampled-point measurement. The absence of observed close hits within sampled front-edge point rays **is not evidence of full swept-footprint clearance**. It is inappropriate to auto-select +0.15 rad from highest measurement coverage.

Report JSON: `/home/ftenth/f1tenth_hw_evidence/real_exploration_baseline_20261105_101514/offline_footprint_report.json`.

## New research improvement (not yet executed on Jetson)

`tools/exploration/offline_swept_footprint.py` samples an **inflated rectangular vehicle body**, including interior and boundary points, at multiple future bicycle-model poses for steering -0.150, -0.075, 0, +0.075 and +0.150 rad. Each sample is classified as:
- `observed_to_point`: finite range return beyond the sampled position on the selected ray;
- `unknown_ray`: missing/nonfinite ray — **never turned into free space**;
- `near_or_occluded_return`: observed return before or near the position;
- `already_under_body`: region occupied by the car in its initial pose, excluded from the *new* swept region statistics with an explicit limitation.

The algorithm is a *finite grid/ray sampling experiment*, **not a certified continuous vehicle sweep**, not low-obstacle protection, not a steering planner and does not publish actuator commands. The geometry is based on 0.260 m wheelbase, [-0.040,+0.305] m longitudinal body bounds, 0.0985 m half-width, 0.215 m forward laser mount with an additional **provisional** 0.05 m body margin. Neither servo tracking nor physical emergency stop is yet validated.

### First execution on Jetson (no need to start ROS nodes)

```bash
cd ~/f1tenth_uj
git pull --ff-only origin zaq/hardware-integration
source /opt/ros/humble/setup.bash
source ~/f1tenth_uj/install/local_setup.bash
python3 -m py_compile tools/exploration/offline_swept_footprint.py
python3 tools/exploration/offline_swept_footprint.py --self-test
RUN="$HOME/f1tenth_hw_evidence/real_exploration_baseline_20261105_101514"
python3 tools/exploration/offline_swept_footprint.py "$RUN/rosbag" \
  --every 5 --preview-m .50 \
  --output "$RUN/offline_full_body_probe.json" | tee "$RUN/offline_full_body_probe.log"
```

The next report must be interpreted as diagnostics and compared against the front-edge-only baseline. In particular, more interior/side samples may identify previously unexamined unknown or occluded points.

## Engineering gates

No powered autonomous operation until an independent stop is installed/verified, VESC watchdog behavior and safe electrical limits are validated, Jetson brownout risk investigated, steering and Ackermann geometry calibrated, and low-speed controlled drive commissioning is completed. Physical explorer remains **interlocked**. This experiment is intended to guide a later unknown-aware planner rather than bypass the interlock.
