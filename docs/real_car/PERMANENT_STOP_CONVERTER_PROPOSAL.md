# Proposal: permanent stop fix for F-01 (our own Ackermann -> VESC converter)

For Zaid, from Moatasem (16 Nov 2026). **Proposal only: no code written yet.** Asks for your decisions in
section 7. Evidence: [REAL_CAR_TEST_RESULTS.md](REAL_CAR_TEST_RESULTS.md) (T1 close-out row),
handoff section 5.

## 1. Why

`ackermann_to_vesc` stops the car by sending `/commands/motor/speed 0`. On the stand that is not a stop:

| Stop method (T1, stand) | Steady stops | Failing (> 300 ERPM rebound) | Worst rebound rise | Worst settle < 100 ERPM |
|---|---|---|---|---|
| speed 0 (today's `ackermann_to_vesc`) | 2 runs | 4 of 5 | **7644 ERPM** (peak 7871, above the 4630 clip, so VESC-internal) | 0.513 s |
| **brake 5 A, 2 s hold, then current 0** | **23** at 0.5 / 1.0 / 1.5 m/s | **0** | 197 ERPM | 0.227 s |

T1 is closed on the stand with brake 5 A (Moatasem, 16 Nov). The rule from now on: **speed 0 is never
a stop command.** Today the brake exists only in the test-only `tools/real_car_tests/t1b_stop_converter.py`,
so `real_integrated.launch.py` still stops with speed 0. This proposal makes the tested behaviour the
real one.

## 2. What: a converter package of our own

A new package (proposed `src/hardware/uj_vesc_converter`, node `uj_ackermann_to_vesc`; name is your
call), not a fork: `vesc_ackermann` stays untouched. It **replaces** `ackermann_to_vesc` in the real
stack (never runs next to it) and keeps the same interface: subscribes `/drive`, publishes
`/commands/motor/*` and `/commands/servo/position`. Autonomy and the mux do not change.

The stop logic is the T1b `StopLogic`, unchanged in behaviour, because that is what was tested:

| State | Entered when | Output (one command per tick, 50 Hz) |
|---|---|---|
| `DRIVE` | fresh `/drive` (age < 0.2 s) with speed > 0 | `/commands/motor/speed = gain * v + offset` |
| `BRAKE_HOLD` | falling edge: speed request goes to 0 **or** `/drive` goes stale | `/commands/motor/brake = 5.0` (amps) for 2.0 s |
| `RELEASED` | hold over, or idle before any motion | `/commands/motor/current = 0` (motor released) |

- A new nonzero request in `BRAKE_HOLD` or `RELEASED` returns to `DRIVE` on the same tick.
- Brake only after a zero request, never while a speed is requested.
- Servo: `servo_offset + servo_gain * steer`; stale `/drive` -> straight ahead.
- Parameters come from `config/real_car/vehicle_params.yaml` (gain 8414, offset 0.320, servo gain
  -0.50, brake 5 A, hold 2.0 s). This also removes the 0.361 offset still in
  `src/f1tenth_bringup/config/ackermann_to_vesc.yaml` (F-17).
- Start-up validation as in T1b: brake in (0, 10] A, hold in (0, 5] s, otherwise the node refuses to
  start. On exit: current 0 (best effort, not a safe stop).
- 50 Hz output keeps the VESC firmware timeout from firing while the converter is alive.
- Status JSON topic (state, output, value, `/drive` age) for the bag, like `/t1b_stop_converter/status`.

### The brake hold (2 s, then current 0)

Every brake stop shows a small rise (77-101 ERPM) at +2.09-2.22 s: the moment the hold ends and the
converter switches to current 0. That stays far below 300 ERPM, so v1 keeps **2.0 s then current 0**
exactly as tested. Two consequences:
- After 2 s the motor is free: on a slope the car can roll. Flat floors only until we know more.
- Tuning the hold (longer, or ending when |ERPM| < ~100 for 0.5 s, which needs `/sensors/core` in the
  converter) is a later change with its own T1 repeat. Not part of v1.

### Required config change with it

`src/f1tenth_bringup/config/vesc_real_integration.yaml` has **`brake_min 0.0, brake_max 0.0`**:
vesc_driver would clip the 5 A brake to 0 and the converter would silently coast instead of brake.
It needs `brake_max: 5.0` (the tested value; the T1b yamls used 10.0), `brake_min: 0.0`. `current_min/max
+/-2.0` already allow current 0.

## 3. Mux ownership check (your file, needs your review)

Today `physical_drive_mux.py` latches `ACTUATOR_GRAPH_OWNERSHIP_FAULT` unless `/drive` and
`/commands/motor/speed` each have exactly 1 publisher and 1 subscriber. With brake and current in the
command path, a second node could brake or release the motor without the mux seeing it. Proposed
addition, same fail-closed latch, nothing else in the mux changes:

| Topic | Required |
|---|---|
| `/commands/motor/speed`, `/commands/motor/brake`, `/commands/motor/current` | exactly 1 publisher each, **the same node** (compare `node_name` + `node_namespace` from `get_publishers_info_by_topic`), and 1 subscriber each |
| `/commands/motor/duty_cycle`, `/commands/motor/position` | 0 publishers |
| `/commands/servo/position` (optional, recommended) | 1 publisher, the same node |

The bag already never records `/drive` or `/commands/*` (it would add a subscriber). This change would go
in its own commit for your review, like `fea48d3`.

## 4. VESC Tool items still to read (before the converter drives the car)

From [T1_VESC_TOOL_CHECKLIST.md](T1_VESC_TOOL_CHECKLIST.md); export motor and app XML first, change nothing
without a T1 repeat.

| Row | Setting | Why it matters for this proposal |
|---|---|---|
| 11 | `l_current_min` (motor brake current limit) | Requesting 10 A gave no more measured current than 5 A (<= 4.2 vs <= 3.7 A). If `l_current_min` caps braking near 5 A, a higher brake setting is meaningless. |
| 11 | `l_in_current_min` (battery regen limit) | Braking pushes current into the battery; a low limit weakens the brake. |
| 14 | `timeout_msec`, `timeout_brake_current` | The only stop when the converter or the Jetson is dead. The ~1 s stand release seen in October suggests ~1000 ms and 0 A (coast). Proposed: ~250 ms and the tested 5 A, then a T1 repeat. |
| 5, 6 | `s_pid_allow_braking`, `s_pid_ramp_erpms_s` | Option B (fix the speed-0 cause in the VESC). A small +0.16 s rise is still seen in 4 of 10 brake stops at 1.0/1.5 m/s, the speed-0 rebound's timing: the cause is suppressed, not removed. |

## 5. What this does not cover

- **Dead Jetson / dead converter / USB loss:** only the VESC timeout (row 14) acts. Physical cutoff stays
  mandatory.
- **Slow-down to a nonzero speed** still uses the speed PID; overshoot there is untested (option B).
- **Reverse:** like T1b, a negative speed request is treated as a stop. Keep it that way for v1?
- **Ground stop distance:** unknown until T4 with this converter.
- It is not an emergency stop.

## 6. Acceptance tests

1. Self-test of the state logic (the T1b tests ported, plus reverse = stop).
2. Disarmed on the car stack: mux status shows no ownership fault with the new converter; with
   `ackermann_to_vesc` started as well (two speed publishers), the mux must latch the fault.
   (A deliberate second publisher: needs Moatasem's approval at the time.)
3. Stand T1 repeat with the new converter: 5 steady stops at 0.5, 1.0, 1.5 m/s, none > 300 ERPM.
4. Then T4 (ground straight line) measures the stop distance and deceleration.

## 7. Decisions asked of you

1. OK to add our own converter package and replace `ackermann_to_vesc` in `real_integrated.launch.py`
   (your file)? Package name and location?
2. OK for the ownership check addition in `physical_drive_mux.py` (section 3)?
3. `vesc_real_integration.yaml` `brake_max 5.0`: OK?
4. Reverse = stop for v1: OK?
5. `teleop.launch.py` / `teleop_mapping.launch.py` / `vesc.launch.py` still use `ackermann_to_vesc` with
   speed 0 and `config/vesc.yaml` (`brake_max 200000`, `speed_max 10000`): switch them to the new
   converter, or mark them sensors/disarmed only?
