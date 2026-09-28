#!/usr/bin/env python3

import time
import json
import math
import statistics
from pathlib import Path
from smbus import SMBus


BUS = 7
ADDR = 0x69

N = 2000

REG_CHIP_ID   = 0x00
REG_GYR_DATA  = 0x0C
REG_ACC_CONF  = 0x40
REG_ACC_RANGE = 0x41
REG_GYR_CONF  = 0x42
REG_GYR_RANGE = 0x43
REG_CMD       = 0x7E

ACC_SCALE = 8192.0
GYR_SCALE = 32.768


bus = SMBus(BUS)


def signed16(lsb, msb):
    value = (msb << 8) | lsb

    if value & 0x8000:
        value -= 65536

    return value


def read_sensor():
    data = bus.read_i2c_block_data(
        ADDR,
        REG_GYR_DATA,
        12
    )

    gx = signed16(data[0], data[1]) / GYR_SCALE
    gy = signed16(data[2], data[3]) / GYR_SCALE
    gz = signed16(data[4], data[5]) / GYR_SCALE

    ax = signed16(data[6], data[7]) / ACC_SCALE
    ay = signed16(data[8], data[9]) / ACC_SCALE
    az = signed16(data[10], data[11]) / ACC_SCALE

    return ax, ay, az, gx, gy, gz


# ============================================================
# Verify BMI160
# ============================================================

chip_id = bus.read_byte_data(
    ADDR,
    REG_CHIP_ID
)

print(f"CHIP_ID = 0x{chip_id:02X}")

if chip_id != 0xD1:
    print("ERROR: BMI160 not detected.")
    bus.close()
    raise SystemExit(1)


# ============================================================
# Reset + configure
# ============================================================

bus.write_byte_data(
    ADDR,
    REG_CMD,
    0xB6
)

time.sleep(0.2)


# Accelerometer normal
bus.write_byte_data(
    ADDR,
    REG_CMD,
    0x11
)

time.sleep(0.02)


# Gyroscope normal
bus.write_byte_data(
    ADDR,
    REG_CMD,
    0x15
)

time.sleep(0.1)


# Accel:
# 100 Hz
# ±4 g

bus.write_byte_data(
    ADDR,
    REG_ACC_CONF,
    0x28
)

bus.write_byte_data(
    ADDR,
    REG_ACC_RANGE,
    0x05
)


# Gyro:
# 100 Hz
# ±1000 deg/s

bus.write_byte_data(
    ADDR,
    REG_GYR_CONF,
    0x28
)

bus.write_byte_data(
    ADDR,
    REG_GYR_RANGE,
    0x01
)

time.sleep(0.2)


# ============================================================
# Collect one stationary orientation
# ============================================================

def collect_orientation():

    ax_values = []
    ay_values = []
    az_values = []

    gx_values = []
    gy_values = []
    gz_values = []

    print()
    print("Collecting...")
    print("DO NOT TOUCH THE SENSOR.")
    print()

    start = time.perf_counter()
    next_sample = start

    for i in range(N):

        next_sample += 0.01

        ax, ay, az, gx, gy, gz = read_sensor()

        ax_values.append(ax)
        ay_values.append(ay)
        az_values.append(az)

        gx_values.append(gx)
        gy_values.append(gy)
        gz_values.append(gz)

        if (i + 1) % 500 == 0:
            print(
                f"Collected {i + 1}/{N}"
            )

        remaining = (
            next_sample -
            time.perf_counter()
        )

        if remaining > 0:
            time.sleep(remaining)

    result = {
        "ax_mean": statistics.mean(ax_values),
        "ay_mean": statistics.mean(ay_values),
        "az_mean": statistics.mean(az_values),

        "ax_std": statistics.stdev(ax_values),
        "ay_std": statistics.stdev(ay_values),
        "az_std": statistics.stdev(az_values),

        "gx_mean": statistics.mean(gx_values),
        "gy_mean": statistics.mean(gy_values),
        "gz_mean": statistics.mean(gz_values),
    }

    return result


# ============================================================
# Determine which axis was vertical
# FROM THE SAME DATA THAT WAS COLLECTED
# ============================================================

def classify_orientation(result):

    ax = result["ax_mean"]
    ay = result["ay_mean"]
    az = result["az_mean"]

    values = {
        "X": ax,
        "Y": ay,
        "Z": az
    }

    axis = max(
        values,
        key=lambda name: abs(values[name])
    )

    dominant = values[axis]

    magnitude = math.sqrt(
        ax * ax +
        ay * ay +
        az * az
    )

    print()
    print("Measured:")
    print(
        f"  X = {ax:+.6f} g"
    )
    print(
        f"  Y = {ay:+.6f} g"
    )
    print(
        f"  Z = {az:+.6f} g"
    )
    print(
        f"  |a| = {magnitude:.6f} g"
    )

    # Board must be reasonably close to one axis
    if abs(dominant) < 0.80:
        print()
        print(
            "REJECTED: no axis is close enough "
            "to vertical."
        )
        return None

    # Other axes should not be excessively tilted
    others = [
        abs(value)
        for name, value in values.items()
        if name != axis
    ]

    if max(others) > 0.30:
        print()
        print(
            "REJECTED: board is too tilted."
        )
        return None

    sign = "+" if dominant > 0 else "-"

    return sign + axis


# ============================================================
# Main
# ============================================================

required = {
    "+X",
    "-X",
    "+Y",
    "-Y",
    "+Z",
    "-Z"
}

results = {}


print()
print("========================================")
print(" BMI160 SIX-POSITION CALIBRATION")
print("========================================")

print()
print("IMPORTANT:")
print()
print("You do not choose +X/-X/+Y/etc.")
print("Just place the board on six different")
print("faces.")
print()
print("The program determines the axis AFTER")
print("collecting the data.")
print()


while len(results) < 6:

    print()
    print("Already collected:")

    if results:
        print(
            "  " +
            " ".join(sorted(results.keys()))
        )
    else:
        print("  none")

    print()

    print("Still needed:")
    print(
        "  " +
        " ".join(
            sorted(required - set(results.keys()))
        )
    )

    print()

    input(
        "Place board on a NEW face, "
        "wait until stable, then press ENTER..."
    )

    # Allow vibration to settle
    time.sleep(2.0)

    result = collect_orientation()

    label = classify_orientation(result)

    if label is None:
        print()
        print(
            "Try this position again."
        )
        continue

    print()
    print(
        f"Detected orientation: {label}"
    )

    if label in results:

        print()
        print(
            f"{label} is already recorded."
        )

        print(
            "Flip/rotate the board to a truly "
            "opposite or different face."
        )

        continue

    results[label] = result

    print()
    print(
        f"Accepted {label}."
    )


# ============================================================
# Extract six endpoints
# ============================================================

x_pos = results["+X"]["ax_mean"]
x_neg = results["-X"]["ax_mean"]

y_pos = results["+Y"]["ay_mean"]
y_neg = results["-Y"]["ay_mean"]

z_pos = results["+Z"]["az_mean"]
z_neg = results["-Z"]["az_mean"]


# ============================================================
# Calibration
# ============================================================

bias_x = (
    x_pos + x_neg
) / 2.0

bias_y = (
    y_pos + y_neg
) / 2.0

bias_z = (
    z_pos + z_neg
) / 2.0


scale_x = (
    x_pos - x_neg
) / 2.0

scale_y = (
    y_pos - y_neg
) / 2.0

scale_z = (
    z_pos - z_neg
) / 2.0


# ============================================================
# Sanity checks
# ============================================================

print()
print("========================================")
print(" RAW SIX-POSITION VALUES")
print("========================================")
print()

print(f"+X = {x_pos:+.6f} g")
print(f"-X = {x_neg:+.6f} g")
print()

print(f"+Y = {y_pos:+.6f} g")
print(f"-Y = {y_neg:+.6f} g")
print()

print(f"+Z = {z_pos:+.6f} g")
print(f"-Z = {z_neg:+.6f} g")


valid = True

for name, scale in [
    ("X", scale_x),
    ("Y", scale_y),
    ("Z", scale_z)
]:

    if not 0.80 < scale < 1.20:

        print()
        print(
            f"ERROR: {name} scale = {scale:.4f}"
        )

        print(
            "Expected approximately 1.0."
        )

        valid = False


if not valid:

    print()
    print(
        "Calibration FAILED sanity check."
    )

    print(
        "Do NOT save/use these values."
    )

    bus.close()

    raise SystemExit(1)


# ============================================================
# Print calibration
# ============================================================

print()
print("========================================")
print(" FINAL CALIBRATION")
print("========================================")

print()

print("Bias [g]:")

print(
    f"  X = {bias_x:+.8f}"
)

print(
    f"  Y = {bias_y:+.8f}"
)

print(
    f"  Z = {bias_z:+.8f}"
)


print()

print("Scale:")

print(
    f"  X = {scale_x:.8f}"
)

print(
    f"  Y = {scale_y:.8f}"
)

print(
    f"  Z = {scale_z:.8f}"
)


print()
print("Correction equations:")
print()

print(
    f"ax = (raw_ax - ({bias_x:+.8f})) "
    f"/ {scale_x:.8f}"
)

print(
    f"ay = (raw_ay - ({bias_y:+.8f})) "
    f"/ {scale_y:.8f}"
)

print(
    f"az = (raw_az - ({bias_z:+.8f})) "
    f"/ {scale_z:.8f}"
)


# ============================================================
# Save calibration
# ============================================================

calibration = {

    "sensor": "BMI160",

    "i2c_bus": BUS,

    "i2c_address": hex(ADDR),

    "accel_range_g": 4,

    "sample_rate_hz": 100,

    "bias_g": {
        "x": bias_x,
        "y": bias_y,
        "z": bias_z
    },

    "scale": {
        "x": scale_x,
        "y": scale_y,
        "z": scale_z
    },

    "six_position_raw": results
}


output_dir = (
    Path.home() /
    "f1tenth_uj" /
    "config"
)

output_dir.mkdir(
    parents=True,
    exist_ok=True
)


output_file = (
    output_dir /
    "bmi160_accel_calibration.json"
)


with open(
    output_file,
    "w"
) as f:

    json.dump(
        calibration,
        f,
        indent=4
    )


print()
print(
    "Calibration saved to:"
)

print(
    output_file
)

print()

bus.close()
