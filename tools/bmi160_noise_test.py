#!/usr/bin/env python3

import time
import math
import statistics
from smbus import SMBus


BUS = 7
ADDR = 0x69
N = 5000

REG_CHIP_ID   = 0x00
REG_GYR_DATA  = 0x0C

REG_ACC_CONF  = 0x40
REG_ACC_RANGE = 0x41
REG_GYR_CONF  = 0x42
REG_GYR_RANGE = 0x43

REG_CMD       = 0x7E

ACC_SCALE = 8192.0
GYR_SCALE = 32.8

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
# Check BMI160
# ============================================================

chip_id = bus.read_byte_data(ADDR, REG_CHIP_ID)

print(f"CHIP_ID = 0x{chip_id:02X}")

if chip_id != 0xD1:
    print("BMI160 not detected.")
    bus.close()
    raise SystemExit(1)


# ============================================================
# Reset + configure
# ============================================================

bus.write_byte_data(ADDR, REG_CMD, 0xB6)
time.sleep(0.2)

# Accelerometer normal mode
bus.write_byte_data(ADDR, REG_CMD, 0x11)
time.sleep(0.02)

# Gyro normal mode
bus.write_byte_data(ADDR, REG_CMD, 0x15)
time.sleep(0.1)

# 100 Hz, ±4 g
bus.write_byte_data(ADDR, REG_ACC_CONF, 0x28)
bus.write_byte_data(ADDR, REG_ACC_RANGE, 0x05)

# 100 Hz, ±1000 deg/s
bus.write_byte_data(ADDR, REG_GYR_CONF, 0x28)
bus.write_byte_data(ADDR, REG_GYR_RANGE, 0x01)

time.sleep(0.2)


data = {
    "ax": [],
    "ay": [],
    "az": [],
    "gx": [],
    "gy": [],
    "gz": []
}


print()
print("=========================================")
print(" BMI160 NOISE TEST")
print("=========================================")
print()
print("DO NOT TOUCH OR MOVE THE SENSOR.")
print(f"Collecting {N} samples...")
print()


start = time.perf_counter()
next_sample = start


for i in range(N):

    next_sample += 0.01

    ax, ay, az, gx, gy, gz = read_sensor()

    data["ax"].append(ax)
    data["ay"].append(ay)
    data["az"].append(az)

    data["gx"].append(gx)
    data["gy"].append(gy)
    data["gz"].append(gz)

    if (i + 1) % 500 == 0:
        print(f"Collected {i + 1}/{N}")

    sleep_time = next_sample - time.perf_counter()

    if sleep_time > 0:
        time.sleep(sleep_time)


elapsed = time.perf_counter() - start


# ============================================================
# Statistical functions
# ============================================================

def print_stats(name, values, unit):

    mean = statistics.mean(values)
    std = statistics.stdev(values)
    median = statistics.median(values)

    print(name)
    print(f"  Mean   : {mean:+.6f} {unit}")
    print(f"  Median : {median:+.6f} {unit}")
    print(f"  Std    : {std:.6f} {unit}")
    print(f"  Min    : {min(values):+.6f} {unit}")
    print(f"  Max    : {max(values):+.6f} {unit}")
    print()


print()
print("=========================================")
print(" RESULTS")
print("=========================================")
print()

print_stats("ACC X", data["ax"], "g")
print_stats("ACC Y", data["ay"], "g")
print_stats("ACC Z", data["az"], "g")

print_stats("GYRO X", data["gx"], "deg/s")
print_stats("GYRO Y", data["gy"], "deg/s")
print_stats("GYRO Z", data["gz"], "deg/s")


# ============================================================
# Gravity magnitude based on means
# ============================================================

ax_mean = statistics.mean(data["ax"])
ay_mean = statistics.mean(data["ay"])
az_mean = statistics.mean(data["az"])

gravity = math.sqrt(
    ax_mean**2 +
    ay_mean**2 +
    az_mean**2
)

print(f"Gravity magnitude: {gravity:.6f} g")


# ============================================================
# Outlier analysis
# ============================================================

print()
print("=========================================")
print(" GYRO OUTLIER ANALYSIS")
print("=========================================")


for axis in ["gx", "gy", "gz"]:

    values = data[axis]

    mean = statistics.mean(values)

    print()
    print(axis.upper())

    for threshold in [1, 2, 5, 10]:

        count = sum(
            abs(v - mean) > threshold
            for v in values
        )

        percentage = count / len(values) * 100.0

        print(
            f"  > ±{threshold:2d} deg/s from mean : "
            f"{count:5d} "
            f"({percentage:.3f}%)"
        )


print()
print("Timing:")
print(f"  Samples     : {N}")
print(f"  Elapsed     : {elapsed:.3f} s")
print(f"  Read rate   : {N / elapsed:.3f} Hz")

print()
print("=========================================")

bus.close()
