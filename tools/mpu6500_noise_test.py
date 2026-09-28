#!/usr/bin/env python3

import time
import math
import statistics
from smbus import SMBus


# ============================================================
# Configuration
# ============================================================

BUS = 7
ADDR = 0x68

N = 5000

# MPU-6500 registers
REG_SMPLRT_DIV    = 0x19
REG_CONFIG        = 0x1A
REG_GYRO_CONFIG   = 0x1B
REG_ACCEL_CONFIG  = 0x1C
REG_ACCEL_CONFIG2 = 0x1D

REG_ACCEL_DATA    = 0x3B

REG_PWR_MGMT_1    = 0x6B
REG_WHO_AM_I      = 0x75


# ±4 g
ACC_SCALE = 8192.0

# ±1000 deg/s
GYR_SCALE = 32.8


bus = SMBus(BUS)


# ============================================================
# Helpers
# ============================================================

def signed16(msb, lsb):
    value = (msb << 8) | lsb

    if value & 0x8000:
        value -= 65536

    return value


def read_sensor():

    # MPU-6500 data layout beginning at 0x3B:
    #
    # 0x3B-0x40 accel XYZ
    # 0x41-0x42 temperature
    # 0x43-0x48 gyro XYZ
    #
    # Read all 14 bytes in ONE burst transaction.

    data = bus.read_i2c_block_data(
        ADDR,
        REG_ACCEL_DATA,
        14
    )

    ax_raw = signed16(data[0], data[1])
    ay_raw = signed16(data[2], data[3])
    az_raw = signed16(data[4], data[5])

    temp_raw = signed16(data[6], data[7])

    gx_raw = signed16(data[8], data[9])
    gy_raw = signed16(data[10], data[11])
    gz_raw = signed16(data[12], data[13])

    ax = ax_raw / ACC_SCALE
    ay = ay_raw / ACC_SCALE
    az = az_raw / ACC_SCALE

    gx = gx_raw / GYR_SCALE
    gy = gy_raw / GYR_SCALE
    gz = gz_raw / GYR_SCALE

    # MPU-6500 temperature conversion
    temp_c = temp_raw / 333.87 + 21.0

    return ax, ay, az, gx, gy, gz, temp_c


# ============================================================
# Verify sensor
# ============================================================

whoami = bus.read_byte_data(
    ADDR,
    REG_WHO_AM_I
)

print(f"WHO_AM_I = 0x{whoami:02X}")

if whoami != 0x70:
    print("WARNING: Expected 0x70 from this MPU-6500 module.")


# ============================================================
# Reset
# ============================================================

print("Resetting MPU-6500...")

bus.write_byte_data(
    ADDR,
    REG_PWR_MGMT_1,
    0x80
)

time.sleep(0.2)


# ============================================================
# Wake sensor
# ============================================================

# PLL with X-axis gyro reference
bus.write_byte_data(
    ADDR,
    REG_PWR_MGMT_1,
    0x01
)

time.sleep(0.1)


# ============================================================
# Configure sampling/filtering
# ============================================================

# DLPF enabled.
#
# CONFIG = 0x03
# Gyroscope DLPF setting appropriate for our initial test.

bus.write_byte_data(
    ADDR,
    REG_CONFIG,
    0x03
)


# Sample rate:
#
# With DLPF enabled, gyro output rate is 1 kHz.
#
# Sample Rate = 1000 / (1 + SMPLRT_DIV)
#
# DIV = 9:
#
# 1000 / 10 = 100 Hz

bus.write_byte_data(
    ADDR,
    REG_SMPLRT_DIV,
    9
)


# ============================================================
# Gyroscope
# ============================================================

# FS_SEL = 2
#
# ±1000 deg/s
#
# bits 4:3 = 10
#
# 0x10

bus.write_byte_data(
    ADDR,
    REG_GYRO_CONFIG,
    0x10
)


# ============================================================
# Accelerometer
# ============================================================

# AFS_SEL = 1
#
# ±4 g
#
# bits 4:3 = 01
#
# 0x08

bus.write_byte_data(
    ADDR,
    REG_ACCEL_CONFIG,
    0x08
)


# Accelerometer DLPF
bus.write_byte_data(
    ADDR,
    REG_ACCEL_CONFIG2,
    0x03
)

time.sleep(0.2)


# ============================================================
# Storage
# ============================================================

data = {
    "ax": [],
    "ay": [],
    "az": [],
    "gx": [],
    "gy": [],
    "gz": [],
    "temp": []
}


print()
print("=========================================")
print(" MPU-6500 NOISE TEST")
print("=========================================")

print()
print("Configuration:")
print("  Accelerometer : ±4 g")
print("  Gyroscope     : ±1000 deg/s")
print("  Target rate   : 100 Hz")
print()

print("DO NOT TOUCH OR MOVE THE SENSOR.")
print(f"Collecting {N} samples...")
print()


# ============================================================
# Sampling
# ============================================================

start = time.perf_counter()
next_sample = start


for i in range(N):

    next_sample += 0.01

    ax, ay, az, gx, gy, gz, temp = read_sensor()

    data["ax"].append(ax)
    data["ay"].append(ay)
    data["az"].append(az)

    data["gx"].append(gx)
    data["gy"].append(gy)
    data["gz"].append(gz)

    data["temp"].append(temp)

    if (i + 1) % 500 == 0:
        print(f"Collected {i + 1}/{N}")

    remaining = next_sample - time.perf_counter()

    if remaining > 0:
        time.sleep(remaining)


elapsed = time.perf_counter() - start


# ============================================================
# Statistics
# ============================================================

def print_stats(name, values, unit):

    mean = statistics.mean(values)
    median = statistics.median(values)
    std = statistics.stdev(values)

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
# Gravity magnitude
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
# Gyro outlier analysis
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


# ============================================================
# Temperature
# ============================================================

print()
print("=========================================")
print(" TEMPERATURE")
print("=========================================")

print(
    f"Start : {data['temp'][0]:.2f} °C"
)

print(
    f"End   : {data['temp'][-1]:.2f} °C"
)

print(
    f"Mean  : {statistics.mean(data['temp']):.2f} °C"
)


# ============================================================
# Timing
# ============================================================

print()
print("Timing:")
print(f"  Samples   : {N}")
print(f"  Elapsed   : {elapsed:.3f} s")
print(f"  Read rate : {N / elapsed:.3f} Hz")

print()
print("=========================================")

bus.close()
