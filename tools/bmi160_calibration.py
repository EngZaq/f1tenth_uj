#!/usr/bin/env python3

import time
import math
import statistics
from smbus import SMBus


# ============================================================
# Configuration
# ============================================================

BUS = 7
ADDR = 0x69

N = 10000


REG_CHIP_ID = 0x00
REG_PMU_STATUS = 0x03

REG_GYR_DATA = 0x0C

REG_ACC_CONF = 0x40
REG_ACC_RANGE = 0x41
REG_GYR_CONF = 0x42
REG_GYR_RANGE = 0x43

REG_CMD = 0x7E


ACC_SCALE = 8192.0
GYR_SCALE = 32.768


bus = SMBus(BUS)


# ============================================================
# Helpers
# ============================================================

def signed16(lsb, msb):
    value = (msb << 8) | lsb

    if value & 0x8000:
        value -= 65536

    return value


def read_sensor():
    # One burst containing gyro XYZ + accel XYZ
    data = bus.read_i2c_block_data(
        ADDR,
        REG_GYR_DATA,
        12
    )

    gx_raw = signed16(data[0], data[1])
    gy_raw = signed16(data[2], data[3])
    gz_raw = signed16(data[4], data[5])

    ax_raw = signed16(data[6], data[7])
    ay_raw = signed16(data[8], data[9])
    az_raw = signed16(data[10], data[11])

    gx = gx_raw / GYR_SCALE
    gy = gy_raw / GYR_SCALE
    gz = gz_raw / GYR_SCALE

    ax = ax_raw / ACC_SCALE
    ay = ay_raw / ACC_SCALE
    az = az_raw / ACC_SCALE

    return ax, ay, az, gx, gy, gz


# ============================================================
# Verify chip
# ============================================================

chip_id = bus.read_byte_data(
    ADDR,
    REG_CHIP_ID
)

print(f"CHIP_ID = 0x{chip_id:02X}")

if chip_id != 0xD1:
    print("ERROR: BMI160 not detected.")
    bus.close()
    exit(1)


# ============================================================
# Reset
# ============================================================

bus.write_byte_data(
    ADDR,
    REG_CMD,
    0xB6
)

time.sleep(0.2)


# ============================================================
# Enable accelerometer normal mode
# ============================================================

bus.write_byte_data(
    ADDR,
    REG_CMD,
    0x11
)

time.sleep(0.02)


# ============================================================
# Enable gyro normal mode
# ============================================================

bus.write_byte_data(
    ADDR,
    REG_CMD,
    0x15
)

time.sleep(0.1)


# ============================================================
# Configuration
# ============================================================

# Accelerometer:
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


# Gyroscope:
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
# Storage
# ============================================================

ax_data = []
ay_data = []
az_data = []

gx_data = []
gy_data = []
gz_data = []


print()
print("==========================================")
print(" BMI160 STATIONARY CALIBRATION")
print("==========================================")
print()
print(f"Samples: {N}")
print()
print("DO NOT MOVE THE SENSOR.")
print()


# ============================================================
# Sampling
# ============================================================

start_time = time.perf_counter()

next_sample = start_time

for i in range(N):

    # Target 100 Hz
    next_sample += 0.01

    ax, ay, az, gx, gy, gz = read_sensor()

    ax_data.append(ax)
    ay_data.append(ay)
    az_data.append(az)

    gx_data.append(gx)
    gy_data.append(gy)
    gz_data.append(gz)

    if (i + 1) % 1000 == 0:
        print(f"Collected {i + 1}/{N}")

    remaining = next_sample - time.perf_counter()

    if remaining > 0:
        time.sleep(remaining)


end_time = time.perf_counter()

elapsed = end_time - start_time

actual_rate = N / elapsed


# ============================================================
# Statistics
# ============================================================

ax_mean = statistics.mean(ax_data)
ay_mean = statistics.mean(ay_data)
az_mean = statistics.mean(az_data)

gx_mean = statistics.mean(gx_data)
gy_mean = statistics.mean(gy_data)
gz_mean = statistics.mean(gz_data)


ax_std = statistics.stdev(ax_data)
ay_std = statistics.stdev(ay_data)
az_std = statistics.stdev(az_data)

gx_std = statistics.stdev(gx_data)
gy_std = statistics.stdev(gy_data)
gz_std = statistics.stdev(gz_data)


gravity = math.sqrt(
    ax_mean ** 2 +
    ay_mean ** 2 +
    az_mean ** 2
)


# ============================================================
# Output
# ============================================================

print()
print("==========================================")
print(" BMI160 CALIBRATION RESULT")
print("==========================================")

print()
print("Accelerometer mean:")
print(f"  X = {ax_mean:+.6f} g")
print(f"  Y = {ay_mean:+.6f} g")
print(f"  Z = {az_mean:+.6f} g")

print()
print(f"Gravity magnitude = {gravity:.6f} g")

print()
print("Accelerometer standard deviation:")
print(f"  X = {ax_std:.6f} g")
print(f"  Y = {ay_std:.6f} g")
print(f"  Z = {az_std:.6f} g")


print()
print("Gyroscope bias:")
print(f"  X = {gx_mean:+.6f} deg/s")
print(f"  Y = {gy_mean:+.6f} deg/s")
print(f"  Z = {gz_mean:+.6f} deg/s")

print()
print("Gyroscope standard deviation:")
print(f"  X = {gx_std:.6f} deg/s")
print(f"  Y = {gy_std:.6f} deg/s")
print(f"  Z = {gz_std:.6f} deg/s")


print()
print("Timing:")
print(f"  Samples       = {N}")
print(f"  Elapsed       = {elapsed:.3f} s")
print(f"  Actual rate   = {actual_rate:.3f} Hz")

print()
print("==========================================")


bus.close()
