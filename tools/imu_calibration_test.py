#!/usr/bin/env python3

import time
import math
from smbus import SMBus

BUS = 7
ADDR = 0x68

# Registers
PWR_MGMT_1   = 0x6B
CONFIG       = 0x1A
SMPLRT_DIV   = 0x19
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C

bus = SMBus(BUS)


def read_word(reg):
    high = bus.read_byte_data(ADDR, reg)
    low = bus.read_byte_data(ADDR, reg + 1)

    value = (high << 8) | low

    if value >= 32768:
        value -= 65536

    return value


def read_sensor():
    # ±4 g
    accel_scale = 8192.0

    # ±1000 deg/s
    gyro_scale = 32.8

    ax = read_word(0x3B) / accel_scale
    ay = read_word(0x3D) / accel_scale
    az = read_word(0x3F) / accel_scale

    gx = read_word(0x43) / gyro_scale
    gy = read_word(0x45) / gyro_scale
    gz = read_word(0x47) / gyro_scale

    return ax, ay, az, gx, gy, gz


# -----------------------------
# Configure sensor
# -----------------------------

# Wake sensor, PLL clock
bus.write_byte_data(ADDR, PWR_MGMT_1, 0x01)
time.sleep(0.1)

# Digital low-pass filter
bus.write_byte_data(ADDR, CONFIG, 0x03)

# ~100 Hz sample rate
bus.write_byte_data(ADDR, SMPLRT_DIV, 9)

# Gyro ±1000 deg/s
bus.write_byte_data(ADDR, GYRO_CONFIG, 0x10)

# Accelerometer ±4 g
bus.write_byte_data(ADDR, ACCEL_CONFIG, 0x08)

time.sleep(0.2)

print()
print("MPU configuration:")
print("  Accelerometer: ±4 g")
print("  Gyroscope:     ±1000 deg/s")
print("  Target rate:   ~100 Hz")
print()


# -----------------------------
# Calibration
# -----------------------------

N = 100000

print("Keep the IMU COMPLETELY STILL.")
print(f"Collecting {N} samples...")
print()

sum_ax = 0.0
sum_ay = 0.0
sum_az = 0.0

sum_gx = 0.0
sum_gy = 0.0
sum_gz = 0.0


for i in range(N):

    ax, ay, az, gx, gy, gz = read_sensor()

    sum_ax += ax
    sum_ay += ay
    sum_az += az

    sum_gx += gx
    sum_gy += gy
    sum_gz += gz

    # Show progress every 100 samples
    if (i + 1) % 100 == 0:
        print(f"Collected {i + 1}/{N}")

    time.sleep(0.001)


# -----------------------------
# Calculate averages
# -----------------------------

ax_avg = sum_ax / N
ay_avg = sum_ay / N
az_avg = sum_az / N

gx_bias = sum_gx / N
gy_bias = sum_gy / N
gz_bias = sum_gz / N

gravity_mag = math.sqrt(
    ax_avg**2 +
    ay_avg**2 +
    az_avg**2
)


# -----------------------------
# Results
# -----------------------------

print()
print("========== CALIBRATION RESULT ==========")

print()
print("Average accelerometer:")
print(f"  X = {ax_avg:+.5f} g")
print(f"  Y = {ay_avg:+.5f} g")
print(f"  Z = {az_avg:+.5f} g")
print(f"  Magnitude = {gravity_mag:.5f} g")

print()
print("Gyroscope bias:")
print(f"  X = {gx_bias:+.5f} deg/s")
print(f"  Y = {gy_bias:+.5f} deg/s")
print(f"  Z = {gz_bias:+.5f} deg/s")

print()
print("========================================")
print()
print("Calibration finished.")
