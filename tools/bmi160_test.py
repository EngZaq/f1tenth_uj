#!/usr/bin/env python3

import time
from smbus import SMBus

# ============================================================
# BMI160 configuration
# ============================================================

BUS = 7
ADDR = 0x69

# Registers
REG_CHIP_ID    = 0x00
REG_PMU_STATUS = 0x03

REG_GYR_DATA   = 0x0C
REG_TEMP       = 0x20

REG_ACC_CONF   = 0x40
REG_ACC_RANGE  = 0x41
REG_GYR_CONF   = 0x42
REG_GYR_RANGE  = 0x43

REG_CMD        = 0x7E

# Scaling for our selected ranges:
#
# Accelerometer ±4 g
# 32768 / 4 = 8192 LSB/g
#
# Gyroscope ±1000 deg/s
# 32768 / 1000 = 32.768 LSB/(deg/s)

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


def read_sensor_data():

    # BMI160 registers are conveniently arranged:
    #
    # 0x0C-0x11 = gyro XYZ
    # 0x12-0x17 = accel XYZ
    #
    # So read all 12 bytes in one transaction.

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


def read_temperature():

    data = bus.read_i2c_block_data(
        ADDR,
        REG_TEMP,
        2
    )

    raw = signed16(data[0], data[1])

    # BMI160 temperature:
    # raw = 0 corresponds approximately to 23 °C
    temp_c = 23.0 + raw / 512.0

    return temp_c


# ============================================================
# Check chip identity
# ============================================================

chip_id = bus.read_byte_data(
    ADDR,
    REG_CHIP_ID
)

print(f"CHIP_ID = 0x{chip_id:02X}")

if chip_id != 0xD1:
    print("ERROR: Sensor does not identify as BMI160.")
    bus.close()
    exit(1)

print("BMI160 detected successfully.")


# ============================================================
# Reset sensor
# ============================================================

print("Resetting BMI160...")

bus.write_byte_data(
    ADDR,
    REG_CMD,
    0xB6
)

time.sleep(0.2)


# ============================================================
# Enable accelerometer
# ============================================================

# 0x11 = accelerometer normal mode

bus.write_byte_data(
    ADDR,
    REG_CMD,
    0x11
)

time.sleep(0.01)


# ============================================================
# Enable gyroscope
# ============================================================

# 0x15 = gyroscope normal mode

bus.write_byte_data(
    ADDR,
    REG_CMD,
    0x15
)

# Gyroscope takes longer to start
time.sleep(0.1)


# ============================================================
# Configure accelerometer
# ============================================================

# ACC_CONF = 0x28
#
# ODR = 100 Hz
# normal filtering

bus.write_byte_data(
    ADDR,
    REG_ACC_CONF,
    0x28
)

# ACC_RANGE = 0x05
# ±4 g

bus.write_byte_data(
    ADDR,
    REG_ACC_RANGE,
    0x05
)


# ============================================================
# Configure gyroscope
# ============================================================

# GYR_CONF = 0x28
#
# ODR = 100 Hz
# normal filtering

bus.write_byte_data(
    ADDR,
    REG_GYR_CONF,
    0x28
)

# GYR_RANGE = 0x01
# ±1000 deg/s

bus.write_byte_data(
    ADDR,
    REG_GYR_RANGE,
    0x01
)

time.sleep(0.1)


# ============================================================
# Check power state
# ============================================================

pmu = bus.read_byte_data(
    ADDR,
    REG_PMU_STATUS
)

print(f"PMU_STATUS = 0x{pmu:02X}")

print()
print("Configuration:")
print("  Accelerometer : ±4 g")
print("  Gyroscope     : ±1000 deg/s")
print("  Sample rate   : 100 Hz")
print()
print("Starting data...")
print("Press Ctrl+C to stop.")
print()


# ============================================================
# Main loop
# ============================================================

try:

    while True:

        ax, ay, az, gx, gy, gz = read_sensor_data()

        temp = read_temperature()

        print(
            f"ACC[g] "
            f"X={ax:+7.4f} "
            f"Y={ay:+7.4f} "
            f"Z={az:+7.4f}"
            f"    |    "
            f"GYRO[dps] "
            f"X={gx:+8.3f} "
            f"Y={gy:+8.3f} "
            f"Z={gz:+8.3f}"
            f"    |    "
            f"T={temp:5.2f} °C"
        )

        # Just for displaying readable output.
        # The sensor itself is configured for 100 Hz.
        time.sleep(0.05)


except KeyboardInterrupt:

    print("\nStopped.")


finally:

    bus.close()
