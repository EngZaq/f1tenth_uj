#!/usr/bin/env python3

import time
from smbus import SMBus

BUS = 7
ADDR = 0x68

bus = SMBus(BUS)


def read_word(reg):
    high = bus.read_byte_data(ADDR, reg)
    low = bus.read_byte_data(ADDR, reg + 1)

    value = (high << 8) | low

    if value >= 0x8000:
        value -= 65536

    return value


# Wake / select clock source
bus.write_byte_data(ADDR, 0x6B, 0x01)

time.sleep(0.1)

print("Reading MPU-6500...")
print("Press Ctrl+C to stop")
print()

try:
    while True:

        ax_raw = read_word(0x3B)
        ay_raw = read_word(0x3D)
        az_raw = read_word(0x3F)

        temp_raw = read_word(0x41)

        gx_raw = read_word(0x43)
        gy_raw = read_word(0x45)
        gz_raw = read_word(0x47)

        # Default power-on ranges:
        # Accel: ±2g => 16384 LSB/g
        # Gyro: ±250 deg/s => 131 LSB/(deg/s)

        ax = ax_raw / 16384.0
        ay = ay_raw / 16384.0
        az = az_raw / 16384.0

        gx = gx_raw / 131.0
        gy = gy_raw / 131.0
        gz = gz_raw / 131.0

        # MPU-6500 temperature approximation
        temp_c = temp_raw / 333.87 + 21.0

        print(
            f"ACC [g]  "
            f"X={ax:+7.3f} "
            f"Y={ay:+7.3f} "
            f"Z={az:+7.3f}   |   "
            f"GYRO [deg/s] "
            f"X={gx:+8.2f} "
            f"Y={gy:+8.2f} "
            f"Z={gz:+8.2f}   |   "
            f"T={temp_c:.1f} C"
        )

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopped.")
