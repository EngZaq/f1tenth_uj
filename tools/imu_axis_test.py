#!/usr/bin/env python3

import time
from smbus import SMBus

BUS = 7
ADDR = 0x68

bus = SMBus(BUS)

# Your measured gyro biases
GX_BIAS = -1.86173
GY_BIAS = +0.80938
GZ_BIAS = -0.70834


def read_word(reg):
    high = bus.read_byte_data(ADDR, reg)
    low = bus.read_byte_data(ADDR, reg + 1)

    value = (high << 8) | low

    if value >= 32768:
        value -= 65536

    return value


# Sensor configuration
bus.write_byte_data(ADDR, 0x6B, 0x01)

# DLPF
bus.write_byte_data(ADDR, 0x1A, 0x03)

# ~100 Hz
bus.write_byte_data(ADDR, 0x19, 9)

# Gyro ±1000 deg/s
bus.write_byte_data(ADDR, 0x1B, 0x10)

# Accel ±4 g
bus.write_byte_data(ADDR, 0x1C, 0x08)

time.sleep(0.2)

print("IMU AXIS TEST")
print("Ctrl+C to stop")
print()

try:
    while True:

        ax = read_word(0x3B) / 8192.0
        ay = read_word(0x3D) / 8192.0
        az = read_word(0x3F) / 8192.0

        gx = read_word(0x43) / 32.8 - GX_BIAS
        gy = read_word(0x45) / 32.8 - GY_BIAS
        gz = read_word(0x47) / 32.8 - GZ_BIAS

        print(
            f"A[g] "
            f"X={ax:+6.3f} "
            f"Y={ay:+6.3f} "
            f"Z={az:+6.3f}   "
            f"| G[dps] "
            f"X={gx:+8.2f} "
            f"Y={gy:+8.2f} "
            f"Z={gz:+8.2f}"
        )

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopped.")
