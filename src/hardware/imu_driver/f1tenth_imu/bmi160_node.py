#!/usr/bin/env python3

import json
import math
import os
import statistics
import time

from smbus import SMBus

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu

from ament_index_python.packages import get_package_share_directory


class BMI160Node(Node):

    # =========================================================
    # BMI160 registers
    # =========================================================

    REG_CHIP_ID = 0x00
    REG_PMU_STATUS = 0x03

    REG_GYR_DATA = 0x0C

    REG_ACC_CONF = 0x40
    REG_ACC_RANGE = 0x41

    REG_GYR_CONF = 0x42
    REG_GYR_RANGE = 0x43

    REG_CMD = 0x7E

    EXPECTED_CHIP_ID = 0xD1

    # =========================================================
    # Sensor configuration
    # =========================================================

    # ±4 g
    ACC_SCALE = 8192.0

    # ±1000 deg/s
    GYR_SCALE = 32.8

    GRAVITY = 9.80665


    def __init__(self):

        super().__init__('bmi160_node')

        # -----------------------------------------------------
        # Default calibration file
        # -----------------------------------------------------

        default_calibration_file = os.path.join(
            get_package_share_directory('f1tenth_imu'),
            'config',
            'bmi160_accel_calibration.json'
        )

        # -----------------------------------------------------
        # ROS parameters
        # -----------------------------------------------------

        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('i2c_address', 0x69)

        self.declare_parameter(
            'frame_id',
            'imu_link'
        )

        self.declare_parameter(
            'topic_name',
            '/imu/data_raw'
        )

        self.declare_parameter(
            'publish_rate_hz',
            100.0
        )

        self.declare_parameter(
            'warmup_seconds',
            2.0
        )

        self.declare_parameter(
            'gyro_calibration_samples',
            500
        )

        self.declare_parameter(
            'gyro_motion_std_limit_dps',
            0.5
        )

        self.declare_parameter(
            'calibration_file',
            default_calibration_file
        )

        # -----------------------------------------------------
        # Read parameters
        # -----------------------------------------------------

        self.bus_number = int(
            self.get_parameter('i2c_bus').value
        )

        self.address = int(
            self.get_parameter('i2c_address').value
        )

        self.frame_id = str(
            self.get_parameter('frame_id').value
        )

        self.topic_name = str(
            self.get_parameter('topic_name').value
        )

        self.publish_rate = float(
            self.get_parameter('publish_rate_hz').value
        )

        self.warmup_seconds = float(
            self.get_parameter('warmup_seconds').value
        )

        self.gyro_calibration_samples = int(
            self.get_parameter(
                'gyro_calibration_samples'
            ).value
        )

        self.gyro_motion_limit = float(
            self.get_parameter(
                'gyro_motion_std_limit_dps'
            ).value
        )

        calibration_file = str(
            self.get_parameter(
                'calibration_file'
            ).value
        )

        # -----------------------------------------------------
        # Load our six-position accel calibration
        # -----------------------------------------------------

        self.load_accel_calibration(
            calibration_file
        )

        # -----------------------------------------------------
        # Open I2C
        # -----------------------------------------------------

        self.get_logger().info(
            f'Opening I2C bus {self.bus_number}, '
            f'address 0x{self.address:02X}'
        )

        self.bus = SMBus(
            self.bus_number
        )

        # -----------------------------------------------------
        # Verify the actual chip
        # -----------------------------------------------------

        chip_id = self.bus.read_byte_data(
            self.address,
            self.REG_CHIP_ID
        )

        self.get_logger().info(
            f'CHIP_ID = 0x{chip_id:02X}'
        )

        if chip_id != self.EXPECTED_CHIP_ID:

            raise RuntimeError(
                f'BMI160 not detected. '
                f'Expected 0xD1, got 0x{chip_id:02X}'
            )

        # -----------------------------------------------------
        # Configure BMI160
        # -----------------------------------------------------

        self.configure_sensor()

        # -----------------------------------------------------
        # Warm-up
        # -----------------------------------------------------

        self.get_logger().info(
            f'Warming IMU for '
            f'{self.warmup_seconds:.1f} seconds...'
        )

        time.sleep(
            self.warmup_seconds
        )

        # -----------------------------------------------------
        # Gyro calibration
        # -----------------------------------------------------

        self.get_logger().warn(
            'KEEP THE SENSOR COMPLETELY STATIONARY.'
        )

        self.get_logger().info(
            'Starting gyro bias calibration...'
        )

        self.gyro_bias = (
            self.calibrate_gyro()
        )

        # -----------------------------------------------------
        # Noise covariance
        #
        # These come from our real BMI160 bench test.
        # -----------------------------------------------------

        gyro_std_dps = [
            0.050392,
            0.043957,
            0.042130
        ]

        accel_std_g = [
            0.001040,
            0.001108,
            0.001323
        ]

        self.gyro_variance = [
            math.radians(value) ** 2
            for value in gyro_std_dps
        ]

        self.accel_variance = [
            (value * self.GRAVITY) ** 2
            for value in accel_std_g
        ]

        # -----------------------------------------------------
        # ROS publisher
        # -----------------------------------------------------

        self.publisher = self.create_publisher(
            Imu,
            self.topic_name,
            qos_profile_sensor_data
        )

        # -----------------------------------------------------
        # ROS timer
        # -----------------------------------------------------

        timer_period = (
            1.0 / self.publish_rate
        )

        self.timer = self.create_timer(
            timer_period,
            self.timer_callback
        )

        self.get_logger().info(
            'BMI160 initialization complete.'
        )

        self.get_logger().info(
            f'Publishing: {self.topic_name}'
        )

        self.get_logger().info(
            f'Rate: {self.publish_rate:.1f} Hz'
        )

        self.get_logger().info(
            f'Frame: {self.frame_id}'
        )


    # =========================================================
    # Load accelerometer calibration
    # =========================================================

    def load_accel_calibration(self, filename):

        self.get_logger().info(
            f'Loading calibration from: {filename}'
        )

        with open(filename, 'r') as file:
            calibration = json.load(file)

        bias = calibration['bias_g']
        scale = calibration['scale']

        self.accel_bias = [
            float(bias['x']),
            float(bias['y']),
            float(bias['z'])
        ]

        self.accel_scale = [
            float(scale['x']),
            float(scale['y']),
            float(scale['z'])
        ]

        # Reject obviously bad calibration
        for axis_scale in self.accel_scale:

            if not 0.8 < axis_scale < 1.2:

                raise RuntimeError(
                    'Accelerometer calibration '
                    'contains invalid scale values.'
                )

        self.get_logger().info(
            'Accel bias [g]: '
            f'X={self.accel_bias[0]:+.6f}, '
            f'Y={self.accel_bias[1]:+.6f}, '
            f'Z={self.accel_bias[2]:+.6f}'
        )

        self.get_logger().info(
            'Accel scale: '
            f'X={self.accel_scale[0]:.6f}, '
            f'Y={self.accel_scale[1]:.6f}, '
            f'Z={self.accel_scale[2]:.6f}'
        )


    # =========================================================
    # Configure BMI160
    # =========================================================

    def configure_sensor(self):

        self.get_logger().info(
            'Resetting BMI160...'
        )

        # Soft reset
        self.bus.write_byte_data(
            self.address,
            self.REG_CMD,
            0xB6
        )

        time.sleep(0.2)

        # Accelerometer normal mode
        self.bus.write_byte_data(
            self.address,
            self.REG_CMD,
            0x11
        )

        time.sleep(0.02)

        # Gyroscope normal mode
        self.bus.write_byte_data(
            self.address,
            self.REG_CMD,
            0x15
        )

        time.sleep(0.1)

        # -----------------------------------------------------
        # Accelerometer:
        # 100 Hz
        # ±4 g
        # -----------------------------------------------------

        self.bus.write_byte_data(
            self.address,
            self.REG_ACC_CONF,
            0x28
        )

        self.bus.write_byte_data(
            self.address,
            self.REG_ACC_RANGE,
            0x05
        )

        # -----------------------------------------------------
        # Gyroscope:
        # 100 Hz
        # ±1000 deg/s
        # -----------------------------------------------------

        self.bus.write_byte_data(
            self.address,
            self.REG_GYR_CONF,
            0x28
        )

        self.bus.write_byte_data(
            self.address,
            self.REG_GYR_RANGE,
            0x01
        )

        time.sleep(0.2)

        pmu_status = self.bus.read_byte_data(
            self.address,
            self.REG_PMU_STATUS
        )

        self.get_logger().info(
            f'PMU_STATUS = 0x{pmu_status:02X}'
        )

        self.get_logger().info(
            'BMI160 configured: '
            'Accel ±4g, Gyro ±1000 dps, 100 Hz'
        )


    # =========================================================
    # Convert 16-bit signed value
    # =========================================================

    @staticmethod
    def signed16(lsb, msb):

        value = (
            (msb << 8) |
            lsb
        )

        if value & 0x8000:
            value -= 65536

        return value


    # =========================================================
    # Read gyro XYZ + accel XYZ in one I2C burst
    # =========================================================

    def read_sensor(self):

        data = self.bus.read_i2c_block_data(
            self.address,
            self.REG_GYR_DATA,
            12
        )

        gx_raw = self.signed16(
            data[0],
            data[1]
        )

        gy_raw = self.signed16(
            data[2],
            data[3]
        )

        gz_raw = self.signed16(
            data[4],
            data[5]
        )

        ax_raw = self.signed16(
            data[6],
            data[7]
        )

        ay_raw = self.signed16(
            data[8],
            data[9]
        )

        az_raw = self.signed16(
            data[10],
            data[11]
        )

        gx = gx_raw / self.GYR_SCALE
        gy = gy_raw / self.GYR_SCALE
        gz = gz_raw / self.GYR_SCALE

        ax = ax_raw / self.ACC_SCALE
        ay = ay_raw / self.ACC_SCALE
        az = az_raw / self.ACC_SCALE

        return (
            ax, ay, az,
            gx, gy, gz
        )


    # =========================================================
    # Startup gyro bias calibration
    # =========================================================

    def calibrate_gyro(self):

        gx_values = []
        gy_values = []
        gz_values = []

        period = (
            1.0 / self.publish_rate
        )

        next_sample = (
            time.perf_counter()
        )

        for _ in range(
            self.gyro_calibration_samples
        ):

            next_sample += period

            (
                _,
                _,
                _,
                gx,
                gy,
                gz
            ) = self.read_sensor()

            gx_values.append(gx)
            gy_values.append(gy)
            gz_values.append(gz)

            remaining = (
                next_sample -
                time.perf_counter()
            )

            if remaining > 0:
                time.sleep(remaining)

        bias = [
            statistics.mean(gx_values),
            statistics.mean(gy_values),
            statistics.mean(gz_values)
        ]

        std = [
            statistics.stdev(gx_values),
            statistics.stdev(gy_values),
            statistics.stdev(gz_values)
        ]

        self.get_logger().info(
            'Gyro bias [deg/s]: '
            f'X={bias[0]:+.5f}, '
            f'Y={bias[1]:+.5f}, '
            f'Z={bias[2]:+.5f}'
        )

        self.get_logger().info(
            'Gyro std [deg/s]: '
            f'X={std[0]:.5f}, '
            f'Y={std[1]:.5f}, '
            f'Z={std[2]:.5f}'
        )

        if max(std) > self.gyro_motion_limit:

            raise RuntimeError(
                'Sensor moved during gyro calibration. '
                'Restart while the car is stationary.'
            )

        return bias


    # =========================================================
    # ROS publishing callback
    # =========================================================

    def timer_callback(self):

        try:

            (
                raw_ax,
                raw_ay,
                raw_az,
                raw_gx,
                raw_gy,
                raw_gz
            ) = self.read_sensor()

        except OSError as error:

            self.get_logger().error(
                f'I2C read failed: {error}'
            )

            return

        # -----------------------------------------------------
        # Accelerometer calibration
        # -----------------------------------------------------

        ax_g = (
            raw_ax -
            self.accel_bias[0]
        ) / self.accel_scale[0]

        ay_g = (
            raw_ay -
            self.accel_bias[1]
        ) / self.accel_scale[1]

        az_g = (
            raw_az -
            self.accel_bias[2]
        ) / self.accel_scale[2]

        # -----------------------------------------------------
        # Gyro bias correction
        # -----------------------------------------------------

        gx_dps = (
            raw_gx -
            self.gyro_bias[0]
        )

        gy_dps = (
            raw_gy -
            self.gyro_bias[1]
        )

        gz_dps = (
            raw_gz -
            self.gyro_bias[2]
        )

        # -----------------------------------------------------
        # Convert to ROS SI units
        #
        # accel: g -> m/s²
        # gyro : deg/s -> rad/s
        # -----------------------------------------------------

        ax = ax_g * self.GRAVITY
        ay = ay_g * self.GRAVITY
        az = az_g * self.GRAVITY

        gx = math.radians(
            gx_dps
        )

        gy = math.radians(
            gy_dps
        )

        gz = math.radians(
            gz_dps
        )

        # -----------------------------------------------------
        # sensor_msgs/Imu
        # -----------------------------------------------------

        msg = Imu()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            self.frame_id
        )

        # -----------------------------------------------------
        # We currently do NOT estimate orientation.
        #
        # -1 tells ROS consumers that orientation is unavailable.
        # -----------------------------------------------------

        msg.orientation_covariance[0] = -1.0

        # -----------------------------------------------------
        # Angular velocity [rad/s]
        # -----------------------------------------------------

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.angular_velocity_covariance = [
            self.gyro_variance[0], 0.0, 0.0,
            0.0, self.gyro_variance[1], 0.0,
            0.0, 0.0, self.gyro_variance[2]
        ]

        # -----------------------------------------------------
        # Linear acceleration [m/s²]
        # -----------------------------------------------------

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.linear_acceleration_covariance = [
            self.accel_variance[0], 0.0, 0.0,
            0.0, self.accel_variance[1], 0.0,
            0.0, 0.0, self.accel_variance[2]
        ]

        self.publisher.publish(
            msg
        )


    # =========================================================
    # Cleanup
    # =========================================================

    def destroy_node(self):

        if hasattr(self, 'bus'):
            self.bus.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = BMI160Node()

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    except Exception as error:

        if node is not None:

            node.get_logger().fatal(
                str(error)
            )

        else:

            print(
                f'BMI160 fatal error: {error}'
            )

    finally:

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
