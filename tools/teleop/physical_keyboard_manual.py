#!/usr/bin/env python3
"""Supervised, pulse-only terminal teleop for the physical F1TENTH mux.

NOT a hardware e-stop. Use only with the real independent propulsion cutoff
tested and in reach. This node never publishes directly to /drive or VESC.
Each explicit key event creates at most one 0.8s forward pulse; MANUAL must
be rearmed with the joystick deadman released between pulses.
"""
import json
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String

PULSE_SEC = 0.80
SPEED_MPS = 0.12
STEER_RAD = 0.06
LOOP_SEC = 0.04
ARM_TIMEOUT_SEC = 1.25


class KeyboardManual(Node):
    def __init__(self):
        super().__init__('physical_keyboard_manual')
        self.drive = self.create_publisher(AckermannDriveStamped, '/manual/drive', 10)
        self.joy = self.create_publisher(Joy, '/joy', 10)
        self.mode = self.create_publisher(String, '/control/request_mode', 10)
        self.create_subscription(String, '/control/status', self.status_cb, 10)
        self.status = None
        self.status_at = 0.0
        self.state = 'IDLE'
        self.arm_start = 0.0
        self.arm_sent = False
        self.pulse_end = 0.0
        self.steer = 0.0
        self.active_seen = False
        self.last_output = 0.0

    def say(self, message):
        sys.stdout.write('\r\n' + message + '\r\n')
        sys.stdout.flush()

    def status_cb(self, msg):
        try:
            self.status = json.loads(msg.data)
            self.status_at = time.monotonic()
        except (ValueError, TypeError):
            self.status = None

    def publish_joy(self, held):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = []
        msg.buttons = [0, 0, 0, 0, 0, int(bool(held))]
        self.joy.publish(msg)

    def publish_drive(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steer
        self.drive.publish(msg)

    def request_mode(self, name):
        msg = String()
        msg.data = name
        self.mode.publish(msg)

    def valid_environment(self, require_stop=False):
        s = self.status
        if s is None or time.monotonic() - self.status_at > 0.30:
            return 'No fresh mux status'
        if not s.get('real_actuation_enabled'):
            return 'Real actuation is disabled at launch'
        if not s.get('scan_fresh'):
            return 'LiDAR stale'
        if s.get('scan_reason') != 'FRONT_MEASURED':
            return 'Front LiDAR not clear (%s)' % s.get('scan_reason')
        if s.get('front_valid_fraction', 0) < 0.75:
            return 'Insufficient front LiDAR returns'
        distance = s.get('nearest_front_m')
        if distance is None or distance < 0.80:
            return 'Measured front obstacle or unknown range'
        for k in ('drive_publisher_count', 'drive_subscriber_count',
                  'motor_publisher_count', 'motor_subscriber_count'):
            if s.get(k) != 1:
                return 'Actuator ownership fault: ' + k
        if require_stop and s.get('mode') != 'STOP':
            return 'Mux must be STOP before a new pulse'
        return None

    def stop(self, reason=None):
        self.publish_joy(False)
        self.publish_drive(0.0, 0.0)
        self.request_mode('STOP')
        self.state = 'IDLE'
        self.active_seen = False
        if reason:
            self.say('STOP: ' + reason)

    def key(self, key):
        if key in ('q', '\x03'):
            self.stop('Operator quit')
            return False
        if key in (' ', 'x'):
            self.stop('Operator key')
            return True
        if key not in 'wad':
            return True
        if self.state != 'IDLE':
            return True  # never extend an existing movement pulse
        error = self.valid_environment(require_stop=True)
        if error:
            self.say('REFUSED: ' + error)
            return True
        self.steer = {'w': 0.0, 'a': STEER_RAD, 'd': -STEER_RAD}[key]
        self.state = 'ARMING'
        self.arm_start = time.monotonic()
        self.arm_sent = False
        self.say('Arming one %.2fs pulse: %.2fm/s, steer %+.3frad'
                 % (PULSE_SEC, SPEED_MPS, self.steer))
        return True

    def tick(self):
        now = time.monotonic()
        if self.state == 'IDLE':
            return
        if self.state == 'ARMING':
            self.publish_joy(False)  # release before requesting MANUAL
            self.publish_drive(0.0, 0.0)
            if now - self.arm_start >= 0.14 and not self.arm_sent:
                self.request_mode('MANUAL')
                self.arm_sent = True
            if (self.arm_sent and self.status is not None
                    and self.status.get('mode') == 'MANUAL_READY'):
                self.state = 'PULSE'
                self.pulse_end = now + PULSE_SEC
                self.active_seen = False
                self.say('Pulse started; SPACE/X stops, physical cutoff for emergency')
                return
            if now - self.arm_start > ARM_TIMEOUT_SEC:
                self.stop('MANUAL_READY not confirmed')
            return
        if self.state == 'PULSE':
            error = self.valid_environment()
            if error:
                self.stop(error)
                return
            if self.status and self.status.get('mode') == 'MANUAL_ACTIVE':
                self.active_seen = True
            if self.active_seen and self.status and self.status.get('mode') == 'STOP':
                self.stop('Mux latched STOP: ' +
                          str(self.status.get('last_stop_reason')))
                return
            if now >= self.pulse_end:
                self.stop('Pulse complete')
                return
            self.publish_joy(True)
            self.publish_drive(SPEED_MPS, self.steer)


def main():
    if not sys.stdin.isatty():
        raise SystemExit('Run in a real, foreground terminal (not a pipe).')
    rclpy.init()
    node = KeyboardManual()
    fd = sys.stdin.fileno()
    original = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        node.say('W=short forward pulse, A=short left pulse, D=short right pulse')
        node.say('SPACE/X=STOP, Q=STOP and quit. No reverse. Focus this terminal.')
        node.say('Independent physical motor cutoff MUST be within reach.')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if ready:
                if not node.key(sys.stdin.read(1).lower()):
                    break
            node.tick()
            time.sleep(LOOP_SEC)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        # Send STOP repeatedly before disconnect; the mux also latches
        # STOP on lost Joy heartbeat within 0.20 s.
        for _ in range(5):
            node.stop() if _ == 0 else (
                node.publish_joy(False),
                node.publish_drive(0.0, 0.0),
                node.request_mode('STOP'))
            rclpy.spin_once(node, timeout_sec=0.005)
            time.sleep(0.04)
        termios.tcsetattr(fd, termios.TCSADRAIN, original)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
