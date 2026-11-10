#!/usr/bin/env python3
"""Supervised continuous terminal keyboard teleoperation for F1TENTH hardware.

Publishes /manual/drive and a simulated joystick deadman to the existing
physical_drive_mux. NEVER writes /drive or VESC commands directly.

W starts/refreshes continuous forward driving; A/D steer incrementally;
C centers; +/- change speed; S, SPACE or X STOP; Q quits.
For terminal input (which cannot robustly detect key-release), motion requires
repeated W/A/D/C/+/- input. A short input-inactivity watchdog latches STOP.
A physical independent propulsion cutoff is still required. No reverse.
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

SPEED_MIN_MPS = 0.10
SPEED_MAX_MPS = 0.30
DEFAULT_SPEED_MPS = 0.12
SPEED_STEP_MPS = 0.02
STEER_MAX_RAD = 0.10
STEER_STEP_RAD = 0.02
CONTROL_INPUT_TIMEOUT_SEC = 1.75
COMMAND_PERIOD_SEC = 0.05
ARM_TIMEOUT_SEC = 1.8


class KeyboardManual(Node):
    def __init__(self):
        super().__init__('physical_keyboard_manual')
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/manual/drive', 10)
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.mode_pub = self.create_publisher(String, '/control/request_mode', 10)
        self.create_subscription(String, '/control/status', self.status_cb, 10)
        self.status = None
        self.status_at = 0.0
        self.state = 'IDLE'
        self.rearm_required = False
        self.speed = DEFAULT_SPEED_MPS
        self.steer = 0.0
        self.last_key_at = 0.0
        self.arm_at = 0.0
        self.last_arm_request_at = 0.0
        self.activated = False
        self.positive_speed_seen = False
        self.last_display_at = 0.0

    def print(self, message):
        sys.stdout.write('\r\n' + message + '\r\n')
        sys.stdout.flush()

    def status_cb(self, msg):
        try:
            self.status = json.loads(msg.data)
            self.status_at = time.monotonic()
        except (ValueError, TypeError):
            self.status = None

    def joy(self, held):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.buttons = [0, 0, 0, 0, 0, int(bool(held))]
        self.joy_pub.publish(msg)

    def drive(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(msg)

    def mode(self, name):
        self.mode_pub.publish(String(data=name))

    def stop(self, reason, require_rearm=True):
        self.joy(False)
        self.drive(0.0, 0.0)
        self.mode('STOP')
        self.state = 'IDLE'
        self.activated = False
        self.positive_speed_seen = False
        self.steer = 0.0
        if require_rearm:
            self.rearm_required = True
        self.print('STOP: ' + reason)

    def preflight(self, require_stopped=False):
        s = self.status
        if s is None or time.monotonic() - self.status_at >= .30:
            return 'No fresh mux status'
        if not s.get('real_actuation_enabled'):
            return 'Real actuation disabled in launch'
        if not s.get('scan_fresh'):
            return 'LiDAR data stale'
        threshold = float(s.get('manual_min_front_valid_fraction', .75))
        fraction = float(s.get('front_valid_fraction', 0))
        if fraction < threshold:
            return 'LiDAR observed %.0f%%, manual requires %.0f%%' % (100*fraction, 100*threshold)
        dist = s.get('nearest_front_m')
        if dist is None or float(dist) < .80:
            return 'Front obstacle / unknown distance'
        for name in ('drive_publisher_count', 'drive_subscriber_count',
                     'motor_publisher_count', 'motor_subscriber_count'):
            if s.get(name) != 1:
                return 'Actuator graph fault: ' + name
        if require_stopped and s.get('mode') != 'STOP':
            return 'Mux is not in STOP'
        return None

    def on_key(self, key):
        now = time.monotonic()
        if key in ('q', '\x03'):
            self.stop('Operator quit')
            return False
        if key in ('s', 'x', ' '):
            self.stop('Operator STOP key')
            return True
        if key == 'r':
            if self.state == 'IDLE':
                self.rearm_required = False
                self.print('Rearmed. W starts controlled forward driving.')
            return True
        if key not in ('w', 'a', 'd', 'c', '+', '=', '-', '_'):
            return True
        if self.state == 'IDLE' and self.rearm_required:
            return True
        if key in ('+', '='):
            self.speed = min(SPEED_MAX_MPS, round(self.speed + SPEED_STEP_MPS, 2))
            self.print('Selected speed %.2f m/s' % self.speed)
        elif key in ('-', '_'):
            self.speed = max(SPEED_MIN_MPS, round(self.speed - SPEED_STEP_MPS, 2))
            self.print('Selected speed %.2f m/s' % self.speed)
        elif key == 'a':
            self.steer = min(STEER_MAX_RAD, round(self.steer + STEER_STEP_RAD, 3))
            self.print('Steering %+0.3f rad' % self.steer)
        elif key == 'd':
            self.steer = max(-STEER_MAX_RAD, round(self.steer - STEER_STEP_RAD, 3))
            self.print('Steering %+0.3f rad' % self.steer)
        elif key == 'c':
            self.steer = 0.0
            self.print('Steering centered')
        if self.state == 'IDLE':
            if key != 'w':
                return True  # A/D or speed selection alone must never start motion
            problem = self.preflight(require_stopped=True)
            if problem:
                self.print('REFUSED: ' + problem)
                return True
            self.state = 'ARMING'
            self.arm_at = now
            self.last_arm_request_at = 0.0
            self.activated = False
            self.positive_speed_seen = False
            self.print('Arming continuous MANUAL. Keep sending W/A/D/C keys while driving.')
        if self.state == 'DRIVING':
            self.last_key_at = now
        return True

    def tick(self):
        now = time.monotonic()
        if self.state == 'IDLE':
            return
        if self.state == 'ARMING':
            self.joy(False)
            self.drive(0, 0)
            if now - self.arm_at >= .15 and now - self.last_arm_request_at >= .35:
                self.mode('MANUAL')
                self.last_arm_request_at = now
            if (self.status is not None and
                    self.status.get('mode') == 'MANUAL_READY' and
                    self.last_arm_request_at > 0):
                self.state = 'DRIVING'
                self.last_key_at = now
                self.print('MANUAL_READY accepted. Driving; keyboard inactivity STOP after %.2fs'
                           % CONTROL_INPUT_TIMEOUT_SEC)
                return
            if now - self.arm_at >= ARM_TIMEOUT_SEC:
                self.stop('Mux did not enter MANUAL_READY')
            return
        if now - self.last_key_at >= CONTROL_INPUT_TIMEOUT_SEC:
            self.stop('Keyboard input timed out')
            return
        problem = self.preflight()
        if problem:
            self.stop(problem)
            return
        mode = self.status.get('mode')
        if mode == 'STOP':
            self.stop('Mux STOP: %s; last stop %s' % (
                self.status.get('reason'), self.status.get('last_stop_reason')))
            return
        if mode not in ('MANUAL_READY', 'MANUAL_ACTIVE'):
            self.stop('Unexpected mux mode: ' + str(mode))
            return
        if mode == 'MANUAL_ACTIVE':
            if not self.activated:
                self.print('CONFIRMED MANUAL_ACTIVE')
                self.activated = True
            speed_report = float(self.status.get('speed_mps', 0.0))
            if speed_report > 0.0 and not self.positive_speed_seen:
                self.positive_speed_seen = True
                self.print('CONFIRMED mux nonzero motor command: %.3f m/s' % speed_report)
        if now - self.last_key_at > 0.65 and not self.activated:
            self.stop('Mux never activated MANUAL')
            return
        if self.activated and now - self.last_key_at > 0.65 and not self.positive_speed_seen:
            self.stop('Mux ACTIVE but no nonzero motor command')
            return
        # Fresh joystick deadman and manual command, both at ~20 Hz.
        # Publish every control tick while operator input remains fresh.
        self.drive(self.speed, self.steer)
        self.joy(True)
        if now - self.last_display_at >= 1.0:
            self.last_display_at = now
            self.print('RUN: requested %.2f m/s steer %+.3f | mux %s speed %.3f reason %s'
                       % (self.speed, self.steer, mode,
                          float(self.status.get('speed_mps', 0.0)),
                          self.status.get('reason')))


def main():
    if not sys.stdin.isatty():
        raise SystemExit('Run directly in an interactive terminal')
    rclpy.init()
    node = KeyboardManual()
    fd = sys.stdin.fileno()
    terminal_state = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        node.print('W=start/refresh forward; A=left, D=right; C=center; +/- speed')
        node.print('S/SPACE/X=STOP; R=rearm after STOP; Q=STOP+exit. Reverse not enabled.')
        node.print('Hold W for straights; send A/D to steer; STOP if no keys for %.2fs.'
                   % CONTROL_INPUT_TIMEOUT_SEC)
        node.print('Do not depend on SSH or ROS as an emergency stop; physical cutoff required.')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if ready and not node.on_key(sys.stdin.read(1).lower()):
                break
            node.tick()
            time.sleep(COMMAND_PERIOD_SEC)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        for i in range(5):
            if i == 0:
                node.stop('Process exiting')
            else:
                node.joy(False)
                node.drive(0.0, 0.0)
                node.mode('STOP')
            rclpy.spin_once(node, timeout_sec=0.005)
            time.sleep(.04)
        termios.tcsetattr(fd, termios.TCSADRAIN, terminal_state)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
