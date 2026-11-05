#!/usr/bin/env python3
"""Verify STOP -> AUTO_READY -> AUTO_ACTIVE -> STOP with VESC actuation DISABLED.

Never publishes or subscribes to /drive, motor speed or servo position.
Refuses to start unless
the live physical mux advertises real_actuation_enabled=false.
Refuses multiple existing hold publishers. Not a physical safety test.
"""
import json
import time


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String

    class Check(Node):
        def __init__(self):
            super().__init__('disarmed_auto_handshake_diagnostic')
            self.status = None
            self.status_samples = 0
            self.status_sub = self.create_subscription(
                String, '/control/status', self.status_cb, 10)
            self.hold_pub = None
            self.mode_pub = None

        def status_cb(self, msg):
            try:
                self.status = json.loads(msg.data)
                self.status_samples += 1
            except (ValueError, TypeError):
                pass

        def step(self, duration):
            stop_at = time.monotonic() + duration
            while time.monotonic() < stop_at:
                rclpy.spin_once(self, timeout_sec=.03)

        def verify_disarmed(self):
            if self.status is None or self.status.get('real_actuation_enabled') is not False:
                raise RuntimeError('ABORT: mux not confirmed disarmed')
            if (abs(float(self.status.get('speed_mps', 0))) > 1e-5 or
                    abs(float(self.status.get('steer_rad', 0))) > 1e-5):
                raise RuntimeError('ABORT: mux status reports nonzero actuation request')

        def publish_hold(self, value):
            self.hold_pub.publish(Bool(data=value))

        def publish_mode(self, value):
            self.mode_pub.publish(String(data=value))

        def repeat_until(self, goal, label, timeout, hold, request=None):
            deadline = time.monotonic()+timeout
            last_request = 0.
            while time.monotonic() < deadline:
                self.verify_disarmed()
                self.publish_hold(hold)
                if request and time.monotonic()-last_request>=.25:
                    self.publish_mode(request)
                    last_request=time.monotonic()
                self.step(.055)
                if self.status is not None and goal(self.status):
                    print('PASS:',label,
                          json.dumps({k:self.status.get(k) for k in
                            ('mode','mode_request_result','auto_hold_raw_value',
                             'auto_hold_held_and_fresh','real_actuation_enabled')},
                            separators=(',',':')))
                    return
            raise RuntimeError('TIMEOUT '+label+': '+
                json.dumps(self.status, separators=(',', ':')))

    rclpy.init()
    n = Check()
    armed = False
    try:
        n.step(1.0)
        n.verify_disarmed()
        counts = {
            'mux_status_publishers': len(n.get_publishers_info_by_topic('/control/status')),
            'drive_publishers': len(n.get_publishers_info_by_topic('/drive')),
            'drive_subscribers': len(n.get_subscriptions_info_by_topic('/drive')),
            'motor_publishers': len(n.get_publishers_info_by_topic('/commands/motor/speed')),
            'motor_subscribers': len(n.get_subscriptions_info_by_topic('/commands/motor/speed')),
            'hold_publishers_before_test': len(n.get_publishers_info_by_topic('/control/auto_hold')),
            'mode_request_publishers_before_test': len(n.get_publishers_info_by_topic('/control/request_mode')),
        }
        print('ROS endpoints:', json.dumps(counts))
        if counts['mux_status_publishers'] != 1:
            raise RuntimeError('ABORT: not exactly one physical mux status publisher')
        if any(counts[k] != 1 for k in (
                'drive_publishers','drive_subscribers','motor_publishers','motor_subscribers')):
            raise RuntimeError('ABORT: actuator graph ownership must be 1 publisher and 1 subscriber at both links')
        if counts['hold_publishers_before_test'] != 0:
            raise RuntimeError('ABORT: old AUTO-hold publisher still running; Ctrl+C that terminal first')
        if counts['mode_request_publishers_before_test'] != 0:
            raise RuntimeError('ABORT: another mode controller already publishing; stop it first')
        # Create topics only after verifying zero real actuation and ownership.
        n.hold_pub=n.create_publisher(Bool,'/control/auto_hold',10)
        n.mode_pub=n.create_publisher(String,'/control/request_mode',10)
        n.step(.5)  # DDS discovery
        armed=True

        n.repeat_until(
            lambda s: (s.get('auto_hold_raw_value') is False and
                       s.get('auto_hold_held_and_fresh') is False),
            'AUTO hold visibly released', 3.0, False)
        # Keep streaming false so request cannot race with held=true.
        n.repeat_until(
            lambda s: (s.get('mode')=='AUTO_READY' and
                       s.get('mode_request_result')=='AUTO_READY_ACCEPTED'),
            'AUTO selection acknowledged', 4.0, False, 'AUTO')
        n.repeat_until(
            lambda s: (s.get('mode')=='AUTO_ACTIVE' and
                       s.get('auto_hold_held_and_fresh') is True),
            'AUTO state entered without actuation', 3.0, True)
        # Stay in the state for repeated ticks and verify zero mux output from status.
        until = time.monotonic()+.8
        while time.monotonic()<until:
            n.publish_hold(True)
            n.step(.05)
            n.verify_disarmed()
            if not n.status or n.status.get('mode')!='AUTO_ACTIVE':
                raise RuntimeError('AUTO state dropped unexpectedly: '+
                                   json.dumps(n.status))
        print('PASS: AUTO_ACTIVE stable ~0.8s; mux reports zero physical /drive')
    except Exception as exc:
        print('FAIL:',str(exc))
        raise SystemExit(1)
    finally:
        if armed:
            for _ in range(12):
                # Always STOP mode + RELEASE hold, even on test failure.
                n.publish_hold(False)
                n.publish_mode('STOP')
                n.step(.045)
            print('Cleanup: STOP requested and hold released')
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
