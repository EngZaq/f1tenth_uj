#!/usr/bin/env python3
"""Offline *physical* SLAM map->odom correction diagnostic for ROS2 Humble sqlite3 bags.

Uses standard Python libraries only. Does not issue ROS commands, drive the car,
modify a ROS bag, or use simulator ground truth. This detects sudden changes in
SLAM's global correction; a large change is NOT by itself proof a loop is false.
"""
import argparse
import math
import sqlite3
import struct
from pathlib import Path


class CDR:
    def __init__(self, payload):
        self.data = payload
        self.offset = 4  # CDR encapsulation bytes

    def unpack(self, fmt, alignment):
        self.offset = 4 + ((self.offset - 4 + alignment - 1) // alignment) * alignment
        values = struct.unpack_from('<' + fmt, self.data, self.offset)
        self.offset += struct.calcsize('<' + fmt)
        return values[0] if len(values) == 1 else values

    def string(self):
        length = self.unpack('I', 4)
        if length < 1:
            raise ValueError('Malformed CDR string')
        s = self.data[self.offset:self.offset + length - 1].decode('utf-8')
        self.offset += length
        return s

    def transform(self):
        sec, nanosec = self.unpack('iI', 4)
        parent = self.string()
        child = self.string()
        xyz_q = self.unpack('7d', 8)
        return sec + nanosec * 1e-9, parent, child, xyz_q


def transform_messages(blob, parent, child):
    reader = CDR(blob)
    count = reader.unpack('I', 4)
    if count > 10000:
        raise ValueError('Invalid TF array length')
    for _ in range(count):
        stamp, p, c, x = reader.transform()
        if p == parent and c == child:
            qx, qy, qz, qw = x[3:]
            yaw = math.atan2(2 * (qw * qz + qx * qy),
                             1 - 2 * (qy * qy + qz * qz))
            yield stamp, x[0], x[1], yaw


def audit(path, parent, child, pos_threshold, yaw_threshold, max_gap):
    con = sqlite3.connect('file:' + str(path.resolve()) + '?mode=ro', uri=True)
    try:
        row = con.execute('SELECT id FROM topics WHERE name = ?', ('/tf',)).fetchone()
        if row is None:
            raise ValueError('No /tf topic in bag')
        prev = None
        n = nvalid = violations = 0
        max_pos = max_yaw = 0.0
        worst = None
        first = last = None
        for (stamp_ns, blob) in con.execute(
                'SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp',
                (row[0],)):
            stamp_s = stamp_ns * 1e-9
            for _tf_time, x, y, yaw in transform_messages(blob, parent, child):
                n += 1
                first = stamp_s if first is None else first
                last = stamp_s
                if prev is not None:
                    dt = stamp_s - prev[0]
                    if 0 < dt <= max_gap:
                        nvalid += 1
                        d = math.hypot(x - prev[1], y - prev[2])
                        angle = abs(math.degrees(math.atan2(
                            math.sin(yaw - prev[3]), math.cos(yaw - prev[3]))))
                        max_pos = max(max_pos, d)
                        max_yaw = max(max_yaw, angle)
                        if worst is None or d / pos_threshold + angle / yaw_threshold > worst[0]:
                            worst = (d / pos_threshold + angle / yaw_threshold,
                                     stamp_s, dt, d, angle)
                        if d > pos_threshold or angle > yaw_threshold:
                            violations += 1
                prev = stamp_s, x, y, yaw
    finally:
        con.close()
    print(f'Bag: {path}')
    print(f'Frame: {parent} -> {child}')
    print(f'Transform samples: {n}; adjacent comparisons <= {max_gap:.2f}s: {nvalid}')
    if nvalid == 0:
        print('INCONCLUSIVE: no valid consecutive samples')
        return 2
    print(f'Largest translation step: {max_pos:.4f} m')
    print(f'Largest rotation step:    {max_yaw:.3f} degrees')
    if worst:
        print('Most pronounced combined event: ' +
              f't={worst[1]-first:.2f}s from first TF, ' +
              f'dt={worst[2]:.3f}s, translation={worst[3]:.3f}m, yaw={worst[4]:.2f}deg')
    print(f'Regression bounds: {pos_threshold:.3f} m or {yaw_threshold:.1f} degrees in one TF step')
    print(f'Violations: {violations}')
    if violations:
        print('REVIEW REQUIRED: abnormal global correction; do not certify the map from this audit.')
        print('Note: a genuine loop closure may change map->odom abruptly; verify map consistency.')
        return 1
    print('PASS (correction-step diagnostic only; NOT a map-accuracy/safety certificate).')
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('db3', type=Path, help='ROS2 sqlite3 .db3 file; no replay required')
    p.add_argument('--parent', default='map')
    p.add_argument('--child', default='odom')
    p.add_argument('--pos-step-m', type=float, default=0.25,
                   help='diagnostic regression bound from previous stable/bad simulation runs')
    p.add_argument('--yaw-step-deg', type=float, default=5.0)
    # Teammate's tests show a 0.10 s gap misses corrections during optimization.
    p.add_argument('--max-gap-s', type=float, default=2.0)
    a = p.parse_args()
    if not a.db3.is_file():
        p.error('DB3 file does not exist')
    if any(not math.isfinite(v) or v <= 0 for v in
           (a.pos_step_m, a.yaw_step_deg, a.max_gap_s)):
        p.error('bounds must be finite positive numbers')
    try:
        return audit(a.db3, a.parent, a.child,
                     a.pos_step_m, a.yaw_step_deg, a.max_gap_s)
    except (ValueError, sqlite3.Error, struct.error) as exc:
        p.exit(2, f'INCONCLUSIVE: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
