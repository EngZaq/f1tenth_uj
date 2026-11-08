#!/usr/bin/env python3
"""Pure-Python regression of the physical SLAM TF jump audit (synthetic bag).

Tests the teammate finding: a 0.6 s SLAM optimization publication gap can
hide a 1.5 m / 24 deg map->odom jump if the auditor only compares <0.1 s.
No ROS process or physical actuator is started.
"""
import math
import sqlite3
import struct
import tempfile
from pathlib import Path

from physical_mapping_correction_audit import audit


def cdr_tf(x, y, heading_deg, stamp_ns):
    blob = bytearray(b'\x00\x01\x00\x00')

    def align(n):
        while (len(blob)-4) % n:
            blob.append(0)

    align(4)
    blob.extend(struct.pack('<I', 1))  # TFMessage.transforms[1]
    align(4)
    blob.extend(struct.pack('<iI', int(stamp_ns // 10**9),
                            int(stamp_ns % 10**9)))
    for s in ('map', 'odom'):
        align(4)
        data = s.encode('utf8') + b'\0'
        blob.extend(struct.pack('<I', len(data)))
        blob.extend(data)
    align(8)
    a = math.radians(heading_deg)/2.
    blob.extend(struct.pack('<7d', x, y, 0., 0., 0.,
                            math.sin(a), math.cos(a)))
    return bytes(blob)


def build_bag(path, records):
    db = sqlite3.connect(str(path))
    try:
        db.execute('CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT)')
        db.execute('CREATE TABLE messages (timestamp INTEGER, topic_id INTEGER, data BLOB)')
        db.execute("INSERT INTO topics(id,name) VALUES(1,'/tf')")
        for secs,x,y,angle in records:
            ns = int(secs*1e9)
            db.execute('INSERT INTO messages(timestamp,topic_id,data) VALUES(?,?,?)',
                       (ns,1,cdr_tf(x,y,angle,ns)))
        db.commit()
    finally:
        db.close()


def main():
    with tempfile.TemporaryDirectory() as directory:
        db3 = Path(directory)/'synthetic.db3'
        # Two TF samples 0.6 s apart: large correction during solve gap.
        build_bag(db3, [(1.0,0.,0.,0.), (1.6,1.5,0.,24.)])
        short = audit(db3, 'map','odom',.25,5.,.10)
        long = audit(db3, 'map','odom',.25,5.,2.0)
        assert short == 2, 'Short max-gap should be inconclusive'
        assert long == 1, '2-second max-gap must flag correction'

        db3b = Path(directory)/'stable.db3'
        build_bag(db3b, [(1.0,0.,0.,0.), (1.6,.02,.0,.2),
                          (2.2,.04,.0,.4)])
        assert audit(db3b,'map','odom',.25,5.,2.0)==0
    print('PASS: physical TF jump audit catches long-gap SLAM correction; stable case passes.')


if __name__ == '__main__':
    main()
