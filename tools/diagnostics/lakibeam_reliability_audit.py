#!/usr/bin/env python3
"""READ-ONLY LakiBeam1 scan diagnosis. Subscribes to /scan; HTTP GET only.

No publishers, no firmware writes, no LiDAR or VESC commands.
An upstream LakiBeam ROS 2 driver converts raw distance=0 into +inf and
hardcodes LaserScan.range_max=100m; neither proves obstacles are >25m.
"""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import statistics
import time
from urllib.request import urlopen


def classify(v, rmin, rmax):
    if math.isnan(v):
        return "nan"
    if math.isinf(v):
        return "positive_inf" if v > 0 else "negative_inf"
    if v == 0 or v < rmin:
        return "zero_or_below_min"
    if v > rmax:
        return "above_range_max"
    return "measured"


def scan_bins(msg):
    """Half-degree sectors; preserve unknown returns as UNKNOWN."""
    bins = defaultdict(lambda: Counter())
    all_counts = Counter()
    for i, raw in enumerate(msg.ranges):
        angle = math.degrees(msg.angle_min + i*msg.angle_increment)
        k = classify(float(raw), float(msg.range_min), float(msg.range_max))
        all_counts[k] += 1
        if abs(angle) <= 40:
            bucket = bins[int(round(angle*2))]
            bucket['n'] += 1
            bucket[k] += 1
            if k == 'measured' and raw < .8:
                bucket['near_80cm'] += 1
    return all_counts, bins


def read_device(ip):
    data = {}
    for key, resource in (
        ("model", "/api/v1/system/firmware"),
        ("overview", "/api/v1/sensor/overview"),
        ("filter", "/api/v1/sensor/filter"),
    ):
        try:
            with urlopen("http://"+ip+resource, timeout=2.0) as response:
                body = json.loads(response.read(8192))
            if key == "model" and isinstance(body,dict):
                body = {k:v for k,v in body.items() if k.lower() not in (
                    'sn','serial','serial_number')}
            data[key] = body
        except Exception as exc:
            data[key] = {"error":str(exc)[:120]}
    return data


def self_test():
    assert classify(float('inf'),0,100) == 'positive_inf'
    assert classify(0.0,0,100) == 'zero_or_below_min'
    assert classify(101.0,0,100) == 'above_range_max'
    assert classify(2.0,0,100) == 'measured'
    assert classify(float('nan'),0,100) == 'nan'
    class Scan:
        ranges = [1.5,float('inf'),0.0,2.0]
        range_min = 0.0
        range_max = 100.0
        angle_min = -math.radians(.75)
        angle_increment = math.radians(.5)
    categories,bins=scan_bins(Scan)
    assert categories['measured']==2 and categories['positive_inf']==1
    assert categories['zero_or_below_min']==1
    assert len(bins)==4
    print("PASS: zero, infinity, out-of-range, finite and per-angle classification")
    print("No ROS/network/actuator access performed.")


def run(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    class Audit(Node):
        def __init__(self):
            super().__init__('lakibeam_readonly_audit')
            self.summary=[]
            self.bins=defaultdict(Counter)
            self.sub=self.create_subscription(
                LaserScan, args.topic, self.cb, qos_profile_sensor_data)
        def cb(self, msg):
            if not msg.ranges or not math.isfinite(msg.angle_increment):
                return
            all_counts, bins=scan_bins(msg)
            for key,c in bins.items():
                self.bins[key].update(c)
            forward=[c for key,c in bins.items() if abs(key)<=40]
            n=sum(c['n'] for c in forward)
            known=sum(c['measured'] for c in forward)
            inf=sum(c['positive_inf'] for c in forward)
            near=sum(c['near_80cm'] for c in forward)
            self.summary.append({
                'receive_monotonic_s':time.monotonic(),
                'ros_stamp_s':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                'scan_count':len(msg.ranges),
                'scan_time_s':round(float(msg.scan_time),5),
                'time_increment_s':round(float(msg.time_increment),8),
                'range_min':float(msg.range_min),
                'range_max':float(msg.range_max),
                'angle_min_deg':round(math.degrees(msg.angle_min),2),
                'angle_max_deg':round(math.degrees(msg.angle_max),2),
                'computed_final_angle_deg':round(math.degrees(
                    msg.angle_min+(len(msg.ranges)-1)*msg.angle_increment),2),
                'front_valid_fraction':round(known/n,4) if n else 0,
                'front_inf_fraction':round(inf/n,4) if n else 0,
                'front_below_0_8m_count':near,
                'categories_full_scan':dict(all_counts),
                'frame':msg.header.frame_id,
            })

    rclpy.init()
    node=Audit()
    try:
        end=time.monotonic()+args.seconds
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not node.summary:
        raise SystemExit("ERROR: No /scan messages received")
    rows=node.summary
    intervals=[rows[i]['receive_monotonic_s']-rows[i-1]['receive_monotonic_s']
               for i in range(1,len(rows))]
    per_angle=[
        {
            'angle_deg':key/2,
            'observations':c['n'],
            'positive_inf_fraction':round(c['positive_inf']/c['n'],4),
            'measured_fraction':round(c['measured']/c['n'],4),
            'near_0_8m_observations':c['near_80cm'],
        } for key,c in sorted(node.bins.items()) if c['n']
    ]
    persistent=[v['angle_deg'] for v in per_angle
                if v['observations']>=10 and v['positive_inf_fraction']>=.9]
    intermittent=[v['angle_deg'] for v in per_angle
                  if .1<v['positive_inf_fraction']<.9]
    fractions=[r['front_valid_fraction'] for r in rows]
    report={
        'warning':("Read only. Upstream driver maps raw distance ZERO to +INF. "
                   "It cannot distinguish beyond-range, weak reflection, filter "
                   "suppression or missing echo. Do not infer an empty path."),
        'device_http_get':None if args.no_http else read_device(args.sensor_ip),
        'scan_frames':len(rows),
        'observed_hz':round(1/statistics.mean(intervals),2) if intervals else None,
        'max_inter_scan_gap_s':round(max(intervals),4) if intervals else None,
        'latest_metadata':{k:v for k,v in rows[-1].items()
                           if k!='receive_monotonic_s'},
        'forward_valid_fraction_min':min(fractions),
        'forward_valid_fraction_median':round(statistics.median(fractions),4),
        'forward_valid_fraction_max':max(fractions),
        'fraction_scans_detecting_any_obstacle_under_0_8m':
            round(sum(r['front_below_0_8m_count']>0 for r in rows)/len(rows),4),
        'persistent_inf_half_degree_sectors':persistent,
        'intermittent_inf_half_degree_sectors':intermittent,
        'angle_data':per_angle,
    }
    out=Path(args.out).expanduser()
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps({k:v for k,v in report.items() if k!='angle_data'},indent=2))
    print("Full per-angle audit saved:",out)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--self-test',action='store_true')
    p.add_argument('--seconds',type=float,default=8.0)
    p.add_argument('--topic',default='/scan')
    p.add_argument('--sensor-ip',default='192.168.198.2')
    p.add_argument('--no-http',action='store_true')
    p.add_argument('--out',default='~/f1tenth_hw_evidence/lakibeam_audit.json')
    args=p.parse_args()
    if args.self_test:
        self_test()
    else:
        if not 2<=args.seconds<=60:
            p.error('--seconds must be between 2 and 60')
        run(args)


if __name__=='__main__':
    main()
