#!/usr/bin/env python3
"""Physical mapping mission stage: save observed SLAM map and plan a body-audited route.

Triggered only after /mission/first_lap_complete. Uses existing physical
geometry planning tools, never a simulator oracle. Does NOT publish /drive
or interact with VESC. All plan outputs are nominal and require further checks.
"""
import argparse
import json
import subprocess
import time
from datetime import datetime
from pathlib import Path

def run_step(command, label, out):
    proc = subprocess.run(command, text=True, capture_output=True,
                          timeout=180)
    (out/(label+'.stdout.log')).write_text(proc.stdout)
    (out/(label+'.stderr.log')).write_text(proc.stderr)
    if proc.returncode:
        raise RuntimeError(label+' failed (exit='+str(proc.returncode)+'): '+
                           (proc.stdout+proc.stderr)[-1200:])

def finalize(workspace, folder, lap_json):
    workspace = Path(workspace).expanduser().resolve()
    out = Path(folder).expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    plan = workspace/'tools/physical_planning'
    lap = json.loads(Path(lap_json).expanduser().read_text())
    if not lap.get('completed') or not lap.get('observed_corridor', {}).get('closed'):
        raise ValueError('No verified physical complete-lap and closed observed map')
    status = {'stage': 'SAVING_MAP', 'source': 'real SLAM /map and RF2O',
              'lap_evidence': lap, 'output': str(out)}
    status_file = out/'mission_status.json'
    status_file.write_text(json.dumps(status, indent=2)+'\n')
    prefix = out/'new_track'
    run_step(['ros2','run','nav2_map_server','map_saver_cli',
              '-t','/map','-f',str(prefix)], 'map_save', out)
    yaml_path = prefix.with_suffix('.yaml')
    pgm_path = prefix.with_suffix('.pgm')
    if not yaml_path.is_file() or not pgm_path.is_file():
        raise RuntimeError('Map saver did not create YAML and PGM files')

    status['stage'] = 'PLANNING'
    status_file.write_text(json.dumps(status, indent=2)+'\n')
    run_step(['python3',str(plan/'extract_robust_centerline.py'),
              str(yaml_path),'--out',str(prefix),
              '--step','0.10','--max-speed','0.35',
              '--expected-length',str(lap['distance_odom_m'])],
             'extract_centerline',out)
    centerline = out/'new_track_centerline.csv'
    run_step(['python3',str(plan/'prepare_vehicle_route.py'),
              str(yaml_path),str(centerline),
              '--out',str(out/'new_track_body_optimized.csv'),
              '--report',str(out/'route_preparation.json')],
             'prepare_route',out)
    report = json.loads((out/'route_preparation.json').read_text())
    if not report.get('accepted'):
        raise RuntimeError('Physical route geometry audit rejected proposed path')
    status['stage'] = 'COMPLETED_MAPPING_AND_PLANNING'
    status['note'] = (
        'Real map saved and nominal physically-sized route audited. '
        'No real Pure Pursuit, localization handoff or safe driving claimed.')
    status_file.write_text(json.dumps(status, indent=2)+'\n')
    print('PHYSICAL MAP + NOMINAL PLAN COMPLETE:', out, flush=True)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', default='~/f1tenth_uj')
    parser.add_argument('--evidence-root', default='~/f1tenth_hw_evidence')
    parser.add_argument('--offline-lap', default='',
                        help='Analyze existing completed-lap JSON (without ROS subscriptions)')
    args = parser.parse_args()
    evidence = Path(args.evidence_root).expanduser()
    out = evidence/('physical_mapping_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    if args.offline_lap:
        finalize(args.workspace, out, args.offline_lap)
        return
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import Bool
    class CompletionWatcher(Node):
        def __init__(self):
            super().__init__('physical_map_finalizer')
            self.complete = False
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            self.sub = self.create_subscription(
                Bool, '/mission/first_lap_complete', self.on_done, qos)
        def on_done(self, msg):
            if msg.data:
                self.complete = True
    watcher_started_at = time.time()
    rclpy.init()
    node = CompletionWatcher()
    try:
        while rclpy.ok() and not node.complete:
            rclpy.spin_once(node, timeout_sec=.2)
        if not node.complete:
            raise RuntimeError('ROS shutdown before first lap completion')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    # The mux subscribes to the same latch and must STOP; the mapping
    # stage does not issue drive commands or attempt to arm the car.
    lap_file = evidence/'physical_first_lap.json'
    deadline = time.monotonic()+3.0
    # Never consume stale completion evidence from a previous run.
    while time.monotonic() < deadline:
        if lap_file.is_file() and lap_file.stat().st_mtime >= watcher_started_at:
            break
        time.sleep(.1)
    if not lap_file.is_file() or lap_file.stat().st_mtime < watcher_started_at:
        raise RuntimeError('fresh first-lap evidence missing; refusing to use a previous run')
    try:
        finalize(args.workspace,out,lap_file)
    except Exception as exc:
        out.mkdir(parents=True,exist_ok=True)
        (out/'mission_status.json').write_text(json.dumps({
            'stage':'STOPPED','reason':str(exc)},indent=2)+'\n')
        raise

if __name__ == '__main__':
    main()
