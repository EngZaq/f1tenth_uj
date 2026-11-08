#!/usr/bin/env python3
"""Offline checks: teammate EKF port must never own physical TF or actuators."""
import math
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[2]
cfg=ROOT/'src/f1tenth_bringup/config/ekf_moatasem_vesc_imu_shadow.yaml'
p=yaml.safe_load(cfg.read_text())['ekf_moatasem_shadow']['ros__parameters']

assert p['publish_tf'] is False, 'Shadow EKF MUST NOT publish odom->base TF'
assert p['two_d_mode'] is True
assert p['world_frame']=='odom' and p['odom_frame']=='odom'
assert p['base_link_frame']=='base_link'
assert p['odom0']=='/shadow_inputs/odom_vesc'
assert p['imu0']=='/imu/data_raw'
assert len(p['odom0_config'])==15 and len(p['imu0_config'])==15
assert [i for i,v in enumerate(p['odom0_config']) if v] == [6], 'Only VESC vx should be fused'
assert [i for i,v in enumerate(p['imu0_config']) if v] == [11], 'Only IMU yaw rate should be fused'
matrix=p['process_noise_covariance']
assert len(matrix)==225, 'Q must be 15 x 15'
for i in range(15):
    assert math.isfinite(float(matrix[i*15+i])) and float(matrix[i*15+i])>=0
for i in range(15):
    for j in range(15):
        assert abs(float(matrix[i*15+j])-float(matrix[j*15+i]))<1e-10
assert not any(k.startswith('odom1') for k in p), 'RF2O must not be silently fused into Config C'

drive=yaml.safe_load((ROOT/'src/f1tenth_bringup/config/ackermann_to_vesc.yaml').read_text())
wheel=yaml.safe_load((ROOT/'src/f1tenth_bringup/config/vesc_to_odom.yaml').read_text())
gain=float(drive['/ackermann_to_vesc_node']['ros__parameters']['speed_to_erpm_gain'])
gain_odom=float(wheel['/vesc_to_odom_node']['ros__parameters']['speed_to_erpm_gain'])
assert gain and gain_odom
print('PASS: shadow EKF only fuses real VESC vx + IMU gyro z; does not publish TF')
print('WARNING: physical Ackermann output gain = %.1f, wheel-odom conversion gain = %.1f;' %
      (gain,gain_odom))
print('         measure actual forward ERPM sign before trusting fused velocity.')
