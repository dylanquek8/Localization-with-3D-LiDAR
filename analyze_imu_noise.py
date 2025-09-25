#!/usr/bin/env python3
# analyze_imu_noise.py
import pandas as pd
import numpy as np
import argparse
import math
from scipy.spatial.transform import Rotation as R

parser = argparse.ArgumentParser()
parser.add_argument('csv', help='CSV produced by recorder')
parser.add_argument('--unit', choices=['auto','g','mps2'], default='auto',
                    help='Accel units: auto (guess), g, or mps2')
parser.add_argument('--print-samples', action='store_true')
args = parser.parse_args()

df = pd.read_csv(args.csv)

# acceleration quaternion
q = [-0.701591, 0.712543, -0.00591461, -0.00427578]
R_imu_to_lidar = R.from_quat(q)

# Build timestamps in seconds (float)
df['t'] = df['sec'].astype(float) + df['nsec'].astype(float) * 1e-9
df = df.sort_values('t').reset_index(drop=True)

# compute dt series (use message header times)
dt_series = df['t'].diff().dropna().to_numpy()
mean_dt = np.mean(dt_series) if len(dt_series)>0 else 0.0
print(f"Samples: {len(df)}, mean dt = {mean_dt:.6f} s (std {np.std(dt_series):.6e})")

# accel and gyro raw
accel_raw = df[['ax','ay','az']].to_numpy()
gyro_raw  = df[['wx','wy','wz']].to_numpy()

# rotate into LiDAR/REP-103 frame
accel_rot = R_imu_to_lidar.apply(accel_raw)
gyro_rot  = R_imu_to_lidar.apply(gyro_raw)

# unpack rotated arrays
ax, ay, az = accel_rot[:,0], accel_rot[:,1], accel_rot[:,2]
wx, wy, wz = gyro_rot[:,0],  gyro_rot[:,1],  gyro_rot[:,2]

# detect units if auto: check magnitude of accel vector mean
mag_mean = np.mean(np.sqrt(ax*ax + ay*ay + az*az))
if args.unit == 'auto':
    # if magnitude near 1 => likely 'g', if near 9.8 => m/s^2
    if 0.8 <= mag_mean <= 1.2:
        unit = 'g'
    elif 8.0 <= mag_mean <= 11.0:
        unit = 'mps2'
    else:
        unit = 'unknown'
else:
    unit = args.unit

print(f"Detected accel magnitude mean ≈ {mag_mean:.4f} -> assuming units = {unit}")

# convert to m/s^2 if in g
if unit == 'g':
    g = 9.80665
    ax = ax * g
    ay = ay * g
    az = az * g

# gyro columns in deg/s, convert to rad/s
wx = df['wx'].to_numpy() * (np.pi / 180.0)
wy = df['wy'].to_numpy() * (np.pi / 180.0)
wz = df['wz'].to_numpy() * (np.pi / 180.0)

def stats(arr):
    return np.mean(arr), np.var(arr, ddof=1), np.std(arr, ddof=1)

ax_mean, ax_var, ax_std = stats(ax)
ay_mean, ay_var, ay_std = stats(ay)
az_mean, az_var, az_std = stats(az)

wx_mean, wx_var, wx_std = stats(wx)
wy_mean, wy_var, wy_std = stats(wy)
wz_mean, wz_var, wz_std = stats(wz)

print("\nAccelerometer (m/s^2) bias, var, std:")
print(f" ax: mean={ax_mean:.6e}, var={ax_var:.6e}, std={ax_std:.6e}")
print(f" ay: mean={ay_mean:.6e}, var={ay_var:.6e}, std={ay_std:.6e}")
print(f" az: mean={az_mean:.6e}, var={az_var:.6e}, std={az_std:.6e}")

print("\nGyro (rad/s) bias, var, std:")
print(f" wx: mean={wx_mean:.6e}, var={wx_var:.6e}, std={wx_std:.6e}")
print(f" wy: mean={wy_mean:.6e}, var={wy_var:.6e}, std={wy_std:.6e}")
print(f" wz: mean={wz_mean:.6e}, var={wz_var:.6e}, std={wz_std:.6e}")

# velocity variance increment per step: var_v += var_accel * dt^2
dt = mean_dt if mean_dt>0 else 0.01
print(dt)
var_vx_inc = ax_var * dt * dt
var_vy_inc = ay_var * dt * dt
var_vz_inc = az_var * dt * dt

print(f"\nUsing dt = {dt:.6f} s => per-step velocity variance increment:")
print(f" var_vx_inc = {var_vx_inc:.6e} (m^2/s^2)")
print(f" var_vy_inc = {var_vy_inc:.6e}")
print(f" var_vz_inc = {var_vz_inc:.6e}")

print("\nSuggested covariance matrix entries for TwistWithCovariance (diagonal indices):")
print(f" covariance[0] (var vx) initial: {var_vx_inc:.6e}")
print(f" covariance[7] (var vy) initial: {var_vy_inc:.6e}")
print(f" covariance[14] (var vz) initial: {var_vz_inc:.6e}")
print(f" covariance[21] (var wx) : {wx_var:.6e}")
print(f" covariance[28] (var wy) : {wy_var:.6e}")
print(f" covariance[35] (var wz) : {wz_var:.6e}")

print("\nBias (means) you should subtract before integration (ax_mean,ay_mean,az_mean):")
print(ax_mean, ay_mean, az_mean)

if args.print_samples:
    print("\nFirst 5 samples (converted if applicable):")
    head = pd.DataFrame({'t': df['t'].head(), 'ax': ax[:5], 'ay': ay[:5], 'az': az[:5],
                         'wx': wx[:5], 'wy': wy[:5], 'wz': wz[:5]})
    print(head.to_string(index=False))
