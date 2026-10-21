#!/usr/bin/env python3
"""Simulate a calibration recording: HEX on top of a reference F/T sensor, a hand pushing a handle.

Used by the tests, and to try the fit without hardware:

    python3 test/simulate_crosscal.py --out /tmp/sim1.npz
    python3 scripts/hex_calibration_fit.py /tmp/sim1.npz

Everything about the "truth" (mount, HEX errors, lag) is printed and stored in the file's meta.
"""
from __future__ import division, print_function

import argparse
import math

import numpy as np

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import ft_crosscal as fc  # noqa: E402

HEX_NOISE = np.array([0.035, 0.035, 0.15, 0.002, 0.002, 0.001])      # OnRobot HEX-E, 1 sigma
REF_NOISE = np.array([0.012, 0.012, 0.025, 0.0008, 0.0008, 0.0008])   # reference (like an ATI Gamma)
REF_RATED = np.array([65.0, 65.0, 200.0, 5.0, 5.0, 5.0])               # its rated range
HEX_COMPLIANCE = np.radians([2.5 / 10, 2.5 / 10, 5.0 / 6.5])           # rad per N*m (HEX-E datasheet)


def smooth_noise(rng, n, fs, f_hi, k=24):
    """Zero-mean smooth random signal (sum of sines up to f_hi Hz), unit peak."""
    t = np.arange(n) / fs
    x = np.zeros(n)
    for _ in range(k):
        f = rng.uniform(0.05, f_hi)
        x += rng.uniform(0.3, 1.0) * np.sin(2 * math.pi * f * t + rng.uniform(0, 2 * math.pi))
    return x / np.max(np.abs(x))


def hand_loads(rng, duration, fs, grip_low, grip_high):
    """Wrench at the HEX native origin (N, N*m) from a hand on the handle, in phases.

    Each phase excites a few components; 2 s rest between phases (hands off).
    grip_*: grip point heights above the HEX native origin (m)."""
    n = int(duration * fs)
    W = np.zeros((n, 6))
    modes = [('fx',), ('fy',), ('fz',), ('fx', 'fy'), ('cz',), ('cx', 'cy'), ('fx', 'cy'),
             ('fy', 'cx'), ('fx', 'fy', 'fz', 'cx', 'cy', 'cz')]
    amp = {'fx': 30.0, 'fy': 30.0, 'fz': 70.0, 'cx': 1.5, 'cy': 1.5, 'cz': 2.0}
    phase_len, rest = 14.0, 2.0
    i, k = 0, 0
    while i < n:
        mode = modes[k % len(modes)]
        grip = grip_high if (k // len(modes)) % 2 else grip_low
        L = min(int(phase_len * fs), n - i)
        r = int(rest * fs)
        seg = slice(i + r, i + L)
        m = max(L - r, 1)
        env = np.sin(np.linspace(0, math.pi, m)) ** 0.5
        f = np.zeros((m, 3))
        c = np.zeros((m, 3))
        for j, ax in enumerate('xyz'):
            if 'f' + ax in mode:
                f[:, j] = amp['f' + ax] * env * smooth_noise(rng, m, fs, 0.9)
            if 'c' + ax in mode:
                c[:, j] = amp['c' + ax] * env * smooth_noise(rng, m, fs, 0.9)
        rg = np.array([0.0, 0.0, grip]) + rng.normal(0, 0.003, 3)       # hand position wanders
        W[seg, :3] = f
        W[seg, 3:] = c + np.cross(rg, f)
        i += L
        k += 1
    return W


def simulate_session(duration=200.0, seed=1, sensor_seed=0, fs=1000.0, yaw_deg=22.5, tilt_deg=(0.4, -0.3),
                     origin_mm=(0.3, -0.2, 37.0), gain_err=0.04, crosstalk=0.04, lag_s=0.004,
                     drift=(0.8, 0.8, 3.0, 0.03, 0.03, 0.01), compliance=False, mirror_fz=False,
                     quad=None):
    """Return (Session, truth dict). sensor_seed fixes the HEX's errors; seed the loads and noise."""
    srng = np.random.RandomState(sensor_seed)
    rng = np.random.RandomState(seed)
    R = fc.rpy_to_matrix(math.radians(tilt_deg[0]), math.radians(tilt_deg[1]), math.radians(yaw_deg))
    p = np.array(origin_mm) / 1000.0
    X = fc.wrench_transform(R, p)
    # HEX error: raw = G w_true + drift + noise
    G = np.eye(6) + np.diag(srng.uniform(-gain_err, gain_err, 6))
    off = srng.uniform(-crosstalk, crosstalk, (6, 6))
    np.fill_diagonal(off, 0)
    unit = np.array([1, 1, 1, 0.05, 0.05, 0.05])                         # N per N*m scale
    G += off * unit[:, None] / unit[None, :]
    if mirror_fz:
        G[2, :] *= -1
    W_n = hand_loads(rng, duration, fs, grip_low=0.050, grip_high=0.120)
    W_true = W_n.copy()
    if compliance:
        # the HEX top tilts under torque; the reference sees the wrench rotated by that tilt
        th = W_n[:, 3:] * HEX_COMPLIANCE
        W_a_local = W_n.copy()
        W_a_local[:, :3] = W_n[:, :3] + np.cross(th, W_n[:, :3])
        W_a_local[:, 3:] = W_n[:, 3:] + np.cross(th, W_n[:, 3:])
    else:
        W_a_local = W_n
    W_a = W_a_local.dot(X.T)
    # keep the reference inside 90 % of its rated range by scaling the loads
    peak = np.max(np.abs(W_a) / REF_RATED, 1)
    s = np.minimum(1.0, 0.9 / np.maximum(peak, 1e-9))
    s = np.convolve(s, np.ones(501) / 501, 'same')                       # smooth the scaling
    s = np.minimum(s, 0.9 / np.maximum(peak, 1e-9))
    W_true *= s[:, None]
    W_a *= s[:, None]
    n = len(W_a)
    t = np.arange(n) / fs + 1000.0
    gravity = X.dot([0, 0, -4.2, 0.01, -0.02, 0])                        # parts above the reference
    ref = W_a + gravity + rng.normal(0, 1, (n, 6)) * REF_NOISE
    tau = (t - t[0]) / (t[-1] - t[0])
    b = np.outer(tau, drift) + rng.uniform(-2, 2, 6) * unit
    raw = W_true.dot(G.T) + b + rng.normal(0, 1, (n, 6)) * HEX_NOISE
    if quad is not None:
        raw = raw + quad(W_true)
    # stamps: reference exact; HEX = receive time = true + lag + jitter, at its own clock
    hex_t = t + lag_s + rng.exponential(0.0003, n)
    sess = fc.Session(fc.Series(t, ref), fc.Series(hex_t, raw), name='sim%d' % seed)
    truth = {'R': R, 'p': p, 'G': G, 'K_true': np.linalg.inv(G), 'lag_s': lag_s + 0.0003,
             'yaw_deg': yaw_deg, 'origin_mm': origin_mm}
    return sess, truth


def degrade(sess, republish=False, freeze=None, garbage_s=0.0, jump=None, seed=5):
    """The HEX failures seen on the real setup, applied to a simulated session (in place).

    republish: the DAQ delivers 500 Hz but the driver publishes 1000 Hz, every value twice.
    freeze: (t0, t1) seconds from the start: the sensor stops sending and the driver republishes
            its last packet (t1 None = to the end); garbage_s of wild values before it.
    jump: offset (6,) added to the HEX after the freeze (it came back with another zero)."""
    rng = np.random.RandomState(seed)
    t, w = sess.dut.t, sess.dut.w.copy()
    if republish:
        w[1::2] = w[0::2][:len(w[1::2])]
    if freeze is not None:
        t0 = t[0] + freeze[0]
        t1 = t[-1] + 1 if freeze[1] is None else t[0] + freeze[1]
        g = (t >= t0 - garbage_s) & (t < t0)
        if g.any():
            w[g] += np.cumsum(rng.normal(0, 1, (g.sum(), 6)), 0) * [2, 2, 8, 0.2, 0.2, 0.01]
        f = (t >= t0) & (t < t1)
        last = np.flatnonzero(t < t0)[-1]
        w[f] = w[last]
        if jump is not None:
            w[t >= t1] += np.asarray(jump, float)
    sess.dut = fc.Series(t, w)
    return sess


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default='sim_session.npz')
    ap.add_argument('--duration', type=float, default=200.0)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--compliance', action='store_true', help='include the HEX tilting under torque')
    ap.add_argument('--mirror-fz', action='store_true', help='HEX reports Fz with the opposite sign')
    ap.add_argument('--republish', action='store_true', help='every HEX value published twice (500 Hz DAQ)')
    ap.add_argument('--freeze-at', type=float, help='the HEX stops sending at this second; its last '
                    'value is republished to the end (with 2 s of garbage before)')
    a = ap.parse_args()
    sess, tr = simulate_session(a.duration, a.seed, compliance=a.compliance, mirror_fz=a.mirror_fz)
    if a.republish or a.freeze_at is not None:
        degrade(sess, republish=a.republish, garbage_s=2.0 if a.freeze_at is not None else 0.0,
                freeze=None if a.freeze_at is None else (a.freeze_at, None))
    meta = {'simulated': True, 'yaw_deg': tr['yaw_deg'], 'origin_mm': list(tr['origin_mm']),
            'hex_delay_ms': tr['lag_s'] * 1e3, 'K_true': np.round(tr['K_true'], 6).tolist()}
    fc.save_session_npz(a.out, sess.ref, sess.dut, meta)
    print('wrote %s: %.0f s, mount yaw %.1f deg, origin %s mm, HEX stamps %.2f ms late'
          % (a.out, a.duration, tr['yaw_deg'], tr['origin_mm'], tr['lag_s'] * 1e3))
    np.set_printoptions(precision=4, suppress=True)
    print('true correction K (HEX raw -> true wrench in the HEX frame):')
    print(tr['K_true'])


if __name__ == '__main__':
    main()
