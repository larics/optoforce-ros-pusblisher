#!/usr/bin/env python3
"""Offline (re)fit of the HEX calibration from recordings of the HEX on a reference F/T sensor.

    rosrun optoforce_sensor hex_calibration_fit.py calibration/<run>/s1.npz calibration/<run>/s2.npz
    rosrun optoforce_sensor hex_calibration_fit.py my.bag --reference-topic /ft_sensor/wrench

Writes hex_calibration.yaml and hex_calibration_report.txt next to the first recording (or --out),
optionally a plot (--plot). --activate also makes it the active calibration of this package
(cfg/hex_calibration.yaml, loaded by hex_calibrated.launch).
"""
from __future__ import division, print_function

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ft_crosscal as fc  # noqa: E402
from crosscal_ros import calibration_file  # noqa: E402


def parse_list(s, n):
    v = [float(x) for x in s.split(',')]
    if len(v) != n:
        raise argparse.ArgumentTypeError('need %d comma-separated numbers' % n)
    return v


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('recordings', nargs='+', help='.npz from hex_calibration_node.py or ROS1 .bag files')
    ap.add_argument('--out', default=None,
                    help='calibration file to write (default: hex_calibration.yaml next to the first recording)')
    ap.add_argument('--activate', action='store_true',
                    help='also copy it to this package\'s cfg/hex_calibration.yaml (the active calibration)')
    ap.add_argument('--model', choices=['linear', 'quadratic'], default='linear',
                    help='linear: 6x6 matrix (gains + cross-talk). quadratic: also products of axes '
                         '(non-linearity, the HEX tilting under load); needs more data')
    ap.add_argument('--drift', default='linear',
                    help='offset drift during a recording: none | linear | knots:SECONDS (default linear)')
    ap.add_argument('--rate', type=float, default=100.0, help='fit sample rate, Hz (default 100)')
    ap.add_argument('--cutoff', type=float, default=5.0, help='low-pass on both sensors, Hz (default 5)')
    ap.add_argument('--max-lag', type=float, default=0.1, help='largest stamp delay searched, s')
    ap.add_argument('--delay-ms', type=float, default=None, help='use this HEX stamp delay instead of estimating it')
    ap.add_argument('--frame', choices=['native', 'top', 'reference'], default='native',
                    help='output frame: native = where the HEX itself measures (default; a corrected '
                         'drop-in for the raw topic); top = HEX axes, origin --top-mm above the reference '
                         'origin (e.g. the HEX top flange); reference = the reference sensor\'s frame')
    ap.add_argument('--top-mm', type=float, default=None, help='height for --frame top, mm above the reference origin')
    ap.add_argument('--frame-name', default=None, help='frame_id written into the calibration')
    ap.add_argument('--nominal-yaw-deg', type=float, default=None,
                    help='CAD yaw of the HEX axes in the reference frame, to compare with the identified mount')
    ap.add_argument('--nominal-z-mm', type=float, default=None,
                    help='CAD height of the HEX measuring origin above the reference origin')
    ap.add_argument('--reference-limits', type=lambda s: parse_list(s, 6), default=None,
                    help='reference rated range Fx,Fy,Fz,Tx,Ty,Tz: samples above 95 %% are not used '
                         '(default: not checked)')
    ap.add_argument('--hex-limits', type=lambda s: parse_list(s, 6), default=list(fc.HEX_E_CAPACITY),
                    help='HEX capacity Fx,..,Tz (default HEX-E: 200,200,200,10,10,6.5)')
    ap.add_argument('--reference-topic', default=None, help='reference sensor topic (bags only)')
    ap.add_argument('--reference-invalid-log', default='',
                    help='text the reference driver logs for invalid readings, e.g. "gauge saturated" (bags only)')
    ap.add_argument('--hex-topic', default='/optoforce_node/OptoForceWrench', help='HEX topic (bags only)')
    ap.add_argument('--plot', default=None, help='write a PNG with reference / factory / calibrated')
    a = ap.parse_args()

    try:
        sessions = [fc.load_session(p, a.reference_topic, a.hex_topic, a.reference_invalid_log) for p in a.recordings]
    except ValueError as e:
        ap.error(str(e))
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.recordings[0])), 'hex_calibration.yaml')
    for s in sessions:
        print('%s: reference %d samples (%.0f Hz), HEX %d samples (%.0f Hz)'
              % (s.name, len(s.ref), s.ref.rate(), len(s.dut), s.dut.rate()))
    if a.frame == 'top':
        if a.top_mm is None:
            ap.error('--frame top needs --top-mm')
        spec = {'mode': 'native_at', 'xyz_mm': [0.0, 0.0, a.top_mm], 'name': a.frame_name or 'hex_top'}
    elif a.frame == 'reference':
        spec = {'mode': 'reference', 'name': a.frame_name or 'reference_sensor'}
    else:
        spec = {'mode': 'native', 'name': a.frame_name or 'hex_calibrated'}
    nominal = None
    if a.nominal_yaw_deg is not None or a.nominal_z_mm is not None:
        nominal = (fc.rot_z(np.radians(a.nominal_yaw_deg or 0.0)),
                   np.array([0.0, 0.0, (a.nominal_z_mm or 0.0) / 1000.0]))
    try:
        cal, rep = fc.fit(sessions, model=a.model, drift=a.drift, rate=a.rate, cutoff=a.cutoff,
                          max_lag=a.max_lag, lag=None if a.delay_ms is None else a.delay_ms / 1000.0,
                          ref_limits=a.reference_limits, dut_limits=a.hex_limits, frame_spec=spec, nominal=nominal)
    except ValueError as e:
        print('\nFIT NOT POSSIBLE: %s' % e)
        print('Details per recording: rosrun optoforce_sensor hex_calibration_check.py <recordings> --plot check.png')
        return 1
    text = fc.format_report(cal, rep)
    print()
    print(text)
    cal.save(out)
    rpt = os.path.splitext(out)[0] + '_report.txt'
    with open(rpt, 'w') as f:
        f.write(text + '\n')
    print()
    print('wrote %s and %s' % (out, rpt))
    if a.plot:
        plot(rep, cal, a.plot)
    if cal.info.get('poor_axes'):
        print('\nWARNING: %s' % cal.info['notes'][0])
        if a.activate:
            print('not activated')
        return 2
    if a.activate:
        import shutil
        shutil.copyfile(out, calibration_file())
        print('active calibration: %s' % calibration_file())
    return 0


def plot(rep, cal, path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('(matplotlib not installed: no plot)')
        return
    d = rep['design']
    Xn = fc.wrench_transform_inv(rep['identified']['R'], rep['identified']['p'])
    raw_out = d.lin * np.sign(np.diag(Xn.dot(rep['M_ref_from_raw'])))
    ref, pred = rep['ref_out'], rep['pred_out']
    # sessions one after another on the time axis, each tared by its median, gaps between them
    T, Y = [], {'ref': [], 'raw': [], 'cal': []}
    t_off = 0.0
    for s in np.unique(d.sid):
        m = d.sid == s
        tt = d.t[m] - d.t[m][0] + t_off
        T += [tt, [np.nan]]
        for key, arr in (('ref', ref), ('raw', raw_out), ('cal', pred)):
            Y[key] += [arr[m] - np.median(arr[m], 0), np.full((1, 6), np.nan)]
        t_off = tt[-1] + 5.0
    t = np.concatenate(T)
    Y = {k: np.vstack(v) for k, v in Y.items()}
    fig, ax = plt.subplots(6, 2, figsize=(14, 13), gridspec_kw={'width_ratios': [3, 1]})
    for i, a in enumerate(fc.AXES):
        ax[i, 0].plot(t, Y['ref'][:, i], color='#1f2937', lw=1.2, label='reference')
        ax[i, 0].plot(t, Y['raw'][:, i], color='#d97706', lw=0.8, label='HEX factory')
        ax[i, 0].plot(t, Y['cal'][:, i], color='#2563eb', lw=0.8, ls='--', label='HEX calibrated')
        ax[i, 0].set_ylabel('%s [%s]' % (a, fc.UNITS[i]))
        ax[i, 1].plot(Y['ref'][:, i], Y['raw'][:, i] - Y['ref'][:, i], '.', ms=1, color='#d97706')
        ax[i, 1].plot(Y['ref'][:, i], Y['cal'][:, i] - Y['ref'][:, i], '.', ms=1, color='#2563eb')
        ax[i, 1].set_ylabel('error [%s]' % fc.UNITS[i])
        ax[i, 1].axhline(0, color='#9ca3af', lw=0.5)
        if i < 5:
            ax[i, 0].tick_params(labelbottom=False)
    ax[0, 0].legend(loc='upper right', fontsize=8)
    ax[-1, 0].set_xlabel('time [s]')
    ax[-1, 1].set_xlabel('reference value')
    fig.suptitle('HEX vs reference in frame "%s" (each session tared; factory error includes drift)' % cal.frame['name'])
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print('wrote %s' % path)


if __name__ == '__main__':
    sys.exit(main())
