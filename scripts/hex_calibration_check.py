#!/usr/bin/env python3
"""Check a HEX calibration recording before fitting: are both sensors really delivering data?

    rosrun optoforce_sensor hex_calibration_check.py calibration/<run>/s1.npz --plot check.png
    rosrun optoforce_sensor hex_calibration_check.py my.bag --reference-topic /ft_sensor/wrench \
        --reference-limits 65,65,200,5,5,5 --reference-invalid-log "gauge saturated"

Per sensor: message rate and how many of the messages carry NEW values, frozen spans (the
driver kept publishing its last packet after the sensor stopped sending), gaps without messages,
the noise while nothing touches the handle, how hard the reference was loaded (with
--reference-limits) and its invalid-reading logs (bags, with --reference-invalid-log). Ends with
OK or what to fix.
"""
from __future__ import division, print_function

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ft_crosscal as fc  # noqa: E402


def spans_text(spans, t0, n=4):
    s = ', '.join('%.1f-%.1f s' % (a - t0, b - t0) for a, b in spans[:n])
    return s + (' and %d more' % (len(spans) - n) if len(spans) > n else '')


def quiet_window(sess, q, win=1.0):
    """The calmest 1 s of the reference where both sensors deliver new data (hands off)."""
    ref = sess.ref
    t0, t1 = max(ref.t[0], sess.dut.t[0]), min(ref.t[-1], sess.dut.t[-1])
    best = None
    lim = np.maximum(np.std(ref.w, 0), 1e-9)                  # normalise the axes
    for a in np.arange(t0, t1 - win, win / 2):
        if any(x < a + win and y > a for x, y in q['exclude']):
            continue
        m = (ref.t >= a) & (ref.t < a + win)
        if m.sum() < 10:
            continue
        score = float(np.sum(np.std(ref.w[m], 0) / lim))
        if best is None or score < best[0]:
            best = (score, a)
    return None if best is None else (best[1], best[1] + win)


def check(path, a):
    sess = fc.load_session(path, a.reference_topic, a.hex_topic, a.reference_invalid_log)
    q = fc.session_quality(sess, freeze_s=a.freeze)
    t0 = max(sess.ref.t[0], sess.dut.t[0])
    print('== %s: %.0f s of both sensors' % (path, q['overlap_s']))
    problems = []
    for key, who in (('ref', 'reference'), ('dut', 'HEX')):
        s = q[key]
        print('  %s: %d messages, %.0f/s, %.0f/s with new values (%.0f %% repeats)'
              % (who, s['n'], s['msg_rate'], s['new_rate'], 100 * s['repeat_frac']))
        if s['frozen']:
            print('      FROZEN %.0f s (same values repeated: the sensor stopped sending): %s'
                  % (s['frozen_s'], spans_text(s['frozen'], t0)))
            problems.append('%s frozen for %.0f s' % (who, s['frozen_s']))
        if s['gaps']:
            print('      no messages for %.1f s in %d gap(s): %s' % (s['gap_s'], len(s['gaps']), spans_text(s['gaps'], t0)))
            problems.append('%s silent for %.1f s' % (who, s['gap_s']))
        if 0.3 < s['repeat_frac'] and not s['frozen']:
            print('      every %s message is a copy of the previous one: the driver publishes faster than the '
                  'sensor delivers (harmless for the fit; copies are dropped)' % ('second' if s['repeat_frac'] < 0.6 else 'other'))
    w = quiet_window(sess, q)
    if w:
        mr = (sess.ref.t >= w[0]) & (sess.ref.t < w[1])
        hf = sess.dut.fresh()
        mh = (hf.t >= w[0]) & (hf.t < w[1])
        np.set_printoptions(precision=3, suppress=True)
        print('  noise in the calmest second (%.1f-%.1f s), standard deviation Fx Fy Fz [N] Tx Ty Tz [N*m]:'
              % (w[0] - t0, w[1] - t0))
        print('      reference ' + ' '.join('%7.3f' % v for v in np.std(sess.ref.w[mr], 0)))
        if mh.sum() > 5:
            print('      HEX       ' + ' '.join('%7.3f' % v for v in np.std(hf.w[mh], 0)))
    base = np.median(sess.ref.w[sess.ref.t < sess.ref.t[0] + 1.0], 0)
    peak = np.max(np.abs(sess.ref.w - base), 0)
    if a.reference_limits:
        load = peak / np.asarray(a.reference_limits)
        print('  reference highest load, % of rated: ' + '  '.join('%s %.0f' % (ax, 100 * v) for ax, v in zip(fc.AXES, load)))
        if np.any(load > 0.95):
            problems.append('reference above 95 %% of rated on %s' % ', '.join(ax for ax, v in zip(fc.AXES, load) if v > 0.95))
    else:
        print('  reference highest load: ' + '  '.join('%s %.4g' % (ax, v) for ax, v in zip(fc.AXES, peak)))
    if q['invalid']:
        print('  reference reported invalid readings %d time(s), at %s' % (len(q['invalid']), ', '.join(
            '%.0f s' % (x + 0.1 - t0) for x, _ in q['invalid'][:8])))
        problems.append('reference invalid %d time(s)' % len(q['invalid']))
    print('  usable for the fit: %.0f s of %.0f s' % (q['usable_s'], q['overlap_s']))
    if q['usable_s'] < fc.MIN_SEGMENT_S:
        print('  -> NOT USABLE. Fix the sensor connection (cable, connector, USB) and record again.')
    elif problems:
        print('  -> usable, with problems: %s' % '; '.join(problems))
    else:
        print('  -> OK')
    if a.plot:
        plot(sess, q, path, a.plot if len(a.files) == 1 else '%s_%s' % (os.path.splitext(os.path.basename(path))[0], a.plot))
    return q['usable_s'] >= fc.MIN_SEGMENT_S


def plot(sess, q, path, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t0 = max(sess.ref.t[0], sess.dut.t[0])
    ref, dut = sess.ref, sess.dut
    rb = np.median(ref.w[ref.t < ref.t[0] + 1.0], 0)
    db = np.median(dut.w[dut.t < dut.t[0] + 1.0], 0)
    step_r, step_d = max(1, len(ref.t) // 20000), max(1, len(dut.t) // 20000)
    fig, axs = plt.subplots(6, 1, figsize=(12, 13), sharex=True)
    for i, ax in enumerate(axs):
        ax.plot(ref.t[::step_r] - t0, ref.w[::step_r, i] - rb[i], lw=0.6, label='reference (tared)')
        ax.plot(dut.t[::step_d] - t0, dut.w[::step_d, i] - db[i], lw=0.6, alpha=0.8, label='HEX raw (tared, own axes)')
        for k, (a_, b_) in enumerate(q['dut']['frozen'] + q['ref']['frozen']):
            ax.axvspan(a_ - t0, b_ - t0, color='red', alpha=0.15, label='frozen' if k == 0 else None)
        for k, (a_, b_) in enumerate(q['exclude']):
            ax.axvspan(a_ - t0, b_ - t0, color='grey', alpha=0.15, label='left out of the fit' if k == 0 else None)
        ax.set_ylabel('%s [%s]' % (fc.AXES[i], fc.UNITS[i]))
        ax.grid(alpha=0.3)
    axs[0].legend(loc='upper right', fontsize=8, ncol=4)
    axs[0].set_title(os.path.basename(path))
    axs[-1].set_xlabel('time [s]')
    fig.tight_layout()
    fig.savefig(out, dpi=90)
    print('  plot: %s' % out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('files', nargs='+', help='.bag or .npz recordings')
    ap.add_argument('--reference-topic', default=None, help='reference sensor topic (bags only)')
    ap.add_argument('--reference-invalid-log', default='',
                    help='text the reference driver logs for invalid readings (bags only), e.g. "gauge saturated"')
    ap.add_argument('--reference-limits', type=lambda s: [float(x) for x in s.split(',')], default=None,
                    help='reference rated range Fx,Fy,Fz,Tx,Ty,Tz, to report how hard it was loaded')
    ap.add_argument('--hex-topic', default='/optoforce_node/OptoForceWrench', help='HEX topic (bags only)')
    ap.add_argument('--freeze', type=float, default=fc.FREEZE_S,
                    help='same values for longer than this = frozen (default %g s)' % fc.FREEZE_S)
    ap.add_argument('--plot', help='write a plot of both sensors with the bad spans marked (PNG)')
    a = ap.parse_args()
    ok = True
    for f in a.files:
        try:
            ok &= check(f, a)
        except (ValueError, IOError, OSError) as e:
            print('== %s: %s' % (f, e))
            ok = False
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
