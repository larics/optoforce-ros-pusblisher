#!/usr/bin/env python3
"""Tests of the HEX calibration maths with simulated recordings (no ROS, no hardware):

    python3 test/test_hex_calibration.py
"""
from __future__ import division, print_function

import math
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import ft_crosscal as fc                      # noqa: E402
from simulate_crosscal import degrade, simulate_session  # noqa: E402

FAIL = []


def check(name, ok, detail=''):
    print('%s  %-58s %s' % ('PASS' if ok else 'FAIL', name, detail))
    if not ok:
        FAIL.append(name)


def quiet(*a, **k):
    pass


def test_transform_algebra():
    rng = np.random.RandomState(0)
    R = fc.rpy_to_matrix(0.3, -0.2, 1.1)
    p = np.array([0.01, -0.02, 0.05])
    X, Xi = fc.wrench_transform(R, p), fc.wrench_transform_inv(R, p)
    check('wrench transform inverse', np.allclose(X.dot(Xi), np.eye(6)))
    # torque about A of a force f applied at point r (given in B) equals X . wrench_B
    f_b, r_b = rng.normal(size=3), rng.normal(size=3) * 0.1
    w_b = np.concatenate([f_b, np.cross(r_b, f_b)])
    r_a, f_a = R.dot(r_b) + p, R.dot(f_b)
    w_a = np.concatenate([f_a, np.cross(r_a, f_a)])
    check('force at a point: X maps B wrench to A', np.allclose(X.dot(w_b), w_a))
    r, pi, y = fc.matrix_to_rpy(R)
    check('rpy round trip', np.allclose([r, pi, y], [0.3, -0.2, 1.1]))
    # power is frame independent: twist . wrench (twist maps with X^-T)
    check('wrench transform form [R 0; S(p)R R]', np.allclose(X[3:, :3], fc.skew(p).dot(R)))


def test_full_map_and_lag():
    sess, tr = simulate_session(duration=200, seed=3)
    cal, rep = fc.fit([sess], log=quiet)
    M_true = fc.wrench_transform(tr['R'], tr['p']).dot(tr['K_true'])
    M = rep['M_ref_from_raw']
    rel = np.abs(M - M_true) / np.maximum(np.abs(M_true).max(1, keepdims=True), 1e-9)
    check('full map reference <- HEX raw recovered', rel.max() < 0.004, 'max err %.2f %% of row scale' % (100 * rel.max()))
    lag = cal.info['hex_delay_ms'][0]
    check('HEX stamp delay recovered', abs(lag - tr['lag_s'] * 1e3) < 0.5,
          '%.2f ms (true %.2f)' % (lag, tr['lag_s'] * 1e3))
    cv = np.array([cal.info['error_rms_crossval'][a] for a in fc.AXES])
    raw = rep['rms_raw']
    check('cross-validated error far below factory error', np.all(cv < 0.15 * raw),
          'Fx %.3f vs %.3f N, Tz %.4f vs %.4f N*m' % (cv[0], raw[0], cv[5], raw[5]))
    check('calibrated error at the noise level', cv[0] < 0.05 and cv[2] < 0.1 and cv[3] < 0.002,
          'RMS Fx %.3f N, Fz %.3f N, Tx %.4f N*m' % (cv[0], cv[2], cv[3]))
    check('excitation well conditioned', rep['coverage']['cond'] < 10, 'cond %.1f' % rep['coverage']['cond'])
    return sess, tr, cal


def test_frame_identification():
    # symmetric cross-talk only: then rotation/offset are exactly identifiable
    sess, tr = simulate_session(duration=150, seed=5, crosstalk=0.0)
    cal, rep = fc.fit([sess], log=quiet)
    idf = rep['identified']
    ang = math.degrees(fc.rotation_angle(idf['R'].T.dot(tr['R'])))
    dp = (idf['p'] - tr['p']) * 1000
    check('mount rotation identified', ang < 0.05, '%.3f deg off' % ang)
    check('HEX origin identified', np.abs(dp).max() < 0.2, 'off by (%.2f %.2f %.2f) mm' % tuple(dp))
    K = cal.K
    check('native-frame K = inverse of HEX gains', np.allclose(K, tr['K_true'], atol=2e-3),
          'max diff %.4f' % np.abs(K - tr['K_true']).max())


def test_mirrored_hex():
    sess, tr = simulate_session(duration=120, seed=7, mirror_fz=True)
    cal, rep = fc.fit([sess], log=quiet)
    m = cal.info['identified_mount']
    check('HEX with flipped Fz detected', m['mirrored_axes'] == ['Fz'], str(m['mirrored_axes']))
    R = np.array(cal.frame['R_in_reference'])
    check('output frame stays right-handed', abs(np.linalg.det(R) - 1) < 1e-9)
    cv = np.array([cal.info['error_rms_crossval'][a] for a in fc.AXES])
    check('calibration still accurate', cv[2] < 0.1, 'Fz RMS %.3f N' % cv[2])


def test_quadratic_compliance():
    sess, tr = simulate_session(duration=200, seed=9, compliance=True)
    lin, _ = fc.fit([sess], model='linear', log=quiet)
    qua, _ = fc.fit([sess], model='quadratic', log=quiet)
    el = np.array([lin.info['error_rms_crossval'][a] for a in fc.AXES])
    eq = np.array([qua.info['error_rms_crossval'][a] for a in fc.AXES])
    check('HEX tilt under load: quadratic beats linear', eq[0] < 0.7 * el[0] and eq[1] < 0.7 * el[1],
          'Fx %.3f -> %.3f N, Fy %.3f -> %.3f N' % (el[0], eq[0], el[1], eq[1]))


def test_two_sessions_and_custom_frame():
    s1, tr = simulate_session(duration=100, seed=11)
    s2, _ = simulate_session(duration=100, seed=12)
    cal, rep = fc.fit([s1, s2], frame_spec={'mode': 'native_at', 'xyz_mm': [0, 0, 74.0], 'name': 'hex_top'},
                      log=quiet)
    check('two sessions with separate offsets', cal.info['error_rms_crossval']['Fx'] < 0.06,
          'Fx RMS %.3f N' % cal.info['error_rms_crossval']['Fx'])
    # a force through the new origin must give zero torque there
    R = np.array(cal.frame['R_in_reference'])
    p = np.array(cal.frame['xyz_in_reference_mm']) / 1000
    w_out = np.array([3.0, -2.0, 10.0, 0, 0, 0])
    w_ref = fc.wrench_transform(R, p).dot(w_out)
    raw = np.linalg.solve(rep['M_ref_from_raw'], w_ref)
    back = cal.apply(raw)[0]
    check('output frame at the HEX top: pure force stays pure', np.allclose(back, w_out, atol=1e-6))


def test_yaml_and_apply():
    sess, tr = simulate_session(duration=60, seed=13)
    cal, rep = fc.fit([sess], model='quadratic', log=quiet, folds=0)
    path = os.path.join(tempfile.mkdtemp(), 'cal.yaml')
    cal.save(path)
    back = fc.Calibration.load(path)
    x = np.random.RandomState(1).normal(size=(5, 6)) * [20, 20, 50, 1, 1, 1]
    check('YAML round trip (quadratic model)', np.allclose(back.apply(x), cal.apply(x), rtol=1e-6, atol=1e-6))
    check('report formats', 'Calibration matrix' in fc.format_report(cal, rep))


def test_bad_streams():
    """The failures seen on the real setup (the HEX stops sending, its driver republishes the last
    packet; every value published twice) must be detected and kept out of the fit."""
    s, tr = simulate_session(duration=120, seed=21)
    degrade(s, republish=True)
    q = fc.session_quality(s)
    check('republished copies detected', abs(q['dut']['repeat_frac'] - 0.5) < 0.01 and not q['text'],
          'repeats %.0f %%, new values %.0f/s' % (100 * q['dut']['repeat_frac'], q['dut']['new_rate']))
    cal, rep = fc.fit([s], log=quiet)
    d = cal.info['hex_delay_ms'][0]
    check('copies do not bias the delay or the fit', abs(d - tr['lag_s'] * 1e3) < 0.6
          and cal.info['error_rms_crossval']['Fx'] < 0.06,
          'delay %.2f ms (true %.2f), Fx cv %.3f N' % (d, tr['lag_s'] * 1e3, cal.info['error_rms_crossval']['Fx']))

    # the user's bag: garbage, then frozen from 9 s to the end
    s, _ = simulate_session(duration=120, seed=22)
    degrade(s, republish=True, freeze=(9.0, None), garbage_s=2.0)
    q = fc.session_quality(s)
    check('frozen HEX detected', q['dut']['frozen'] and abs(q['dut']['frozen'][0][0] - s.dut.t[0] - 9.0) < 0.01,
          q['text'][0][:70])
    try:
        fc.fit([s], log=quiet)
        check('fit refuses a frozen HEX', False, 'it fitted')
    except ValueError as e:
        check('fit refuses a frozen HEX', 'no usable data' in str(e) and 'frozen' in str(e), str(e)[:60])

    # frozen near the end, garbage before it: the rest is used, the garbage is not
    s, _ = simulate_session(duration=160, seed=23)
    degrade(s, freeze=(130.0, None), garbage_s=2.0)
    cal, rep = fc.fit([s], log=quiet)
    tmax = rep['design'].t.max() - s.ref.t[0]
    check('frozen tail and the garbage before it left out', tmax < 128.0 and cal.info['error_rms_crossval']['Fz'] < 0.1
          and any('frozen' in n for n in cal.info['notes']),
          'data used up to %.1f s, Fz cv %.3f N' % (tmax, cal.info['error_rms_crossval']['Fz']))

    # outage in the middle, the HEX comes back with another zero: two segments, two offsets
    s, _ = simulate_session(duration=160, seed=24)
    degrade(s, freeze=(70.0, 85.0), garbage_s=1.0, jump=[3, -2, 15, 0.2, -0.1, 0.05])
    cal, rep = fc.fit([s], log=quiet)
    check('outage with an offset jump: separate segments', len(np.unique(rep['design'].sid)) == 2
          and cal.info['error_rms_crossval']['Fz'] < 0.1,
          'Fz cv %.3f N' % cal.info['error_rms_crossval']['Fz'])

    # the reference driver logged invalid readings (e.g. gauge saturation): those seconds are excluded
    s, _ = simulate_session(duration=60, seed=25)
    t0 = s.ref.t[0]
    s.meta['ref_invalid'] = [fc.invalid_window(t0 + 30.0)]
    bad = (s.ref.t > t0 + 30.0) & (s.ref.t < t0 + 31.0)
    s.ref.w[bad] += [40, 0, 0, 0, 3, 0]
    cal, rep = fc.fit([s], log=quiet, folds=0)
    tt = rep['design'].t - t0
    check('reference invalid windows left out', not np.any((tt > 29.9) & (tt < 31.1))
          and cal.info['error_rms']['Fx'] < 0.05, 'Fx RMS %.3f N' % cal.info['error_rms']['Fx'])

    # a HEX that does not follow the reference at all (wrong topic, broken sensor)
    s, _ = simulate_session(duration=60, seed=26)
    s.dut = fc.Series(s.dut.t, np.random.RandomState(3).normal(0, 1, s.dut.w.shape).cumsum(0) * 0.05)
    cal, rep = fc.fit([s], log=quiet, folds=0)
    check('HEX not following the reference is flagged', len(cal.info['poor_axes']) >= 4
          and 'WARNING' in fc.format_report(cal, rep), 'poor axes %s' % cal.info['poor_axes'])


def test_file_compat_and_limits():
    sess, tr = simulate_session(duration=60, seed=31)
    cal, rep = fc.fit([sess], log=quiet, folds=0)
    d = cal.to_dict()
    f = d['frame']                                  # write it the way the first version did
    f['R_in_ati'], f['xyz_in_ati_mm'], f['rpy_in_ati_deg'] = (f.pop('R_in_reference'), f.pop('xyz_in_reference_mm'),
                                                            f.pop('rpy_in_reference_deg'))
    import yaml
    path = os.path.join(tempfile.mkdtemp(), 'old.yaml')
    with open(path, 'w') as fh:
        yaml.safe_dump(d, fh)
    back = fc.Calibration.load(path)
    check('calibration files of the first version still load', np.allclose(back.frame['R_in_reference'],
                                                                         cal.frame['R_in_reference']))
    # with the reference's rated range given, samples above 95 % of it are not used
    lim = np.max(np.abs(sess.ref.w - np.median(sess.ref.w, 0)), 0) * 0.8
    cal2, rep2 = fc.fit([sess], log=quiet, folds=0, ref_limits=lim + np.abs(np.median(sess.ref.w, 0)))
    check('reference limits drop overloaded samples', cal2.info['samples'] < cal.info['samples']
          and any('dropped' in n for n in cal2.info['notes']), '%d -> %d samples' % (cal.info['samples'], cal2.info['samples']))


if __name__ == '__main__':
    test_transform_algebra()
    test_full_map_and_lag()
    test_frame_identification()
    test_mirrored_hex()
    test_quadratic_compliance()
    test_two_sessions_and_custom_frame()
    test_yaml_and_apply()
    test_bad_streams()
    test_file_compat_and_limits()
    print()
    print('FAILED: ' + ', '.join(FAIL) if FAIL else 'All HEX calibration checks passed.')
    sys.exit(1 if FAIL else 0)
