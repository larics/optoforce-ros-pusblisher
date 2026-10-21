#!/usr/bin/env python3
"""ROS1 node: calibrate the OnRobot/OptoForce HEX against a reference F/T sensor.

The HEX is mounted on top of a reference sensor (any sensor publishing geometry_msgs/WrenchStamped,
e.g. an ATI Gamma), so every load on the handle above passes through both. This node listens to
both, records loading sessions, runs the whole fit (stamp delay, drift, 6x6 calibration, mount
identification, cross-validation; docs/hex_calibration_math.md), writes the calibration and then
publishes the calibrated HEX. Driven from its own terminal:

  [s] start / stop a recording session      [f] fit with all sessions of this run
  [z] tare (hands off for 1 s)              [d] delete the last session
  [m] model linear <-> quadratic            [h] help on/off          [q] quit

or with services (std_srvs/Trigger): ~start ~stop ~fit ~zero ~discard

  roslaunch optoforce_sensor hex_calibration.launch   (no keys: roslaunch has no stdin; use services)
  or the tmux session: startup/hex_calibration/start.sh

Everything stays in this package:
  cfg/hex_calibration_params.yaml     settings (reference topic and rated range, targets, model)
  calibration/<date_time>/            s1.npz s2.npz ... hex_calibration.yaml hex_calibration_report.txt
  cfg/hex_calibration.yaml            the active calibration (hex_calibrated.launch loads it); a fit
                                      flagged unusable never replaces it

Parameters (see cfg/hex_calibration_params.yaml):
  ~reference_topic (required), ~reference_name (REF), ~reference_limits (rated range; [] = unknown),
  ~reference_invalid_log (text the reference driver logs for invalid samples; '' = none),
  ~hex_topic (/optoforce_node/OptoForceWrench), ~hex_limits (200,200,200,10,10,6.5 = HEX-E),
  ~targets (40,40,80,2,2,2), ~warn (0.9), ~model (linear), ~drift (linear), ~session_duration (0),
  ~zero_seconds (1.0), ~drift_tau (0), ~frame (native | top | reference), ~top_mm (74),
  ~output_topic (/optoforce_node/OptoForceWrench_calibrated),
  ~reference_output (/hex_calibration/reference_in_hex), ~calibration_file, ~runs_dir

Sensor health: a stream that keeps publishing the same values (a driver republishing its last
packet after the sensor stopped sending) is shown as FROZEN. Recording stops when a sensor freezes
or goes silent; the part before (minus 2 s, often already wrong) is kept if it is long enough.
"""
from __future__ import division, print_function

import collections
import datetime
import os
import re
import select
import shutil
import sys
import textwrap
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ft_crosscal as fc  # noqa: E402
from crosscal_ros import CalibratedPublisher, calibration_file, package_dir, vec  # noqa: E402

AXES, UNITS = fc.AXES, ('N', 'N', 'N', 'N*m', 'N*m', 'N*m')
STATE_TEXT = {'WAIT': 'waiting for both sensors', 'TARE': 'TARING (hands off)', 'IDLE': 'ready',
              'REC': 'RECORDING', 'FIT': 'FITTING', 'APPLY': 'CALIBRATED'}
LIVE_FREEZE_S = 0.5              # no new value for this long while messages keep coming: FROZEN
SILENT_S = 0.5                   # no message at all for this long: NO DATA


def floats(v, n, name):
    """Six numbers from a YAML list or a comma-separated string; None if empty."""
    if v is None or v == '' or v == []:
        return None
    vals = [float(x) for x in (v if isinstance(v, (list, tuple)) else str(v).split(','))]
    if len(vals) != n:
        raise ValueError('~%s needs %d numbers (Fx Fy Fz Tx Ty Tz), got %s' % (name, n, v))
    return np.array(vals)


class Buffer(object):
    """Thread-safe append-only sample store (t, 6 values) -> (N, 7) array."""

    def __init__(self):
        self.lock = threading.Lock()
        self.rows, self.chunks = [], []

    def add(self, t, w):
        with self.lock:
            self.rows.append((t, w[0], w[1], w[2], w[3], w[4], w[5]))

    def array(self):
        with self.lock:
            if self.rows:
                self.chunks.append(np.array(self.rows))
                self.rows = []
            return np.vstack(self.chunks) if self.chunks else np.zeros((0, 7))


class Acc(object):
    """Running sum for block means between ticks."""

    def __init__(self):
        self.s, self.n = np.zeros(6), 0

    def add(self, w):
        self.s += w
        self.n += 1

    def take(self):
        m = self.s / self.n if self.n else None
        self.s, self.n = np.zeros(6), 0
        return m


class Node(object):
    def __init__(self):
        import rospy
        from geometry_msgs.msg import WrenchStamped
        from std_srvs.srv import Trigger
        self.rospy = rospy
        P = lambda k, d: rospy.get_param('~' + k, d)
        self.ref_topic = str(P('reference_topic', ''))
        if not self.ref_topic:
            raise ValueError('set ~reference_topic (any geometry_msgs/WrenchStamped topic of the reference '
                             'sensor), e.g. in cfg/hex_calibration_params.yaml')
        self.ref_name = str(P('reference_name', 'REF'))
        self.ref_lim = floats(P('reference_limits', None), 6, 'reference_limits')
        self.invalid_log = str(P('reference_invalid_log', ''))
        self.hex_topic = str(P('hex_topic', '/optoforce_node/OptoForceWrench'))
        self.hex_lim = floats(P('hex_limits', list(fc.HEX_E_CAPACITY)), 6, 'hex_limits')
        self.targets = floats(P('targets', [40, 40, 80, 2, 2, 2]), 6, 'targets')
        self.warn = float(P('warn', 0.9))
        self.model, self.drift = str(P('model', 'linear')), str(P('drift', 'linear'))
        self.session_s = float(P('session_duration', 0.0))
        self.zero_s = float(P('zero_seconds', 1.0))
        self.drift_tau = float(P('drift_tau', 0.0))
        self.frame, self.top_mm = str(P('frame', 'native')), float(P('top_mm', 74.0))
        self.out_topic = str(P('output_topic', '/optoforce_node/OptoForceWrench_calibrated'))
        self.ref_out_topic = str(P('reference_output', '/hex_calibration/reference_in_hex'))
        self.pkg = package_dir()
        self.cal_file = os.path.expanduser(str(P('calibration_file', '') or calibration_file()))
        self.runs_dir = os.path.expanduser(str(P('runs_dir', '') or os.path.join(self.pkg, 'calibration')))
        self.run = os.path.join(self.runs_dir, datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S'))
        # the "orange line": warn x rated range of the reference, or the targets if it is unknown
        self.orange = self.warn * self.ref_lim if self.ref_lim is not None else self.targets

        self.lock = threading.Lock()
        self.state = 'WAIT'
        self.tty = sys.stdin.isatty() and sys.stdout.isatty()
        self.events = collections.deque(maxlen=5)
        self.help = False
        self.sessions = []                       # dicts: name, path, seconds, ref, hex (arrays), meta
        self.rec = None                          # current session buffers
        self.n_msg = [0, 0]                      # messages: reference, HEX
        self.rate = [0.0, 0.0]
        self.last_msg = [0.0, 0.0]
        self.prev_w = [None, None]
        self.last_new = [0.0, 0.0]               # wall time of the last value that changed
        self.last_new_stamp = [0.0, 0.0]         # and its stamp
        self.n_new = [0, 0]
        self.new_rate = [0.0, 0.0]
        self.invalid_rec = 0                     # reference "invalid" logs while recording
        self.invalid_total = 0
        self.acc = {k: Acc() for k in ('ref', 'hex', 'cal', 'ref_out')}
        self.tare_acc = None
        self.tare_ref = self.tare_hex = None
        self.now = {}
        self.peak_max, self.peak_min = np.zeros(6), np.zeros(6)
        self.over = 0                            # reference samples over 95 % of rated while recording
        self.load_max = (0.0, '')
        self.hist = collections.deque(maxlen=100)    # 5 s of (cal, ref, raw) at 20 Hz in APPLY
        self.pub = None
        self.cal = None
        self.cv = None
        self.excite = None
        self.fitted = 0
        self.report = ''
        self._prev_health = ['ok', 'ok']

        rospy.Subscriber(self.ref_topic, WrenchStamped, self.on_ref, queue_size=2000, tcp_nodelay=True)
        rospy.Subscriber(self.hex_topic, WrenchStamped, self.on_hex, queue_size=2000, tcp_nodelay=True)
        if self.invalid_log:
            from rosgraph_msgs.msg import Log
            rospy.Subscriber('/rosout_agg', Log, self.on_log, queue_size=100)
        for name, fn in (('start', self.start_session), ('stop', self.stop_session), ('fit', self.fit),
                         ('zero', self.zero), ('discard', self.discard)):
            rospy.Service('~' + name, Trigger, self._srv(fn))
        rospy.Timer(rospy.Duration(0.05), self.tick)
        rospy.Timer(rospy.Duration(1.0 if self.tty else 10.0), self.draw)
        if self.tty:
            threading.Thread(target=self.keyboard, daemon=True).start()
        self.event('reference %s: %s   HEX: %s' % (self.ref_name, self.ref_topic, self.hex_topic))
        if self.ref_lim is None:
            self.event('~reference_limits not set: bars show %% of ~targets, no overload check')

    # ------------------------------------------------------------------ helpers
    def short(self, path):
        """Paths inside the package as optoforce_sensor/..."""
        if path.startswith(self.pkg + os.sep):
            return 'optoforce_sensor/' + os.path.relpath(path, self.pkg)
        return path

    def event(self, text):
        self.events.append('%s  %s' % (time.strftime('%H:%M:%S'), text))
        if not self.tty:
            self.rospy.loginfo(text)

    def _srv(self, fn):
        from std_srvs.srv import TriggerResponse

        def handler(_):
            ok, msg = fn()
            return TriggerResponse(ok, msg)
        return handler

    # ------------------------------------------------------------------ data in
    def on_ref(self, msg):
        t = msg.header.stamp.to_sec() or self.rospy.get_time()
        w = vec(msg)
        with self.lock:
            self._seen(0, t, w)
            self.acc['ref'].add(w)
            if self.tare_acc is not None:
                self.tare_acc['ref'].add(w)
            if self.state == 'REC':
                self.rec['ref'].add(t, w)
                if self.ref_lim is not None and self.tare_ref is not None \
                        and np.any(np.abs(w - self.tare_ref) > 0.95 * self.ref_lim):
                    self.over += 1
            pub = self.pub
        if pub is not None:
            pub.on_ref(msg)
            r = pub.st['ref_out']
            if r is not None:
                with self.lock:
                    self.acc['ref_out'].add(r)

    def on_hex(self, msg):
        t = msg.header.stamp.to_sec() or self.rospy.get_time()
        w = vec(msg)
        with self.lock:
            self._seen(1, t, w)
            self.acc['hex'].add(w)
            if self.tare_acc is not None:
                self.tare_acc['hex'].add(w)
            if self.state == 'REC':
                self.rec['hex'].add(t, w)
            pub = self.pub
        if pub is not None:
            pub.on_hex(msg)
            o = pub.st['out']
            if o is not None:
                with self.lock:
                    self.acc['cal'].add(o)

    def _seen(self, i, t, w):         # call with lock held
        now = time.time()
        self.n_msg[i] += 1
        self.last_msg[i] = now
        if self.prev_w[i] is None or np.any(w != self.prev_w[i]):
            self.last_new[i] = now
            self.last_new_stamp[i] = t
            self.n_new[i] += 1
        self.prev_w[i] = w

    def on_log(self, msg):
        if self.invalid_log not in msg.msg:
            return
        t = msg.header.stamp.to_sec() or self.rospy.get_time()
        with self.lock:
            self.invalid_total += 1
            first = False
            if self.state == 'REC':
                self.rec['invalid'].append(fc.invalid_window(t))
                self.invalid_rec += 1
                first = self.invalid_rec == 1
        if first:
            self.event('%s reports invalid readings ("%s"): too much (combined) load, ease off. Those '
                       'seconds are left out' % (self.ref_name, self.invalid_log))

    def health(self, i, now=None):
        """'ok', 'NO DATA' or 'FROZEN' and for how long (s)."""
        now = now or time.time()
        if self.last_msg[i] == 0 or now - self.last_msg[i] > SILENT_S:
            return 'NO DATA', (now - self.last_msg[i]) if self.last_msg[i] else 0.0
        if now - self.last_new[i] > LIVE_FREEZE_S:
            return 'FROZEN', now - self.last_new[i]
        return 'ok', 0.0

    def who(self, i):
        return (self.ref_name, 'HEX')[i]

    # ------------------------------------------------------------------ 20 Hz
    def tick(self, _):
        broken = None
        with self.lock:
            means = {k: a.take() for k, a in self.acc.items()}
            state = self.state
            healthy = [self.health(i)[0] == 'ok' for i in (0, 1)]
            if state == 'REC' and not all(healthy):
                i = healthy.index(False)
                broken = self.who(i), self.health(i)[0], self.last_new_stamp[i]
            if state == 'WAIT' and min(self.n_msg) > 100 and all(healthy):
                self._begin_tare()
            elif state == 'TARE' and not all(healthy):
                self.tare_acc = None
                self.state = 'WAIT' if self.state_after_tare == 'IDLE' else self.state_after_tare
            elif state == 'TARE' and time.time() >= self.tare_until:
                a, h = self.tare_acc['ref'].take(), self.tare_acc['hex'].take()
                self.tare_acc = None
                if a is not None and h is not None:
                    self.tare_ref, self.tare_hex = a, h
                    self.state = self.state_after_tare
                    self.event('tared. Press [s] to record a session (hands off for its first 3 s)'
                               if self.state == 'IDLE' else 'tared')
                else:
                    self._begin_tare()
            if means['ref'] is not None and self.tare_ref is not None:
                a = means['ref'] - self.tare_ref
                self.now['ref'] = a
                if self.state == 'REC':
                    self.peak_max = np.maximum(self.peak_max, a)
                    self.peak_min = np.minimum(self.peak_min, a)
                    if self.ref_lim is not None:
                        load = np.abs(a) / self.ref_lim
                        i = int(np.argmax(load))
                        if load[i] > self.load_max[0]:
                            self.load_max = (float(load[i]), AXES[i])
            if means['hex'] is not None and self.tare_hex is not None:
                self.now['hex'] = means['hex'] - self.tare_hex
            if self.state == 'APPLY' and means['cal'] is not None and means['ref_out'] is not None:
                self.hist.append((means['cal'], means['ref_out'], self.now.get('hex', np.zeros(6)) * self.signs))
            rec = self.rec
        for i in (0, 1):
            h = self.health(i)[0]
            if h != self._prev_health[i] and state != 'WAIT':
                if h == 'ok':
                    self.event('%s data is back. If its zero moved, tare again with [z] (hands off)' % self.who(i))
                elif state != 'REC':
                    self.event('%s %s: the sensor stopped sending. Check its cable/USB and driver' % (self.who(i), h))
            self._prev_health[i] = h
        if broken is not None:
            who, what, stamp = broken
            self.stop_session(cut=stamp - fc.GUARD_S,
                              why='%s %s: the sensor stopped sending (check its cable/USB/driver)' % (who, what))
        elif state == 'REC' and self.session_s > 0 and time.time() - rec['t0'] >= self.session_s:
            self.stop_session()

    def _begin_tare(self):            # call with lock held
        self.state_after_tare = 'APPLY' if self.state == 'APPLY' else 'IDLE'
        self.state = 'TARE'
        self.tare_acc = {'ref': Acc(), 'hex': Acc()}
        self.tare_until = time.time() + self.zero_s

    # ------------------------------------------------------------------ actions
    def start_session(self):
        with self.lock:
            if self.state == 'REC':
                return False, 'already recording'
            if self.state not in ('IDLE', 'APPLY'):
                return False, 'not ready (%s)' % STATE_TEXT[self.state]
            for i in (0, 1):
                h, age = self.health(i)
                if h != 'ok':
                    return False, ('%s %s for %.0f s: not recording. Check the sensor, its cable/USB and driver'
                                   % (self.who(i), h, age))
            name = 's%d' % (len(self.sessions) + 1)
            while os.path.exists(os.path.join(self.run, name + '.npz')):
                name += 'b'
            self.rec = {'name': name, 't0': time.time(), 'ref': Buffer(), 'hex': Buffer(), 'prev': self.state,
                        'invalid': []}
            self.state = 'REC'
            self.invalid_rec = 0
        self.event('recording %s: hands off for 3 s, then load the handle slowly in every direction' % name)
        return True, 'recording ' + name

    def stop_session(self, cut=None, why=None):
        """cut: drop samples stamped after this (a sensor broke); why: tell the user."""
        with self.lock:
            if self.state != 'REC':
                return False, 'not recording'
            rec, self.rec = self.rec, None
            self.state = rec['prev']
        a, h = rec['ref'].array(), rec['hex'].array()
        secs = time.time() - rec['t0']
        if cut is not None:
            a, h = a[a[:, 0] < cut], h[h[:, 0] < cut]
            secs = (min(a[-1, 0], h[-1, 0]) - max(a[0, 0], h[0, 0])) if len(a) and len(h) else 0.0
            el = time.time() - rec['t0']
            self.event('%s at %02d:%02d of %s: recording stopped' % (why, el // 60, el % 60, rec['name']))
        if secs < 10 or len(a) < 1000 or len(h) < 1000:
            self.event('%s too short (%.0f s usable): dropped' % (rec['name'], secs))
            return False, 'session too short, dropped'
        try:
            os.makedirs(self.run, exist_ok=True)
        except OSError as e:
            self.event('cannot write %s: %s' % (self.run, e))
            return False, str(e)
        path = os.path.join(self.run, rec['name'] + '.npz')
        meta = {'ref_topic': self.ref_topic, 'ref_name': self.ref_name, 'dut_topic': self.hex_topic,
                'recorded': datetime.datetime.now().isoformat(timespec='seconds'),
                'ref_invalid': [tuple(x) for x in rec['invalid']]}
        sr, sh = fc.Series(a[:, 0], a[:, 1:]), fc.Series(h[:, 0], h[:, 1:])
        fc.save_session_npz(path, sr, sh, meta)
        self.sessions.append({'name': rec['name'], 'path': path, 'seconds': secs, 'ref': a, 'hex': h, 'meta': meta})
        self._update_excitation()
        q = fc.session_quality(fc.Session(sr, sh, rec['name'], meta))
        self.event('saved %s (%.0f s%s). [s] another session (2+ recommended), [f] fit'
                   % (self.short(path), secs, ('; ' + '; '.join(q['text'])) if q['text'] else ''))
        return True, 'saved ' + path

    def _update_excitation(self):
        hx = np.vstack([s['hex'][::10, 1:] - np.median(s['hex'][:500, 1:], 0) for s in self.sessions])
        self.excite = fc.coverage(hx, self.targets)

    def discard(self):
        with self.lock:
            if self.state in ('REC', 'FIT') or not self.sessions:
                return False, 'nothing to delete now'
            s = self.sessions.pop()
        try:
            os.remove(s['path'])
        except OSError:
            pass
        if self.sessions:
            self._update_excitation()
        else:
            self.excite = None
        self.event('deleted %s' % s['name'])
        return True, 'deleted ' + s['name']

    def zero(self):
        with self.lock:
            if self.state in ('REC', 'FIT', 'TARE', 'WAIT'):
                return False, 'not now (%s)' % STATE_TEXT[self.state]
            self._begin_tare()
            pub = self.pub
        if pub is not None:
            pub.zero()
        self.event('taring: hands off for %.1f s' % self.zero_s)
        return True, 'taring'

    def fit(self):
        if self.state == 'REC':
            self.stop_session()
        with self.lock:
            if self.state == 'FIT':
                return False, 'already fitting'
            if not self.sessions:
                return False, 'record a session first'
            self.state_before_fit = self.state
            self.state = 'FIT'
        threading.Thread(target=self._fit_worker, daemon=True).start()
        return True, 'fitting'

    def _fit_worker(self):
        t0 = time.time()
        try:
            self.event('fitting %d session(s), %s model ... (about %d s)'
                       % (len(self.sessions), self.model, 8 * len(self.sessions)))
            sess = [fc.Session(fc.Series(s['ref'][:, 0], s['ref'][:, 1:]), fc.Series(s['hex'][:, 0], s['hex'][:, 1:]),
                               name=s['name'], meta=s.get('meta')) for s in self.sessions]
            if self.frame == 'top':
                spec = {'mode': 'native_at', 'xyz_mm': [0.0, 0.0, self.top_mm], 'name': 'hex_top'}
            elif self.frame == 'reference':
                spec = {'mode': 'reference', 'name': 'reference_sensor'}
            else:
                spec = {'mode': 'native', 'name': 'hex_calibrated'}
            cal, rep = fc.fit(sess, model=self.model, drift=self.drift, ref_limits=self.ref_lim,
                              dut_limits=self.hex_lim, frame_spec=spec, targets=list(self.targets),
                              log=lambda *a: None)
            cal.info['reference'] = {'topic': self.ref_topic, 'name': self.ref_name,
                                     'limits': None if self.ref_lim is None else self.ref_lim.tolist()}
            text = fc.format_report(cal, rep)
            yml = os.path.join(self.run, 'hex_calibration.yaml')
            cal.save(yml)
            with open(os.path.join(self.run, 'hex_calibration_report.txt'), 'w') as f:
                f.write(text + '\n')
            active = False
            if not cal.info.get('poor_axes'):          # never let a broken fit replace a good one
                try:
                    os.makedirs(os.path.dirname(self.cal_file), exist_ok=True)
                    shutil.copyfile(yml, self.cal_file)
                    active = True
                except (IOError, OSError) as e:
                    self.event('could not write %s: %s' % (self.cal_file, e))
            Xn = fc.wrench_transform_inv(rep['identified']['R'], rep['identified']['p'])
            self.signs = np.sign(np.diag(Xn.dot(rep['M_ref_from_raw'])))
            self.cal, self.report = cal, text
            self.fitted = len(sess)
            self.cv = np.array([cal.info['error_rms_crossval'][a] for a in AXES])
            self.pub = CalibratedPublisher(cal, self.out_topic, self.ref_out_topic, None, self.zero_s,
                                           self.drift_tau, with_reference=True)
            with self.lock:
                self.hist.clear()
                self.state = 'APPLY'
            if active:
                self.event('calibration active: %s (this run: %s), %.0f s'
                           % (self.short(self.cal_file), self.short(yml), time.time() - t0))
            else:
                self.event('NOT USABLE: the HEX does not follow %s on %s. Kept only in %s; the active '
                           'calibration was not changed' % (self.ref_name, ', '.join(cal.info['poor_axes']), self.short(yml)))
            self.event('publishing %s: hands off for %.0f s (tare). Compare in the verify window.'
                       % (self.out_topic, self.zero_s))
            if not self.tty:
                print('\n' + text + '\n')
        except Exception as e:                                            # report, stay usable
            with self.lock:
                self.state = self.state_before_fit
            self.event('fit failed: %s' % e)

    # ------------------------------------------------------------------ keyboard
    def keyboard(self):
        import termios
        import tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        restore = lambda: termios.tcsetattr(fd, termios.TCSADRAIN, old)
        self.rospy.on_shutdown(restore)
        try:
            tty.setcbreak(fd)
            while not self.rospy.is_shutdown():
                r, _, _ = select.select([sys.stdin], [], [], 0.2)
                if not r:
                    continue
                k = sys.stdin.read(1).lower()
                ok, m = True, ''
                if k in ('s', ' '):
                    ok, m = self.stop_session() if self.state == 'REC' else self.start_session()
                elif k == 'f':
                    ok, m = self.fit()
                elif k == 'z':
                    ok, m = self.zero()
                elif k == 'd':
                    ok, m = self.discard()
                elif k == 'm':
                    self.model = 'quadratic' if self.model == 'linear' else 'linear'
                    self.event('model for the next fit: %s' % self.model)
                elif k == 'h':
                    self.help = not self.help
                elif k == 'q':
                    self.event('quitting')
                    self.rospy.signal_shutdown('quit key')
                if not ok:
                    self.event(m)
                self.draw(None)
        finally:
            restore()

    # ------------------------------------------------------------------ screen
    def draw(self, _):
        tty_ = self.tty
        B, D, R, G, Y, C, N = ('\033[1m', '\033[2m', '\033[31m', '\033[32m', '\033[33m', '\033[36m', '\033[0m') if tty_ \
            else ('',) * 7
        now = time.time()
        with self.lock:
            dt = max(now - getattr(self, '_last_draw', now - 1.0), 1e-3)
            self._last_draw = now
            prev_n, prev_new = getattr(self, '_prev_counts', ([0, 0], [0, 0]))
            self.rate = [(self.n_msg[k] - prev_n[k]) / dt for k in (0, 1)]
            self.new_rate = [(self.n_new[k] - prev_new[k]) / dt for k in (0, 1)]
            self._prev_counts = (list(self.n_msg), list(self.n_new))
            state, rec, sessions = self.state, self.rec, list(self.sessions)
            nowv = dict(self.now)
            pmax, pmin, over, lmax = self.peak_max.copy(), self.peak_min.copy(), self.over, self.load_max
            inv_total = self.invalid_total
            hist = list(self.hist)
        rn = self.ref_name
        L = []
        st = STATE_TEXT[state]
        if state == 'REC':
            el = now - rec['t0']
            st = '%s %s  %02d:%02d%s' % (st, rec['name'], el // 60, el % 60,
                                         ' / %02d:%02d' % (self.session_s // 60, self.session_s % 60) if self.session_s else '')
        elif state == 'APPLY':
            st = '%s: publishing %s' % (st, self.out_topic)
        col = {'REC': R, 'FIT': Y, 'APPLY': G, 'TARE': Y}.get(state, C)
        L.append('%sHEX calibration against %s%s    state: %s%s%s%s    model: %s'
                 % (B, rn, N, col, B, st, N, self.model))
        L.append('%srun %s%s' % (D, self.short(self.run), N))

        def stream(i):
            h, age = self.health(i, now)
            name = self.who(i)
            if h == 'NO DATA':
                return '%s%s%s NO DATA%s%s' % (R, B, name, ' %.0f s' % age if self.last_msg[i] else '', N)
            if h == 'FROZEN':
                return '%s%s%s FROZEN %.0f s%s' % (R, B, name, age, N)
            new = '' if self.new_rate[i] >= 0.9 * self.rate[i] else ' (%.0f new)' % self.new_rate[i]
            return '%s %4.0f Hz%s' % (name, self.rate[i], new)
        L.append('streams: %s   %s      tare: %s' % (stream(0), stream(1), 'ok' if self.tare_ref is not None else 'not yet'))
        for i in (0, 1):
            h = self.health(i, now)[0]
            if h == 'FROZEN':
                L.append('%s  %s repeats the same values: the sensor stopped sending. Check cable, connector, USB;'
                         ' restart its driver%s' % (R, self.who(i), N))
            elif h == 'NO DATA' and self.last_msg[i]:
                L.append('%s  no %s messages: is its driver running? (%s)%s'
                         % (R, self.who(i), self.ref_topic if i == 0 else self.hex_topic, N))
        ses = '   '.join('%s %.0f s' % (s['name'], s['seconds']) for s in sessions) or 'none yet'
        L.append('sessions: %s%s' % (ses, '   %s(recording)%s' % (R, N) if state == 'REC' else ''))
        L.append('')
        if self.ref_lim is not None:
            L.append('%sload you apply%s (%s, tared) and how far you got, in %% of the %sorange line%s (%d %% rated):'
                     % (B, N, rn, Y, N, round(100 * self.warn)))
        else:
            L.append('%sload you apply%s (%s, tared) and how far you got, in %% of ~targets:' % (B, N, rn))
        L.append('  axis        now       - reached                + reached')
        a = nowv.get('ref')
        for i, ax in enumerate(AXES):
            fn, fp = max(0.0, -pmin[i]) / self.orange[i], max(0.0, pmax[i]) / self.orange[i]
            v = '%+8.2f %-3s' % (a[i], UNITS[i]) if a is not None else '      -    '
            L.append('  %-3s  %s  %s %4.0f %%   %s %4.0f %%'
                     % (ax, v, self._bar(fn, G, Y, N, rev=True), 100 * fn, self._bar(fp, G, Y, N), 100 * fp))
        if a is not None and self.ref_lim is not None:
            load = np.abs(a) / self.ref_lim
            i = int(np.argmax(load))
            lc = R if load[i] > 0.9 else (Y if load[i] > 0.7 else '')
            L.append('%s load now: %s%3.0f %% (%s)%s   highest while recording: %.0f %% %s   over 95 %%: %s%d%s%s'
                     % (rn, lc, 100 * load[i], AXES[i], N, 100 * lmax[0], lmax[1], R if over else '', over, N,
                        ('   invalid (driver log): %s%d%s' % (R if inv_total else '', inv_total, N))
                        if self.invalid_log else ''))
        if self.excite is not None:
            c = self.excite['cond']
            ready = c < 10 and len(sessions) >= 2 and state in ('IDLE', 'APPLY') and len(sessions) > self.fitted
            L.append('HEX excitation: condition %s%.1f%s (aim < 10)   %s'
                     % (G if c < 10 else Y, c, N, ('enough variety' + ('  ->  ready, press [f] to fit' if ready else ''))
                        if c < 10 else 'next: ' + fc.excitation_hint(self.excite['weakest'])))
        if self.cal is not None and (state == 'APPLY' or hist):
            L.append('')
            info = self.cal.info
            if info.get('poor_axes'):
                L.append('%s%sNOT USABLE: the HEX does not follow %s on %s (R^2 %s). Check the HEX and record again.%s'
                         % (R, B, rn, ', '.join(info['poor_axes']),
                            ', '.join('%.2f' % info['fit_r2'][x] for x in info['poor_axes']), N))
            m = info['identified_mount']
            L.append('%sfit%s: HEX stamps %.1f ms late | HEX mount: yaw %+.1f, tilt %.1f/%.1f deg, z %+.1f mm%s'
                     % (B, N, float(np.mean(info['hex_delay_ms'])), m['rpy_deg'][2], m['rpy_deg'][0], m['rpy_deg'][1],
                        m['xyz_mm'][2], ('; %s mirrored' % ','.join(m['mirrored_axes'])) if m['mirrored_axes'] else ''))
            L.append('     report: %s' % self.short(os.path.join(self.run, 'hex_calibration_report.txt')))
            L.append('%scalibrated HEX vs %s%s (tared; err = RMS over the last 5 s; frame %s):'
                     % (B, rn, N, self.cal.frame['name']))
            L.append('  axis  %9s  HEX cal now    err RMS  factory err   fit cross-val' % (rn + ' now'))
            if hist:
                H = np.array(hist)
                e_cal = np.sqrt(np.mean((H[:, 0] - H[:, 1]) ** 2, 0))
                e_raw = np.sqrt(np.mean((H[:, 2] - H[:, 1]) ** 2, 0))
                for i, ax in enumerate(AXES):
                    fmt = '%9.3f' if i < 3 else '%9.4f'
                    L.append(('  %-3s  ' + fmt + '    ' + fmt + '  ' + fmt + '    ' + fmt + '   ' + fmt + '  %s')
                             % (ax, H[-1, 1, i], H[-1, 0, i], e_cal[i], e_raw[i], self.cv[i], UNITS[i]))
            else:
                L.append('  (waiting for the tare)')
        L.append('')
        L.append('%s[s]%s session start/stop  %s[f]%s fit  %s[z]%s tare  %s[d]%s delete last  %s[m]%s model  %s[h]%s help  %s[q]%s quit'
                 % ((B, N) * 7))
        if self.help:
            L += ['',
                  'How: tare with hands off, then record 2+ sessions of 3-5 min each. Load the handle slowly',
                  '(0.2-1 s per push): sideways in all directions low and high on the handle, press down and',
                  'pull up, twist both ways, rock without pushing, hands off now and then. Bring every axis',
                  'towards 100 % (the orange line in the plots), never past the red line (rated range).',
                  'Then [f]: the fit writes cfg/hex_calibration.yaml and this node publishes the calibrated HEX.',
                  'Services: rosservice call /%s/{start,stop,fit,zero,discard}' % self.rospy.get_name().strip('/')]
        L.append('')
        cols, rows = shutil.get_terminal_size((120, 50)) if tty_ else (120, 50)
        for e in self.events:                       # events wrap (they explain problems), the rest is clipped
            for j, part in enumerate(textwrap.wrap(e, max(cols - 4, 20))):
                L.append('%s%s %s%s' % (D, '>' if j == 0 else ' ', part, N))
        if tty_:
            L = [self._clip(l, cols) for l in L]
            while len(L) > rows - 1 and any(x.startswith(D + '>') for x in L):     # drop old events first
                k = next(j for j, x in enumerate(L) if x.startswith(D + '>'))
                L.pop(k)
                while k < len(L) and L[k].startswith(D + ' '):
                    L.pop(k)
            L = L[:max(rows - 1, 1)]
            sys.stdout.write('\033[H\033[2J' + '\n'.join(L) + '\n')
        else:
            sys.stdout.write('\n'.join(L) + '\n\n')
        sys.stdout.flush()

    @staticmethod
    def _clip(line, cols):
        """Cut a line to the visible width (ANSI colour codes don't count)."""
        if len(re.sub(r'\033\[[0-9;]*m', '', line)) <= cols:
            return line
        out, vis, i = '', 0, 0
        while i < len(line) and vis < cols - 1:
            m = re.match(r'\033\[[0-9;]*m', line[i:])
            if m:
                out += m.group(0)
                i += len(m.group(0))
            else:
                out += line[i]
                vis += 1
                i += 1
        return out + '\033[0m'

    @staticmethod
    def _bar(frac, G, Y, N, width=14, rev=False):
        n = int(round(min(max(frac, 0.0), 1.0) * width))
        col = G if frac >= 1.0 else Y
        s = col + '#' * n + N + '.' * (width - n)
        return ('.' * (width - n) + col + '#' * n + N) if rev else s


def main():
    import rospy
    rospy.init_node('hex_calibration')
    try:
        node = Node()
    except ValueError as e:
        rospy.logfatal(str(e))
        return 1
    rospy.spin()
    if node.state == 'REC':
        node.stop_session()
    if os.path.isdir(node.run):
        print('\nthis run: %s' % node.run)
        for f in sorted(os.listdir(node.run)):
            print('  ' + f)
    return 0


if __name__ == '__main__':
    sys.exit(main())
