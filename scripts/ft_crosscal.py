"""ft_crosscal: calibrate a 6-axis F/T sensor against a reference F/T sensor mounted in series.

Use case: the OnRobot/OptoForce HEX (the "DUT", device under test) bolted on top of a reference
F/T sensor - any sensor that publishes a wrench (we used an ATI Gamma). Every load applied above
the HEX passes through both, so after the constant weight of the parts in between is tared away,
the reference reading is the HEX's true wrench moved to another point (see
docs/hex_calibration_math.md):

    w_ref = X(R, p) . w_hex_true + const          X = [[R, 0], [S(p) R, R]]

This module only needs numpy. ROS is used by the nodes, not here.

Conventions: wrench vectors are [Fx Fy Fz Tx Ty Tz] in N and N*m; positions in m. A frame's
pose (R, p) is "in the reference frame": columns of R are the frame's axes, p is its origin.
"""
from __future__ import division, print_function

import datetime
import math

import numpy as np

AXES = ('Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz')
UNITS = ('N', 'N', 'N', 'N*m', 'N*m', 'N*m')
HEX_E_CAPACITY = (200.0, 200.0, 200.0, 10.0, 10.0, 6.5)      # OnRobot HEX-E nominal capacity
FORMAT = 'ft_crosscal/1'
# A working F/T sensor never repeats all six values exactly for this long (noise alone changes
# them); a driver that republishes its last packet after the sensor stopped sending does.
FREEZE_S = 0.25
# Data just before an outage is often already wrong (a loose connector gives garbage first).
GUARD_S = 2.0
MIN_SEGMENT_S = 10.0


# --------------------------------------------------------------------------------------------
# spatial algebra
# --------------------------------------------------------------------------------------------
def skew(p):
    """S(p) with S(p) v = p x v."""
    x, y, z = p
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def vee(S):
    """Inverse of skew() for the skew-symmetric part of S."""
    A = 0.5 * (S - S.T)
    return np.array([A[2, 1], A[0, 2], A[1, 0]])


def wrench_transform(R, p):
    """6x6 X mapping a wrench given in frame B (about B's origin) to frame A (about A's origin),
    where B's pose in A is (R, p).  f_A = R f_B ;  tau_A = R tau_B + p x (R f_B)."""
    R = np.asarray(R, float)
    X = np.zeros((6, 6))
    X[:3, :3] = R
    X[3:, 3:] = R
    X[3:, :3] = skew(p).dot(R)
    return X


def wrench_transform_inv(R, p):
    """Inverse of wrench_transform(R, p): from A to B."""
    R = np.asarray(R, float)
    Xi = np.zeros((6, 6))
    Xi[:3, :3] = R.T
    Xi[3:, 3:] = R.T
    Xi[3:, :3] = -R.T.dot(skew(p))
    return Xi


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def rpy_to_matrix(roll, pitch, yaw):
    """R = Rz(yaw) Ry(pitch) Rx(roll), radians (ROS / URDF convention)."""
    return rot_z(yaw).dot(rot_y(pitch)).dot(rot_x(roll))


def matrix_to_rpy(R):
    """Inverse of rpy_to_matrix, radians."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def rotation_angle(R):
    """Angle of a rotation matrix in radians."""
    c = (np.trace(R) - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, c)))


def polar_orthogonal(M):
    """Closest orthogonal matrix to M (polar decomposition, via SVD); det may be -1."""
    U, _, Vt = np.linalg.svd(M)
    return U.dot(Vt)


# --------------------------------------------------------------------------------------------
# data
# --------------------------------------------------------------------------------------------
class Series(object):
    """Time-stamped wrench samples: t (N,) seconds, w (N, 6)."""

    def __init__(self, t, w):
        t = np.asarray(t, float).ravel()
        w = np.asarray(w, float).reshape(-1, 6)
        order = np.argsort(t, kind='mergesort')
        t, w = t[order], w[order]
        keep = np.ones(len(t), bool)
        keep[1:] = np.diff(t) > 0                      # drop duplicate stamps
        self.t, self.w = t[keep], w[keep]

    def __len__(self):
        return len(self.t)

    def rate(self):
        return (len(self.t) - 1) / (self.t[-1] - self.t[0]) if len(self.t) > 1 else 0.0

    def new_mask(self):
        """True for samples whose values differ from the previous message."""
        new = np.ones(len(self.t), bool)
        new[1:] = np.any(self.w[1:] != self.w[:-1], axis=1)
        return new

    def fresh(self):
        """Only new values: drops the copies a driver publishes when no new packet arrived (they
        carry later stamps than the real sample, and during an outage they hide it)."""
        new = self.new_mask()
        if new.all():
            return self
        s = Series.__new__(Series)
        s.t, s.w = self.t[new], self.w[new]
        return s


class Session(object):
    def __init__(self, ref, dut, name='session', meta=None):
        self.ref, self.dut, self.name, self.meta = ref, dut, name, dict(meta or {})


def save_session_npz(path, ref, dut, meta=None):
    meta = dict(meta or {})
    np.savez_compressed(path, ref_t=ref.t, ref_w=ref.w, dut_t=dut.t, dut_w=dut.w,
                        meta=np.array(repr(meta)))


def load_session(path, ref_topic=None, dut_topic=None, invalid_log=''):
    """Load a recording: .npz written by the calibration node, or a ROS1 .bag (then the topics are
    needed; invalid_log: text the reference driver logs for invalid samples, '' = none)."""
    if path.endswith('.bag'):
        if not ref_topic:
            raise ValueError('%s: which topic is the reference sensor? (--reference-topic)' % path)
        return _load_bag(path, ref_topic, dut_topic or '/optoforce_node/OptoForceWrench', invalid_log)
    d = np.load(path, allow_pickle=False)
    meta = {}
    if 'meta' in d:
        try:
            import ast
            meta = ast.literal_eval(str(d['meta']))
        except (ValueError, SyntaxError):
            meta = {}
    return Session(Series(d['ref_t'], d['ref_w']), Series(d['dut_t'], d['dut_w']), name=path, meta=meta)


def invalid_window(t_log):
    """Samples to leave out around one log message of the reference driver saying its readings are
    invalid (e.g. the ATI's 'gauge saturated'; such logs are usually throttled to once a second, so
    this covers every bad sample up to 1 s after it)."""
    return (t_log - 0.1, t_log + 1.1)


def _load_bag(path, ref_topic, dut_topic, invalid_log=''):
    import rosbag                                         # ROS1 only
    data = {ref_topic: ([], []), dut_topic: ([], [])}
    sat = set()
    with rosbag.Bag(path) as bag:
        for topic, msg, t_bag in bag.read_messages(topics=[ref_topic, dut_topic, '/rosout', '/rosout_agg']):
            if topic in ('/rosout', '/rosout_agg'):
                if invalid_log and invalid_log in msg.msg:
                    sat.add(round((msg.header.stamp.to_sec() or t_bag.to_sec()), 3))
                continue
            st = msg.header.stamp.to_sec() or t_bag.to_sec()
            f, m = msg.wrench.force, msg.wrench.torque
            data[topic][0].append(st)
            data[topic][1].append((f.x, f.y, f.z, m.x, m.y, m.z))
    for topic in (ref_topic, dut_topic):
        if not data[topic][0]:
            raise ValueError('%s: no messages on %s' % (path, topic))
    return Session(Series(*data[ref_topic]), Series(*data[dut_topic]), name=path,
                   meta={'ref_topic': ref_topic, 'dut_topic': dut_topic,
                         'ref_invalid': [invalid_window(t) for t in sorted(sat)]})


# --------------------------------------------------------------------------------------------
# data quality: frozen streams, outages, invalid reference samples
# --------------------------------------------------------------------------------------------
def stream_quality(series, freeze_s=FREEZE_S):
    """How healthy one recorded stream is.

    frozen: spans where messages kept coming but all six values stayed exactly the same for more
    than freeze_s (the driver republishes an old packet: the sensor stopped sending).
    gaps: spans longer than freeze_s without any message."""
    t, n = series.t, len(series.t)
    dur = float(t[-1] - t[0]) if n > 1 else 0.0
    new = series.new_mask()
    idx = np.flatnonzero(new)
    last = np.r_[idx[1:] - 1, n - 1]                   # last copy of each new value
    frozen = [(float(t[i]), float(t[j])) for i, j in zip(idx, last) if t[j] - t[i] > freeze_s]
    dt = np.diff(t)
    g = np.flatnonzero(dt > freeze_s)
    gaps = [(float(t[i]), float(t[i + 1])) for i in g]
    return {'n': n, 'duration': dur, 'msg_rate': (n - 1) / dur if dur > 0 else 0.0,
            'new_rate': (new.sum() - 1) / dur if dur > 0 else 0.0,
            'repeat_frac': float(1.0 - new.mean()) if n else 0.0,
            'frozen': frozen, 'frozen_s': float(sum(b - a for a, b in frozen)),
            'gaps': gaps, 'gap_s': float(sum(b - a for a, b in gaps)),
            'max_gap': float(dt.max()) if n > 1 else 0.0}


def _merge(spans):
    out = []
    for a, b in sorted(spans):
        if out and a <= out[-1][1]:
            out[-1][1] = max(out[-1][1], b)
        else:
            out.append([a, b])
    return [tuple(s) for s in out]


def _in_spans(t, spans):
    bad = np.zeros(len(t), bool)
    for a, b in spans:
        bad |= (t >= a) & (t <= b)
    return bad


def session_quality(session, freeze_s=FREEZE_S, guard_s=GUARD_S):
    """Outages of either sensor (frozen or silent) and the spans to leave out of the fit.

    Returns dict(ref, dut: stream_quality; outages: merged spans where a sensor delivered nothing
    new; exclude: outages + guard_s before each + invalid reference spans (e.g. gauge saturation);
    usable_s: overlap time that is left; text: one-line problems, empty if all is well)."""
    q = {'ref': stream_quality(session.ref, freeze_s), 'dut': stream_quality(session.dut, freeze_s)}
    t0 = max(session.ref.t[0], session.dut.t[0])
    t1 = min(session.ref.t[-1], session.dut.t[-1])
    outages = []
    for k in ('ref', 'dut'):
        outages += q[k]['frozen'] + q[k]['gaps']
    # one stream ending long before the other is an outage too (e.g. the HEX node died)
    for s in (session.ref, session.dut):
        end = max(session.ref.t[-1], session.dut.t[-1])
        if end - s.t[-1] > freeze_s:
            outages.append((float(s.t[-1]), float(end)))
    outages = _merge(outages)
    invalid = _merge([tuple(s) for s in session.meta.get('ref_invalid', [])])
    exclude = _merge([(a - guard_s, b) for a, b in outages] + invalid)
    covered = sum(max(0.0, min(b, t1) - max(a, t0)) for a, b in exclude)
    q.update(outages=outages, invalid=invalid, exclude=exclude,
             overlap_s=float(max(t1 - t0, 0.0)), usable_s=float(max(t1 - t0 - covered, 0.0)))
    text = []
    for k, who in (('dut', 'HEX'), ('ref', 'reference')):
        s = q[k]
        if s['frozen']:
            text.append('%s frozen (same values repeated, the sensor stopped sending) for %.0f s of %.0f s, '
                        'first at %.1f s' % (who, s['frozen_s'], s['duration'], s['frozen'][0][0] - t0))
        if s['gaps']:
            text.append('%s silent for %.1f s in %d gap(s), longest %.1f s'
                        % (who, s['gap_s'], len(s['gaps']), s['max_gap']))
    if invalid:
        text.append('reference reported invalid readings %d time(s): those seconds are left out' % len(invalid))
    q['text'] = text
    return q


# --------------------------------------------------------------------------------------------
# signal preparation: common grid, identical zero-phase low-pass on both sensors
# --------------------------------------------------------------------------------------------
def _fir_lowpass(cutoff, fs, width_s=None):
    """Symmetric (zero-phase) windowed-sinc low-pass, unity DC gain."""
    width_s = width_s or 2.0 / cutoff
    n = int(round(width_s * fs)) | 1
    k = np.arange(n) - (n - 1) / 2.0
    h = np.sinc(2.0 * cutoff / fs * k) * np.blackman(n)
    return h / h.sum()


def _valid_near(t_samples, t_grid, max_gap):
    """True where the nearest sample of the original series is within max_gap/2 on both sides."""
    idx = np.searchsorted(t_samples, t_grid)
    idx = np.clip(idx, 1, len(t_samples) - 1)
    gap = t_samples[idx] - t_samples[idx - 1]
    inside = (t_grid >= t_samples[0]) & (t_grid <= t_samples[-1])
    return inside & (gap <= max_gap)


def prepare(session, delay=0.0, rate=100.0, cutoff=5.0, fine_rate=1000.0, max_gap=0.02, exclude=()):
    """Resample both sensors on one grid with the same filter.

    delay: how late the DUT stamps are relative to the reference (s); it is subtracted from them.
    exclude: (t0, t1) spans to leave out (session_quality()['exclude']).
    Returns t (n,), ref (n, 6), dut (n, 6) at `rate` Hz, only samples away from gaps.
    Repeated copies are dropped first, so a frozen stream counts as a gap.
    """
    ref, dut = session.ref.fresh(), session.dut.fresh()
    t0 = max(ref.t[0], dut.t[0] - delay)
    t1 = min(ref.t[-1], dut.t[-1] - delay)
    if t1 - t0 < 5.0:
        raise ValueError('%s: less than 5 s of overlap between the two sensors' % session.name)
    tf = np.arange(t0, t1, 1.0 / fine_rate)
    dt_ = dut.t - delay
    rf = np.column_stack([np.interp(tf, ref.t, ref.w[:, i]) for i in range(6)])
    df = np.column_stack([np.interp(tf, dt_, dut.w[:, i]) for i in range(6)])
    ok = _valid_near(ref.t, tf, max_gap) & _valid_near(dt_, tf, max_gap)
    if len(exclude):
        ok &= ~_in_spans(tf, exclude)
    # stage 1: block average down to `rate` (exact boxcar, same on both)
    m = int(round(fine_rate / rate))
    nb = len(tf) // m
    tf, rf, df, ok = tf[:nb * m], rf[:nb * m], df[:nb * m], ok[:nb * m]
    tb = tf.reshape(nb, m).mean(1)
    rb = rf.reshape(nb, m, 6).mean(1)
    db = df.reshape(nb, m, 6).mean(1)
    okb = ok.reshape(nb, m).all(1)
    # stage 2: zero-phase FIR low-pass at `rate`
    if cutoff and cutoff < rate / 2.0:
        h = _fir_lowpass(cutoff, rate)
        rb = np.column_stack([np.convolve(rb[:, i], h, 'same') for i in range(6)])
        db = np.column_stack([np.convolve(db[:, i], h, 'same') for i in range(6)])
        bad = np.convolve((~okb).astype(float), np.ones(len(h)), 'same') > 0
        okb = ~bad
        edge = len(h) // 2
        okb[:edge] = False
        okb[-edge:] = False
    return tb[okb], rb[okb], db[okb]


# --------------------------------------------------------------------------------------------
# model
# --------------------------------------------------------------------------------------------
QUAD_PAIRS = [(i, j) for i in range(6) for j in range(i, 6)]       # 21 products


def quad_features(dut, scale):
    z = dut / np.asarray(scale, float)
    return np.column_stack([z[:, i] * z[:, j] for i, j in QUAD_PAIRS])


def drift_basis(t, kind):
    """Columns modelling the slowly changing offset of each session.

    'none' -> [1]; 'linear' -> [1, tau]; 'knots:S' -> piecewise-linear with a knot every S s.
    """
    tau = (t - t[0]) / max(t[-1] - t[0], 1e-9)
    if kind == 'none':
        return np.ones((len(t), 1))
    if kind == 'linear':
        return np.column_stack([np.ones(len(t)), tau])
    if kind.startswith('knots:'):
        step = float(kind.split(':', 1)[1])
        n = max(2, int(math.ceil((t[-1] - t[0]) / step)) + 1)
        knots = np.linspace(t[0], t[-1], n)
        B = np.zeros((len(t), n))
        for k in range(n):
            left = knots[k - 1] if k > 0 else knots[0] - 1.0
            right = knots[k + 1] if k < n - 1 else knots[-1] + 1.0
            up = (t - left) / (knots[k] - left)
            down = (right - t) / (right - knots[k])
            B[:, k] = np.clip(np.minimum(up, down), 0.0, 1.0)
        return B
    raise ValueError('unknown drift model %r (none | linear | knots:SECONDS)' % kind)


class Calibration(object):
    """w_out = K . raw + Q . quad(raw / scale) + offset      (offset is replaced by taring)."""

    def __init__(self, K, Q=None, scale=None, offset=None, frame=None, info=None):
        self.K = np.asarray(K, float)
        self.Q = None if Q is None else np.asarray(Q, float)
        self.scale = None if scale is None else np.asarray(scale, float)
        self.offset = np.zeros(6) if offset is None else np.asarray(offset, float)
        self.frame = dict(frame or {})
        self.info = dict(info or {})

    @property
    def model(self):
        return 'quadratic' if self.Q is not None else 'linear'

    def apply(self, raw, with_offset=False):
        raw = np.atleast_2d(np.asarray(raw, float))
        out = raw.dot(self.K.T)
        if self.Q is not None:
            out = out + quad_features(raw, self.scale).dot(self.Q.T)
        if with_offset:
            out = out + self.offset
        return out

    # ---- file I/O (YAML) ----
    def to_dict(self):
        d = {'format': FORMAT,
             'created': datetime.datetime.now().isoformat(timespec='seconds'),
             'model': self.model,
             'axes': list(AXES),
             'units': {'force': 'N', 'torque': 'N*m', 'length': 'm'},
             'equation': 'w = K raw + Q quad(raw/scale) - tare   (quad: products raw_i*raw_j, i<=j)',
             'frame': _plain(self.frame),
             'K': _plain(self.K),
             'offset_at_recording': _plain(self.offset)}
        if self.Q is not None:
            d['Q'] = _plain(self.Q)
            d['scale'] = _plain(self.scale)
            d['quad_pairs'] = [list(p) for p in QUAD_PAIRS]
        d['info'] = _plain(self.info)
        return d

    def save(self, path):
        import yaml
        with open(path, 'w') as f:
            f.write('# HEX calibration from the cross-calibration in optoforce_sensor (HEX_CALIBRATION.md)\n')
            yaml.safe_dump(self.to_dict(), f, default_flow_style=None, sort_keys=False, width=110)

    @classmethod
    def load(cls, path):
        import yaml
        with open(path) as f:
            d = yaml.safe_load(f)
        if d.get('format') != FORMAT:
            raise ValueError('%s: not a %s file' % (path, FORMAT))
        frame = dict(d.get('frame') or {})
        for old_key, new_key in (('R_in_ati', 'R_in_reference'), ('xyz_in_ati_mm', 'xyz_in_reference_mm'),
                                 ('rpy_in_ati_deg', 'rpy_in_reference_deg')):     # files of the first version
            if old_key in frame and new_key not in frame:
                frame[new_key] = frame.pop(old_key)
        return cls(d['K'], d.get('Q'), d.get('scale'), d.get('offset_at_recording'), frame, d.get('info'))


def _plain(x):
    """numpy -> plain python, for YAML."""
    if isinstance(x, dict):
        return {str(k): _plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_plain(v) for v in x]
    if isinstance(x, np.ndarray):
        return _plain(x.tolist())
    if isinstance(x, (np.floating,)):
        return float(x)
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, float):
        return float('%.9g' % x)
    return x


# --------------------------------------------------------------------------------------------
# least squares
# --------------------------------------------------------------------------------------------
class _Design(object):
    """Stacked samples of all sessions with per-session offset/drift columns."""

    def __init__(self, blocks, model, drift, scale):
        self.model, self.drift, self.scale = model, drift, scale
        rows_lin, rows_q, rows_d, Y, sid, T = [], [], [], [], [], []
        ndrift = [drift_basis(b[0], drift).shape[1] for b in blocks]
        for s, (t, ref, dut) in enumerate(blocks):
            rows_lin.append(dut)
            if model == 'quadratic':
                rows_q.append(quad_features(dut, scale))
            D = np.zeros((len(t), sum(ndrift)))
            c0 = sum(ndrift[:s])
            D[:, c0:c0 + ndrift[s]] = drift_basis(t, drift)
            rows_d.append(D)
            Y.append(ref)
            sid.append(np.full(len(t), s))
            T.append(t)
        self.lin = np.vstack(rows_lin)
        self.quad = np.vstack(rows_q) if rows_q else None
        self.drift_cols = np.vstack(rows_d)
        self.Y = np.vstack(Y)
        self.sid = np.concatenate(sid)
        self.t = np.concatenate(T)

    def matrix(self):
        parts = [self.lin]
        if self.quad is not None:
            parts.append(self.quad)
        parts.append(self.drift_cols)
        return np.hstack(parts)

    def n_load_cols(self):
        return 6 + (21 if self.quad is not None else 0)


def _solve(A, Y, w, ridge, n_load):
    """Weighted ridge LS; columns scaled to unit RMS, ridge only on load columns."""
    cs = np.sqrt(np.mean(A ** 2, 0))
    cs[cs == 0] = 1.0
    As = A / cs * w[:, None]
    Ys = Y * w[:, None]
    if ridge > 0:
        reg = np.zeros((n_load, A.shape[1]))
        reg[np.arange(n_load), np.arange(n_load)] = math.sqrt(ridge * len(A))
        As = np.vstack([As, reg])
        Ys = np.vstack([Ys, np.zeros((n_load, Y.shape[1]))])
    coef, _, _, _ = np.linalg.lstsq(As, Ys, rcond=None)
    return coef / cs[:, None]


def fit_map(design, ridge=1e-7, robust=True, iterations=4):
    """Fit ref = [lin quad drift] . coef with Huber-reweighted LS. Returns (coef, residual, weights)."""
    A = design.matrix()
    Y = design.Y
    n_load = design.n_load_cols()
    w = np.ones(len(A))
    for _ in range(iterations if robust else 1):
        coef = _solve(A, Y, w, ridge, n_load)
        res = Y - A.dot(coef)
        if not robust:
            break
        # Huber weights on the per-sample residual norm (axes normalised by their MAD)
        mad = np.median(np.abs(res - np.median(res, 0)), 0) * 1.4826
        mad[mad == 0] = 1.0
        r = np.sqrt(np.mean((res / mad) ** 2, 1))
        c = 2.5
        w = np.where(r <= c, 1.0, np.sqrt(c / r))
    return coef, res, w


# --------------------------------------------------------------------------------------------
# lag (time offset between the two sensors' stamps)
# --------------------------------------------------------------------------------------------
def _quick_residual(session, delay, rate, cutoff, exclude=()):
    t, ref, dut = prepare(session, delay, rate=rate, cutoff=cutoff, exclude=exclude)
    d = _Design([(t, ref, dut)], 'linear', 'linear', None)
    coef, res, _ = fit_map(d, robust=False)
    sd = np.std(ref, 0)
    sd[sd == 0] = 1.0
    return float(np.sqrt(np.mean((res / sd) ** 2)))


def estimate_lag(session, max_lag=0.1, rate=200.0, cutoff=20.0, exclude=()):
    """Delay (s) of the DUT stamps relative to the reference, by grid search + refinement.

    Positive = the DUT is stamped later than the reference (e.g. stamped on arrival over USB).
    Returns (delay, at_search_limit)."""
    coarse = np.arange(-max_lag, max_lag + 1e-9, 0.004)
    costs = [_quick_residual(session, l, rate, cutoff, exclude) for l in coarse]
    best = coarse[int(np.argmin(costs))]
    fine = np.arange(best - 0.004, best + 0.004 + 1e-9, 0.0005)
    costs = np.array([_quick_residual(session, l, rate, cutoff, exclude) for l in fine])
    i = int(np.argmin(costs))
    lag = fine[i]
    if 0 < i < len(fine) - 1:                     # parabolic refinement
        a, b, c = costs[i - 1], costs[i], costs[i + 1]
        den = a - 2 * b + c
        if den > 0:
            lag += 0.5 * (a - c) / den * (fine[1] - fine[0])
    edge = abs(abs(lag) - max_lag) < 0.005
    return float(lag), edge


# --------------------------------------------------------------------------------------------
# decomposition: fitted map  ->  mount transform x DUT correction
# --------------------------------------------------------------------------------------------
def identify_frame(M, prior=None):
    """Split M (DUT raw -> wrench at the reference origin) into the DUT's native frame.

    prior: expected rotation (e.g. CAD nominal); only used to name the flipped axis when the DUT
    reports a left-handed frame. Without it the DUT's z axis is assumed to point along the reference's
    +z (sensors stacked on one axis, both pointing up the stack).
    Returns dict(R, p, mirrored, flip, angle_ff_tt_deg, nonrigid) with R proper (det +1)."""
    Mff, Mtt, Mtf = M[:3, :3], M[3:, 3:], M[3:, :3]
    Qf, Qt = polar_orthogonal(Mff), polar_orthogonal(Mtt)
    mirrored = np.linalg.det(Qf) < 0
    flip = np.ones(3)
    if mirrored:
        # The DUT reports a left-handed frame (one axis with the "wrong" sign). Flipping any one
        # axis gives a proper rotation; they differ by 180 deg turns, so pick the one that best
        # matches the prior (default: DUT z along the reference's +z).
        best = None
        for k in range(3):
            D = np.ones(3)
            D[k] = -1
            Qk = polar_orthogonal(Mff * D)
            score = np.trace(prior.T.dot(Qk)) if prior is not None else Qk[2, 2]
            if best is None or score > best[0]:
                best = (score, D, Qk)
        _, flip, Qf = best
    R = Qf
    S = Mtf.dot(np.linalg.inv(Mff))
    p = vee(S)
    sym = 0.5 * (S + S.T)
    nonrigid = float(np.linalg.norm(sym) / max(np.linalg.norm(S), 1e-12))
    ang = math.degrees(rotation_angle(Qf.T.dot(Qt))) if np.linalg.det(Qt) > 0 else float('nan')
    return {'R': R, 'p': p, 'mirrored': bool(mirrored), 'flip': flip,
            'angle_ff_tt_deg': ang, 'nonrigid': nonrigid}


def frame_from_spec(spec, identified):
    """Output frame pose in the reference frame from a spec dict:
    {'mode': 'native'} | {'mode': 'reference'} | {'mode': 'custom', 'xyz_mm': [..], 'rpy_deg': [..]}
    | {'mode': 'native_at', 'xyz_mm': [...]} (native axes, custom origin)."""
    mode = spec.get('mode', 'native')
    if mode == 'native':
        return identified['R'], identified['p']
    if mode in ('reference', 'ati'):
        return np.eye(3), np.zeros(3)
    xyz = np.asarray(spec.get('xyz_mm', [0, 0, 0]), float) / 1000.0
    if mode == 'native_at':
        return identified['R'], xyz
    if mode == 'custom':
        r, p_, y = np.radians(spec.get('rpy_deg', [0, 0, 0]))
        return rpy_to_matrix(r, p_, y), xyz
    raise ValueError('unknown frame mode %r' % mode)


# --------------------------------------------------------------------------------------------
# coverage / excitation
# --------------------------------------------------------------------------------------------
def coverage(w, targets):
    """Per-axis fraction of [-target, +target] reached on each side, and conditioning of the
    excitation (wrench normalised by targets). Returns dict."""
    w = np.asarray(w, float)
    tg = np.asarray(targets, float)
    z = (w - np.median(w, 0)) / tg
    pos = np.clip(np.percentile(z, 99.5, axis=0), 0, None)
    neg = np.clip(-np.percentile(z, 0.5, axis=0), 0, None)
    C = np.cov(z.T) if len(z) > 6 else np.eye(6)
    ev, V = np.linalg.eigh(C)
    ev = np.clip(ev, 1e-12, None)
    weakest = V[:, 0]
    return {'pos': pos, 'neg': neg, 'cond': float(math.sqrt(ev[-1] / ev[0])),
            'weakest': weakest, 'weakest_text': describe_direction(weakest)}


def excitation_hint(v):
    """Turn the least-excited load direction (coverage()['weakest']) into an instruction."""
    a = np.abs(np.asarray(v, float))
    a = a / (a.max() or 1.0)
    big = set(ax for ax, x in zip(AXES, a) if x > 0.35)
    if big & {'Fx', 'Fy'} and big & {'Tx', 'Ty'}:
        return 'push sideways at another grip height (low vs high) and rock the handle without pushing'
    if big <= {'Tx', 'Ty'}:
        return 'rock the handle forward/back and left/right with little net push'
    if 'Tz' in big and len(big) == 1:
        return 'twist the handle both ways'
    if 'Fz' in big and len(big) == 1:
        return 'press down and pull up'
    return 'more of: ' + describe_direction(v)


def describe_direction(v):
    v = np.asarray(v, float)
    v = v / (np.max(np.abs(v)) or 1.0)
    parts = ['%s%s' % ('+' if x > 0 else '-', a) for a, x in sorted(zip(AXES, v), key=lambda q: -abs(q[1]))
             if abs(x) > 0.35]
    return ' with '.join(parts) if parts else 'all axes'


# --------------------------------------------------------------------------------------------
# the whole fit
# --------------------------------------------------------------------------------------------
def residual_stats(res, sid, t, drift='linear'):
    """RMS / 99th percentile of |res| per axis after removing each session's offset+drift."""
    out = np.empty_like(res)
    for s in np.unique(sid):
        m = sid == s
        B = drift_basis(t[m], drift)
        c, _, _, _ = np.linalg.lstsq(B, res[m], rcond=None)
        out[m] = res[m] - B.dot(c)
    return np.sqrt(np.mean(out ** 2, 0)), np.percentile(np.abs(out), 99, axis=0)


def fit(sessions, model='linear', drift='linear', rate=100.0, cutoff=5.0, max_lag=0.1, lag=None,
        ridge=None, ref_limits=None, ref_margin=0.95, dut_limits=HEX_E_CAPACITY,
        frame_spec=None, nominal=None, folds=5, targets=None, log=print):
    """Calibrate. Returns (Calibration, report dict)."""
    frame_spec = frame_spec or {'mode': 'native'}
    ridge = (1e-7 if model == 'linear' else 1e-4) if ridge is None else ridge
    blocks, lags, notes, used, problems = [], [], [], [], []
    for s in sessions:
        q = session_quality(s)
        for txt in q['text']:
            notes.append('%s: %s' % (s.name, txt))
            log('%s: %s' % (s.name, txt))
        if q['usable_s'] < MIN_SEGMENT_S:
            problems.append('%s: only %.0f s of %.0f s usable (%s)'
                            % (s.name, q['usable_s'], q['overlap_s'], '; '.join(q['text']) or 'too short'))
            log('%s: skipped, only %.0f s usable' % (s.name, q['usable_s']))
            continue
        if lag is None:
            l, edge = estimate_lag(s, max_lag, exclude=q['exclude'])
            if edge:
                notes.append('%s: HEX delay %.1f ms is at the search limit; raise --max-lag' % (s.name, l * 1e3))
        else:
            l = float(lag)
        t, ref, dut = prepare(s, l, rate=rate, cutoff=cutoff, exclude=q['exclude'])
        keep = np.all(np.abs(dut) <= np.asarray(dut_limits), 1)
        if ref_limits is not None:
            keep &= np.all(np.abs(ref) <= ref_margin * np.asarray(ref_limits), 1)
        dropped = 1.0 - keep.mean()
        if dropped > 0.01:
            notes.append('%s: %.1f %% of samples dropped (reference above %.0f %% of its rated '
                         'range, or HEX above capacity)' % (s.name, 100 * dropped, 100 * ref_margin))
        # every stretch between outages gets its own offset and drift (a reconnected sensor may
        # come back with another offset); stretches shorter than MIN_SEGMENT_S are left out
        seg = np.searchsorted([a for a, _ in q['outages']], t)
        n_used = 0
        for k in np.unique(seg):
            m = (seg == k) & keep
            if m.sum() > 1 and t[m][-1] - t[m][0] >= MIN_SEGMENT_S:
                blocks.append((t[m], ref[m], dut[m]))
                n_used += int(m.sum())
        if not n_used:
            problems.append('%s: no stretch of %.0f s without an outage' % (s.name, MIN_SEGMENT_S))
            continue
        lags.append(l)
        used.append(s.name)
        log('%s: %.0f s, HEX stamps %+.2f ms later than the reference, %d samples at %.0f Hz'
            % (s.name, t[-1] - t[0], l * 1e3, n_used, rate))
    if not blocks:
        raise ValueError('no usable data. ' + ' | '.join(problems))
    notes += ['left out: ' + p for p in problems]
    all_dut = np.vstack([b[2] for b in blocks])
    all_ref = np.vstack([b[1] for b in blocks])
    scale = np.maximum(np.percentile(np.abs(all_dut - np.median(all_dut, 0)), 99, axis=0), 1e-3)
    design = _Design(blocks, model, drift, scale)
    coef, res, weights = fit_map(design, ridge=ridge)
    M = coef[:6].T                                         # reference <- DUT raw (linear part)
    Qa = coef[6:27].T if model == 'quadratic' else None

    ident = identify_frame(M, prior=None if nominal is None else nominal[0])
    R_out, p_out = frame_from_spec(frame_spec, ident)
    Xi = wrench_transform_inv(R_out, p_out)
    K = Xi.dot(M)
    Q = Xi.dot(Qa) if Qa is not None else None

    # residuals in the output frame
    ref_out = design.Y.dot(Xi.T)
    pred_out = design.matrix().dot(coef).dot(Xi.T)
    rms_cal, p99_cal = residual_stats(ref_out - pred_out, design.sid, design.t, drift)
    # raw (factory) HEX vs reference, both in the HEX's native frame
    Xn = wrench_transform_inv(ident['R'], ident['p'])
    raw_signed = design.lin * np.sign(np.diag(Xn.dot(M)))     # undo axis sign conventions only
    rms_raw, p99_raw = residual_stats(design.Y.dot(Xn.T) - raw_signed, design.sid, design.t, drift)

    # cross-validation: contiguous blocks of every session left out in turn
    cv = None
    if folds and folds > 1:
        pred_cv = np.zeros_like(design.Y)
        fold_id = np.zeros(len(design.Y), int)
        for s in np.unique(design.sid):
            m = np.where(design.sid == s)[0]
            fold_id[m] = np.minimum((np.arange(len(m)) * folds) // len(m), folds - 1)
        A = design.matrix()
        nl = design.n_load_cols()
        for k in range(folds):
            tr = fold_id != k
            c_k = _solve(A[tr], design.Y[tr], weights[tr], ridge, nl)
            pred_cv[~tr] = A[~tr, :nl].dot(c_k[:nl])         # load part only; offset re-tared below
        res_cv = (design.Y - pred_cv).dot(Xi.T)
        sid_cv = design.sid * folds + fold_id                 # tare per (session, fold)
        cv = residual_stats(res_cv, sid_cv, design.t, 'linear')

    targets = targets or [65, 65, 150, 3, 3, 3]
    cov = coverage(ref_out, targets)

    # does the HEX follow the reference at all? (R^2 per axis, offsets/drift removed)
    yc = np.vstack([ref_out[design.sid == k] - ref_out[design.sid == k].mean(0) for k in np.unique(design.sid)])
    var = np.mean(yc ** 2, 0)
    r2 = 1.0 - rms_cal ** 2 / np.maximum(var, 1e-12)
    excited = np.sqrt(var) > 0.05 * np.asarray(targets, float)
    poor = [a for a, r, e in zip(AXES, r2, excited) if e and r < 0.9]
    if poor:
        notes.insert(0, 'the HEX does not follow the reference on %s (R^2 %s): do NOT use this calibration; '
                     'check the HEX, its cable and the stack' % (', '.join(poor), ', '.join(
                         '%.2f' % r for a, r in zip(AXES, r2) if a in poor)))

    info = {
        'sessions': used, 'hex_delay_ms': [l * 1e3 for l in lags],
        'fit_r2': dict(zip(AXES, r2.tolist())), 'poor_axes': poor,
        'samples': int(len(design.Y)), 'rate_hz': rate, 'lowpass_hz': cutoff, 'drift': drift,
        'ridge': ridge,
        'identified_mount': {
            'xyz_mm': (ident['p'] * 1000).tolist(),
            'rpy_deg': np.degrees(matrix_to_rpy(ident['R'])).tolist(),
            'mirrored_axes': [a for a, f in zip(AXES[:3], ident['flip']) if f < 0],
            'force_vs_torque_axes_deg': ident['angle_ff_tt_deg'],
            'nonrigid_torque_coupling': ident['nonrigid']},
        'error_rms': dict(zip(AXES, rms_cal.tolist())),
        'error_p99': dict(zip(AXES, p99_cal.tolist())),
        'error_rms_crossval': dict(zip(AXES, cv[0].tolist())) if cv else None,
        'factory_error_rms': dict(zip(AXES, rms_raw.tolist())),
        'factory_error_p99': dict(zip(AXES, p99_raw.tolist())),
        'dut_limits': list(dut_limits), 'ref_limits': None if ref_limits is None else list(ref_limits),
        'range_covered': {a: [float(np.percentile(ref_out[:, i], 0.5)), float(np.percentile(ref_out[:, i], 99.5))]
                          for i, a in enumerate(AXES)},
        'excitation_condition': cov['cond'],
        'notes': notes}
    if nominal is not None:
        Rn, pn = nominal
        info['nominal_vs_identified'] = {
            'rotation_deg': math.degrees(rotation_angle(Rn.T.dot(ident['R']))),
            'origin_mm': ((ident['p'] - pn) * 1000).tolist()}
    frame = {'name': frame_spec.get('name', frame_spec.get('mode', 'native')),
             'mode': frame_spec.get('mode', 'native'),
             'R_in_reference': R_out.tolist(), 'xyz_in_reference_mm': (p_out * 1000).tolist(),
             'rpy_in_reference_deg': np.degrees(matrix_to_rpy(R_out)).tolist()}
    offset = np.zeros(6)
    cal = Calibration(K, Q, scale if Q is not None else None, offset, frame, info)
    report = {'M_ref_from_raw': M, 'identified': ident, 'coverage': cov, 'calibration': cal,
              'rms_cal': rms_cal, 'p99_cal': p99_cal, 'rms_raw': rms_raw, 'p99_raw': p99_raw,
              'cv': cv, 'design': design, 'ref_out': ref_out, 'pred_out': pred_out}
    return cal, report


def format_report(cal, rep):
    info = cal.info
    L = []
    if info.get('poor_axes'):
        L.append('*** WARNING: ' + info['notes'][0] + ' ***')
        L.append('')
    L.append('Calibration of the HEX against the reference sensor (%s model, %d samples, drift model %s)'
             % (cal.model, info['samples'], info['drift']))
    L.append('Sessions used: %s' % ', '.join(str(x) for x in info['sessions']))
    L.append('')
    L.append('Errors vs the reference after taring (RMS / 99th percentile). Cross-validation = each part of the')
    L.append('recording predicted by a calibration fitted without it, i.e. what to expect on new data.')
    L.append('')
    L.append('%-4s %4s %19s %19s %19s %14s' % ('axis', 'unit', 'range covered', 'factory HEX', 'calibrated',
                                            'cross-val RMS'))
    for i, a in enumerate(AXES):
        lo, hi = info['range_covered'][a]
        cv = info['error_rms_crossval'][a] if info['error_rms_crossval'] else float('nan')
        fmt = '%8.3f' if i < 3 else '%8.4f'
        L.append(('%-4s %4s %8.2f .. %7.2f ' + fmt + ' /' + fmt + '  ' + fmt + ' /' + fmt + ' %13s')
                 % (a, UNITS[i], lo, hi, rep['rms_raw'][i], rep['p99_raw'][i],
                    rep['rms_cal'][i], rep['p99_cal'][i], (fmt % cv).strip()))
    if 'fit_r2' in info:
        L.append('How well the HEX follows the reference (R^2, 1 = perfectly): ' +
                 ', '.join('%s %.4f' % (a, info['fit_r2'][a]) for a in AXES))
    L.append('')
    K = cal.K
    L.append('Calibration matrix K (output frame <- HEX raw); diagonal = gain correction:')
    for i in range(6):
        L.append('  ' + ' '.join('%9.5f' % v for v in K[i]))
    g = np.diag(K)
    L.append('Factory gain error (+ = the HEX reads too high): ' +
             ', '.join('%s %+.1f%%' % (a, 100 * (1 / abs(gi) - 1)) for a, gi in zip(AXES, g)))
    fs = np.asarray(info.get('dut_limits', HEX_E_CAPACITY), float)
    xt = np.abs(K - np.diag(g)) * fs[None, :] / (np.abs(g)[:, None] * fs[:, None])   # % of full scale
    worst = np.unravel_index(np.argmax(xt), xt.shape)
    L.append('Largest cross-talk corrected: full-scale %s on the HEX shows as %.1f %% of full-scale %s'
             % (AXES[worst[1]], 100 * xt[worst], AXES[worst[0]]))
    m = info['identified_mount']
    L.append('')
    L.append('HEX native frame, identified from the data (in the reference frame):')
    L.append('  origin  x %+7.2f  y %+7.2f  z %+7.2f mm' % tuple(m['xyz_mm']))
    L.append('  rotation roll %+7.2f  pitch %+7.2f  yaw %+7.2f deg' % tuple(m['rpy_deg']))
    if m['mirrored_axes']:
        L.append('  NOTE: the HEX reports a left-handed frame; %s has the opposite sign of a right-'
                 'handed frame. The calibrated output is right-handed.' % ', '.join(m['mirrored_axes']))
    L.append('  force axes vs torque axes differ by %.2f deg; non-rigid torque coupling %.1f %%'
             % (m['force_vs_torque_axes_deg'], 100 * m['nonrigid_torque_coupling']))
    if 'nominal_vs_identified' in info:
        n = info['nominal_vs_identified']
        L.append('  vs CAD nominal: rotation differs %.2f deg, origin differs (%+.1f, %+.1f, %+.1f) mm'
                 % ((n['rotation_deg'],) + tuple(n['origin_mm'])))
    f = cal.frame
    L.append('Output frame "%s": origin (%+.1f, %+.1f, %+.1f) mm, rpy (%+.2f, %+.2f, %+.2f) deg in the reference frame'
             % ((f['name'],) + tuple(f['xyz_in_reference_mm']) + tuple(f['rpy_in_reference_deg'])))
    L.append('Excitation condition number %.1f (below ~10 is good). Weakest direction: %s'
             % (info['excitation_condition'], rep['coverage']['weakest_text']))
    for n in info['notes']:
        L.append('NOTE: ' + n)
    return '\n'.join(L)
