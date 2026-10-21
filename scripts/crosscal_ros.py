"""ROS1 helpers shared by hex_calibrated_node.py and hex_calibration_node.py."""
from __future__ import division, print_function

import os
import threading

import numpy as np

import ft_crosscal as fc


def package_dir():
    """The optoforce_sensor package folder: everything the calibration reads and writes is in it."""
    here = os.path.dirname(os.path.abspath(__file__))
    if os.path.isfile(os.path.join(os.path.dirname(here), 'package.xml')):
        return os.path.dirname(here)
    import rospkg                                   # installed elsewhere: ask ROS
    return rospkg.RosPack().get_path('optoforce_sensor')


def calibration_file():
    """The active HEX calibration: <package>/cfg/hex_calibration.yaml."""
    return os.path.join(package_dir(), 'cfg', 'hex_calibration.yaml')


def vec(msg):
    f, m = msg.wrench.force, msg.wrench.torque
    return np.array([f.x, f.y, f.z, m.x, m.y, m.z])


def fill(msg, w):
    msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = float(w[0]), float(w[1]), float(w[2])
    msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = float(w[3]), float(w[4]), float(w[5])


class CalibratedPublisher(object):
    """Raw HEX messages -> calibrated, tared HEX wrench on out_topic.

    Feed it with on_hex(msg) (raw HEX) and, optionally, on_ref(msg) (the reference sensor): then the
    reference moved into the output frame is published on ref_out_topic, and with drift_tau > 0 the
    HEX's slow offset drift follows the reference (complementary filter, see
    docs/hex_calibration_math.md section 9).

    drop_repeats: messages with exactly the previous values are not passed on. A driver that
    republishes its last packet when the sensor sent nothing new would otherwise turn a dead
    sensor into a constant, plausible-looking force; this way the output goes silent instead
    (and a warning is logged).
    """
    FROZEN_WARN_S = 0.5

    def __init__(self, cal, out_topic='/optoforce_node/OptoForceWrench_calibrated',
                 ref_out_topic='/hex_calibration/reference_in_hex', frame_id=None,
                 zero_seconds=1.0, drift_tau=0.0, with_reference=False, zero_on_start=True, drop_repeats=True):
        import rospy
        from geometry_msgs.msg import WrenchStamped
        self.rospy, self.Msg = rospy, WrenchStamped
        self.cal = cal
        self.frame_id = frame_id or cal.frame.get('name', 'hex_calibrated')
        self.zero_s = float(zero_seconds)
        self.drift_tau = float(drift_tau)
        self.with_ref = bool(with_reference)
        self.drop_repeats = bool(drop_repeats)
        self.last_raw, self.last_new = None, None
        R = np.array(cal.frame['R_in_reference'], float)
        p = np.array(cal.frame['xyz_in_reference_mm'], float) / 1000.0
        self.Xi = fc.wrench_transform_inv(R, p)                  # reference frame -> output frame
        self.lock = threading.Lock()
        self.st = {'tare': None, 'ref_tare': None, 'collect': None, 'ref_collect': None,
                   'ref': None, 'ref_time': None, 'last_t': None, 'out': None, 'ref_out': None}
        self.pub = rospy.Publisher(out_topic, WrenchStamped, queue_size=100)
        self.ref_pub = rospy.Publisher(ref_out_topic, WrenchStamped, queue_size=100) if self.with_ref else None
        self.msg_out = WrenchStamped()
        self.msg_out.header.frame_id = self.frame_id
        if zero_on_start:
            self.zero()
        else:
            self.unzero()

    # ---- tare ----
    def zero(self):
        with self.lock:
            self.st['collect'] = []
            self.st['ref_collect'] = [] if self.with_ref else None

    def unzero(self):
        with self.lock:
            self.st['tare'] = np.zeros(6)
            self.st['ref_tare'] = np.zeros(6) if self.with_ref else None
            self.st['collect'] = self.st['ref_collect'] = None

    def zeroing(self):
        return self.st['collect'] is not None or self.st['tare'] is None

    # ---- inputs ----
    def on_ref(self, msg):
        if not self.with_ref:
            return
        w = self.Xi.dot(vec(msg))
        with self.lock:
            self.st['ref'], self.st['ref_time'] = w, self.rospy.get_time()
            if self.st['ref_collect'] is not None:
                self.st['ref_collect'].append(w)
            ref_tare = self.st['ref_tare']
            if ref_tare is not None:
                self.st['ref_out'] = w - ref_tare
        if self.ref_pub is not None and ref_tare is not None:
            m = self.Msg()
            m.header.stamp, m.header.frame_id = msg.header.stamp, self.frame_id
            fill(m, w - ref_tare)
            self.ref_pub.publish(m)

    def on_hex(self, msg):
        raw = vec(msg)
        wall = self.rospy.get_time()
        if self.last_raw is not None and np.array_equal(raw, self.last_raw):
            if wall - self.last_new > self.FROZEN_WARN_S:
                self.rospy.logwarn_throttle(5.0, 'HEX input frozen for %.1f s (the same values repeated): the sensor '
                                            'stopped sending; check its cable/USB and driver.%s'
                                            % (wall - self.last_new, ' Not publishing until new data arrives.'
                                               if self.drop_repeats else ''))
            if self.drop_repeats:
                return
        else:
            self.last_raw, self.last_new = raw, wall
        w = self.cal.apply(raw)[0]
        now = msg.header.stamp.to_sec() or self.rospy.get_time()
        st = self.st
        with self.lock:
            if st['collect'] is not None:
                st['collect'].append(w)
                if len(st['collect']) == 1:
                    st['zero_t0'] = now
                elif now - st['zero_t0'] >= self.zero_s:
                    st['tare'] = np.mean(st['collect'], 0)
                    if st['ref_collect']:
                        st['ref_tare'] = np.mean(st['ref_collect'], 0)
                    st['collect'], st['ref_collect'] = None, None
            tare = st['tare']
            if tare is None:
                return
            out = w - tare
            # drift tracking: pull the HEX's low frequencies onto the reference's
            if self.drift_tau > 0 and st['ref'] is not None and st['ref_tare'] is not None \
                    and self.rospy.get_time() - st['ref_time'] < 0.05 and st['last_t'] is not None:
                dt = min(max(now - st['last_t'], 0.0), 0.1)
                err = out - (st['ref'] - st['ref_tare'])
                st['tare'] = tare + err * (dt / self.drift_tau)
            st['last_t'] = now
            st['out'] = out
        self.msg_out.header.stamp = msg.header.stamp
        fill(self.msg_out, out)
        self.pub.publish(self.msg_out)
