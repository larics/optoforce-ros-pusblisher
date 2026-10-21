#!/usr/bin/env python3
"""ROS1: live data for the rqt_multiplot layouts of the HEX calibration (cfg/hex_calibration_plots.xml,
cfg/hex_calibration_verify.xml): how far you load the stack, against the reference's rated range.

    rosrun optoforce_sensor hex_calibration_monitor.py    (the tmux session and launch file start it)

Publishes at ~rate (default 50 Hz, averaged, light enough for plotting), all WrenchStamped:
  /hex_calibration/plot/reference            reference wrench, tared at start (= the load you apply)
  /hex_calibration/plot/hex                  HEX raw wrench, tared at start (its own axes)
  /hex_calibration/plot/limit_max, limit_min +/- reference rated range  (constant lines: never cross)
  /hex_calibration/plot/warn_max,  warn_min  +/- ~warn x rated (90 %)   (constant lines: aim here)
  /hex_calibration/plot/peak_max,  peak_min  highest / lowest reference value reached so far
  /hex_calibration/plot/reference_load       Vector3Stamped: x = highest axis load in % of rated,
                                             y = warn %, z = 100 %
  /hex_calibration/plot/hex_calibrated, reference_in_hex
                                             50 Hz copies of ~output_topic and ~reference_output
                                             (what the calibration node publishes after the fit)
Services: ~zero (tare again, hands off; also resets the peaks), ~reset_peaks
Parameters (cfg/hex_calibration_params.yaml): ~reference_topic, ~hex_topic, ~reference_limits
  (without them the limit lines show ~targets), ~warn (0.9), ~targets, ~rate, ~zero_seconds (1.0),
  ~output_topic, ~reference_output
Warns (log) when a sensor's values stop changing (FROZEN: its driver repeats the last packet) or
its messages stop (NO DATA).
"""
from __future__ import division, print_function

import threading
import time

import numpy as np


def main():
    import rospy
    from geometry_msgs.msg import WrenchStamped, Vector3Stamped
    from std_srvs.srv import Trigger, TriggerResponse

    rospy.init_node('hex_calibration_monitor')
    ref_topic = rospy.get_param('~reference_topic', '')
    if not ref_topic:
        rospy.logfatal('set ~reference_topic (cfg/hex_calibration_params.yaml)')
        return
    hex_topic = rospy.get_param('~hex_topic', '/optoforce_node/OptoForceWrench')
    rate = float(rospy.get_param('~rate', 50.0))
    warn = float(rospy.get_param('~warn', 0.9))

    def six(v):
        return None if v in (None, '', []) else np.array(
            [float(x) for x in (v if isinstance(v, (list, tuple)) else str(v).split(','))])
    limits = six(rospy.get_param('~reference_limits', None))
    if limits is None:                     # unknown rated range: show the targets instead
        limits = six(rospy.get_param('~targets', [40, 40, 80, 2, 2, 2])) / warn
    zero_s = float(rospy.get_param('~zero_seconds', 1.0))

    lock = threading.Lock()
    acc = {'reference': [np.zeros(6), 0], 'hex': [np.zeros(6), 0], 'hex_calibrated': [np.zeros(6), 0],
           'reference_in_hex': [np.zeros(6), 0]}
    st = {'tare': {'reference': None, 'hex': None}, 'zero': {'reference': [], 'hex': []}, 'zero_until': None,
          'peak_max': np.zeros(6), 'peak_min': np.zeros(6)}
    health = {k: {'prev': None, 'new': 0.0, 'msg': 0.0, 'state': 'ok'} for k in ('reference', 'hex')}

    def vec(m):
        f, t = m.wrench.force, m.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def cb(msg, key):
        v = vec(msg)
        with lock:
            acc[key][0] += v
            acc[key][1] += 1
            h = health.get(key)
            if h is not None:
                now = time.time()
                h['msg'] = now
                if h['prev'] is None or np.any(v != h['prev']):
                    h['new'] = now
                h['prev'] = v

    def check_health():
        now = time.time()
        for k, who, topic in (('reference', 'reference', ref_topic), ('hex', 'HEX', hex_topic)):
            h = health[k]
            if not h['msg']:
                continue
            state = 'NO DATA' if now - h['msg'] > 0.5 else ('FROZEN' if now - h['new'] > 0.5 else 'ok')
            if state != 'ok':
                rospy.logwarn_throttle(5.0, '%s %s for %.0f s on %s: %s' % (
                    who, state, now - (h['msg'] if state == 'NO DATA' else h['new']), topic,
                    'its values do not change, the sensor stopped sending (driver repeats the last packet); '
                    'check cable/USB' if state == 'FROZEN' else 'no messages; is its driver running?'))
            elif h['state'] != 'ok':
                rospy.loginfo('%s data flowing again' % who)
            h['state'] = state

    def wmsg(w, stamp, frame=''):
        m = WrenchStamped()
        m.header.stamp, m.header.frame_id = stamp, frame
        m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = (float(x) for x in w[:3])
        m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z = (float(x) for x in w[3:])
        return m

    pubs = {k: rospy.Publisher('/hex_calibration/plot/' + k, WrenchStamped, queue_size=10)
            for k in ('reference', 'hex', 'limit_max', 'limit_min', 'warn_max', 'warn_min', 'peak_max', 'peak_min',
                      'hex_calibrated', 'reference_in_hex')}
    load_pub = rospy.Publisher('/hex_calibration/plot/reference_load', Vector3Stamped, queue_size=10)

    def start_zero():
        with lock:
            st['zero'] = {'reference': [], 'hex': []}
            st['zero_until'] = rospy.get_time() + zero_s

    def tick(_):
        now = rospy.Time.now()
        check_health()
        with lock:
            means = {}
            for k in acc:
                s, n = acc[k]
                if n:
                    means[k] = s / n
                acc[k] = [np.zeros(6), 0]
            zeroing = st['zero_until'] is not None
            if zeroing:
                for k in ('reference', 'hex'):
                    if k in means:
                        st['zero'][k].append(means[k])
                if rospy.get_time() >= st['zero_until'] and st['zero']['reference'] and st['zero']['hex']:
                    for k in ('reference', 'hex'):
                        st['tare'][k] = np.mean(st['zero'][k], 0)
                    st['peak_max'][:] = 0
                    st['peak_min'][:] = 0
                    st['zero_until'] = None
                    rospy.loginfo('tared the reference and the HEX; peaks reset')
            tare = dict(st['tare'])
        for k in ('hex_calibrated', 'reference_in_hex'):  # already calibrated/tared: pass through
            if k in means:
                pubs[k].publish(wmsg(means[k], now))
        for name, w in (('limit_max', limits), ('limit_min', -limits),
                        ('warn_max', warn * limits), ('warn_min', -warn * limits)):
            pubs[name].publish(wmsg(w, now))
        if zeroing or tare['reference'] is None:
            return
        if 'reference' in means:
            a = means['reference'] - tare['reference']
            with lock:
                st['peak_max'] = np.maximum(st['peak_max'], a)
                st['peak_min'] = np.minimum(st['peak_min'], a)
                pmax, pmin = st['peak_max'].copy(), st['peak_min'].copy()
            pubs['reference'].publish(wmsg(a, now))
            pubs['peak_max'].publish(wmsg(pmax, now))
            pubs['peak_min'].publish(wmsg(pmin, now))
            v = Vector3Stamped()
            v.header.stamp = now
            v.vector.x = float(100.0 * np.max(np.abs(a) / limits))
            v.vector.y = 100.0 * warn
            v.vector.z = 100.0
            load_pub.publish(v)
        if 'hex' in means and tare['hex'] is not None:
            pubs['hex'].publish(wmsg(means['hex'] - tare['hex'], now))

    def srv_zero(_):
        start_zero()
        return TriggerResponse(True, 'taring for %.1f s: hands off the handle' % zero_s)

    def srv_reset(_):
        with lock:
            st['peak_max'][:] = 0
            st['peak_min'][:] = 0
        return TriggerResponse(True, 'peaks reset')

    rospy.Subscriber(ref_topic, WrenchStamped, cb, 'reference', queue_size=2000, tcp_nodelay=True)
    rospy.Subscriber(hex_topic, WrenchStamped, cb, 'hex', queue_size=2000, tcp_nodelay=True)
    rospy.Subscriber(rospy.get_param('~output_topic', '/optoforce_node/OptoForceWrench_calibrated'),
                     WrenchStamped, cb, 'hex_calibrated', queue_size=2000)
    rospy.Subscriber(rospy.get_param('~reference_output', '/hex_calibration/reference_in_hex'),
                     WrenchStamped, cb, 'reference_in_hex', queue_size=2000)
    rospy.Service('~zero', Trigger, srv_zero)
    rospy.Service('~reset_peaks', Trigger, srv_reset)
    start_zero()
    rospy.Timer(rospy.Duration(1.0 / rate), tick)
    rospy.loginfo('HEX calibration monitor: %s + %s -> /hex_calibration/plot/* at %.0f Hz; taring for %.1f s '
                  '(hands off)' % (ref_topic, hex_topic, rate, zero_s))
    rospy.spin()


if __name__ == '__main__':
    main()
