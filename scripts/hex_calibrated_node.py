#!/usr/bin/env python3
"""ROS1 node: the HEX wrench corrected with its calibration (from hex_calibration_node.py).

    roslaunch optoforce_sensor hex_calibrated.launch
    rosrun optoforce_sensor hex_calibrated_node.py

Parameters
  ~calibration      calibration file (default: this package's cfg/hex_calibration.yaml)
  ~input_topic      raw HEX (default /optoforce_node/OptoForceWrench)
  ~output_topic     calibrated HEX (default /optoforce_node/OptoForceWrench_calibrated), N and N*m
  ~frame_id         default: the frame name stored in the calibration (hex_calibrated)
  ~zero_seconds     tare over the first second (default 1.0; keep the sensor unloaded)
  ~zero_on_start    default true
  ~drop_repeats     default true: a message with exactly the previous values is not passed on, so
                    a dead sensor whose driver republishes its last packet makes the output go
                    silent instead of showing a constant force (a warning is logged)
Services  ~zero, ~unzero (std_srvs/Trigger): tare again / remove the tare

Optional, while the HEX still sits on a reference sensor:
  ~reference_topic  any WrenchStamped: also publish it moved into the HEX frame on
                    ~reference_output (default /hex_calibration/reference_in_hex)
  ~drift_tau        > 0: remove the HEX's slow offset drift by following the reference with this
                    time constant (s)
"""
from __future__ import division, print_function

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ft_crosscal as fc  # noqa: E402
from crosscal_ros import CalibratedPublisher, calibration_file  # noqa: E402


def main():
    import rospy
    from geometry_msgs.msg import WrenchStamped
    from std_srvs.srv import Trigger, TriggerResponse

    rospy.init_node('hex_calibrated')
    path = os.path.expanduser(rospy.get_param('~calibration', '') or calibration_file())
    if not os.path.isfile(path):
        rospy.logfatal('no calibration file %s: run the HEX calibration first (HEX_CALIBRATION.md)' % path)
        return
    cal = fc.Calibration.load(path)
    in_topic = rospy.get_param('~input_topic', '/optoforce_node/OptoForceWrench')
    out_topic = rospy.get_param('~output_topic', '/optoforce_node/OptoForceWrench_calibrated')
    frame_id = rospy.get_param('~frame_id', '') or cal.frame.get('name', 'hex_calibrated')
    zero_s = float(rospy.get_param('~zero_seconds', 1.0))
    ref_topic = rospy.get_param('~reference_topic', '')
    ref_out_topic = rospy.get_param('~reference_output', '/hex_calibration/reference_in_hex')
    drift_tau = float(rospy.get_param('~drift_tau', 0.0))

    pub = CalibratedPublisher(cal, out_topic, ref_out_topic, frame_id, zero_s, drift_tau,
                              with_reference=bool(ref_topic),
                              zero_on_start=rospy.get_param('~zero_on_start', True),
                              drop_repeats=rospy.get_param('~drop_repeats', True))

    def srv_zero(_):
        pub.zero()
        return TriggerResponse(True, 'collecting %.1f s for the tare: keep the sensor unloaded' % zero_s)

    def srv_unzero(_):
        pub.unzero()
        return TriggerResponse(True, 'tare removed (output = calibration only)')

    rospy.Service('~zero', Trigger, srv_zero)
    rospy.Service('~unzero', Trigger, srv_unzero)
    if ref_topic:
        rospy.Subscriber(ref_topic, WrenchStamped, pub.on_ref, queue_size=100, tcp_nodelay=True)
    rospy.Subscriber(in_topic, WrenchStamped, pub.on_hex, queue_size=100, tcp_nodelay=True)
    rospy.loginfo('HEX calibration %s (%s model, frame %s): %s -> %s%s'
                  % (path, cal.model, frame_id, in_topic, out_topic,
                     ('; reference %s' % ref_topic + (', drift tracking tau %.0f s' % drift_tau if drift_tau > 0 else ''))
                     if ref_topic else ''))
    rospy.spin()


if __name__ == '__main__':
    main()
