#!/usr/bin/env python3
"""Write the rqt_multiplot layouts of the HEX calibration (developer tool; the files are in cfg/):

    python3 test/make_multiplot_xml.py            write cfg/hex_calibration_plots.xml and _verify.xml
    python3 test/make_multiplot_xml.py --check    only check that they are up to date

hex_calibration_plots.xml (data from hex_calibration_monitor.py), 3 x 3 plots. Rows 1-2: the
reference Fx Fy Fz / Tx Ty Tz (the load you apply) with constant lines at its rated range (red), at
~warn of it (orange, the "full scale" to aim for) and the peaks reached so far (green). Row 3:
reference load in % (highest axis), HEX forces, HEX torques. The y axes scale automatically, so the
layout works for any reference sensor: its range comes from cfg/hex_calibration_params.yaml through
the monitor's limit lines.
hex_calibration_verify.xml, 2 x 3: calibrated HEX against the reference moved into the HEX frame.
"""
from __future__ import print_function

import argparse
import os
import sys
from xml.sax.saxutils import escape

CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'cfg')
T = '/hex_calibration/plot/'
WRENCH = 'geometry_msgs/WrenchStamped'
VEC = 'geometry_msgs/Vector3Stamped'
SOLID, DASH, DOT = 1, 2, 3                       # Qt::PenStyle


def el(name, content, ind):
    pad = '    ' * ind
    if isinstance(content, list):
        return '%s<%s>\n%s\n%s</%s>' % (pad, name, '\n'.join(content), pad, name)
    return '%s<%s>%s</%s>' % (pad, name, escape(str(content)), name)


def axis_scale(kind, lo, hi, ind):
    # kind: 0 auto, 1 absolute, 2 relative (CurveAxisScaleConfig::Type)
    return el('scale', [el('absolute_maximum', hi if kind == 1 else 1000, ind + 1),
                        el('absolute_minimum', lo if kind == 1 else 0, ind + 1),
                        el('relative_maximum', hi if kind == 2 else 0, ind + 1),
                        el('relative_minimum', lo if kind == 2 else -1000, ind + 1),
                        el('type', kind, ind + 1)], ind)


def curve(idx, title, topic, mtype, field, color, width, style, y_scale, window, ind):
    x = el('x_axis', [el('field', '', ind + 3), el('field_type', 1, ind + 3),       # receipt time
                      axis_scale(2, -window, 0, ind + 3),
                      el('topic', topic, ind + 3), el('type', mtype, ind + 3)], ind + 2)
    y = el('y_axis', [el('field', field, ind + 3), el('field_type', 0, ind + 3),
                      axis_scale(y_scale[0], y_scale[1], y_scale[2], ind + 3),
                      el('topic', topic, ind + 3), el('type', mtype, ind + 3)], ind + 2)
    return el('curve_%d' % idx, [
        el('axes', [x, y], ind + 1),
        el('color', [el('custom_color', color, ind + 2), el('type', 1, ind + 2)], ind + 1),
        el('data', [el('circular_buffer_capacity', 2000, ind + 2), el('time_frame_length', window, ind + 2),
                    el('type', 3, ind + 2)], ind + 1),                                    # TimeFrame
        el('style', [el('lines_interpolate', 'false', ind + 2), el('pen_style', style, ind + 2),
                     el('pen_width', width, ind + 2), el('render_antialias', 'true', ind + 2),
                     el('steps_invert', 'false', ind + 2), el('sticks_baseline', 0, ind + 2),
                     el('sticks_orientation', 2, ind + 2), el('type', 0, ind + 2)], ind + 1),
        el('subscriber_queue_size', 100, ind + 1),
        el('title', title, ind + 1)], ind)


def plot(row_col, title, ylabel, curves, legend, ind):
    axes = el('axes', [el('axes', [
        el('x_axis', [el('custom_title', 'time [s]', ind + 4), el('title_type', 1, ind + 4),
                      el('title_visible', 'false', ind + 4)], ind + 3),
        el('y_axis', [el('custom_title', ylabel, ind + 4), el('title_type', 1, ind + 4),
                      el('title_visible', 'true', ind + 4)], ind + 3)], ind + 2)], ind + 1)
    return el(row_col, [axes, el('curves', curves, ind + 1),
                        el('legend', [el('visible', 'true' if legend else 'false', ind + 2)], ind + 1),
                        el('plot_rate', 30, ind + 1), el('title', title, ind + 1)], ind)


def build(warn, window):
    names = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz']
    fields = ['wrench/force/x', 'wrench/force/y', 'wrench/force/z',
              'wrench/torque/x', 'wrench/torque/y', 'wrench/torque/z']
    units = ['N', 'N', 'N', 'N*m', 'N*m', 'N*m']
    rows = {0: [], 1: [], 2: []}
    ind = 4                                   # depth of a column_N element
    auto = (0, 0, 0)
    for i in range(6):
        cs = [curve(0, 'reference %s' % names[i], T + 'reference', WRENCH, fields[i], '#1d4ed8', 2, SOLID, auto, window, ind + 2),
              curve(1, '+rated', T + 'limit_max', WRENCH, fields[i], '#dc2626', 2, SOLID, auto, window, ind + 2),
              curve(2, '-rated', T + 'limit_min', WRENCH, fields[i], '#dc2626', 2, SOLID, auto, window, ind + 2),
              curve(3, '+%d %%' % round(100 * warn), T + 'warn_max', WRENCH, fields[i], '#f59e0b', 2, DASH, auto, window, ind + 2),
              curve(4, '-%d %%' % round(100 * warn), T + 'warn_min', WRENCH, fields[i], '#f59e0b', 2, DASH, auto, window, ind + 2),
              curve(5, 'reached +', T + 'peak_max', WRENCH, fields[i], '#16a34a', 2, DOT, auto, window, ind + 2),
              curve(6, 'reached -', T + 'peak_min', WRENCH, fields[i], '#16a34a', 2, DOT, auto, window, ind + 2)]
        title = 'reference %s  (red = rated, orange %d %%, green = reached)' % (names[i], round(100 * warn))
        rows[i // 3].append(plot('column_%d' % (i % 3), title, '%s [%s]' % (names[i], units[i]), cs, False, ind))
    ys = (1, 0, 115)
    cs = [curve(0, 'load (highest axis)', T + 'reference_load', VEC, 'vector/x', '#1d4ed8', 2, SOLID, ys, window, ind + 2),
          curve(1, '%d %%' % round(100 * warn), T + 'reference_load', VEC, 'vector/y', '#f59e0b', 2, DASH, ys, window, ind + 2),
          curve(2, '100 % rated', T + 'reference_load', VEC, 'vector/z', '#dc2626', 2, SOLID, ys, window, ind + 2)]
    rows[2].append(plot('column_0', 'reference load: highest axis in % of rated', '%', cs, True, ind))
    colors = ['#dc2626', '#16a34a', '#1d4ed8']
    for col, (label, sl, unit) in enumerate((('forces', slice(0, 3), 'N'), ('torques', slice(3, 6), 'N*m')), start=1):
        cs = [curve(k, 'HEX ' + names[j], T + 'hex', WRENCH, fields[j], colors[k], 2, SOLID, auto, window, ind + 2)
              for k, j in enumerate(range(6)[sl])]
        rows[2].append(plot('column_%d' % col, 'HEX raw %s (own axes, tared)' % label, unit, cs, True, ind))
    plots = el('plots', [el('row_%d' % r, rows[r], 3) for r in (0, 1, 2)], 2)
    table = el('table', [el('background_color', '#ffffff', 2), el('foreground_color', '#000000', 2),
                         el('link_cursor', 'false', 2), el('link_scale', 'false', 2), plots,
                         el('track_points', 'false', 2)], 1)
    return '<?xml version="1.0" encoding="UTF-8"?>\n' + el('rqt_multiplot', [table], 0) + '\n'


def build_verify(window):
    """After calibration: calibrated HEX (blue) against the reference moved into the HEX frame (black)."""
    names = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz']
    fields = ['wrench/force/x', 'wrench/force/y', 'wrench/force/z',
              'wrench/torque/x', 'wrench/torque/y', 'wrench/torque/z']
    units = ['N', 'N', 'N', 'N*m', 'N*m', 'N*m']
    rows = {0: [], 1: []}
    auto = (0, 0, 0)
    for i in range(6):
        cs = [curve(0, 'reference in HEX frame', T + 'reference_in_hex', WRENCH, fields[i], '#111827', 2, SOLID, auto, window, 6),
              curve(1, 'HEX calibrated', T + 'hex_calibrated', WRENCH, fields[i], '#1d4ed8', 2, DASH, auto, window, 6)]
        rows[i // 3].append(plot('column_%d' % (i % 3), 'Calibrated HEX vs reference: %s' % names[i],
                                 '%s [%s]' % (names[i], units[i]), cs, i == 0, 4))
    plots = el('plots', [el('row_%d' % r, rows[r], 3) for r in (0, 1)], 2)
    table = el('table', [el('background_color', '#ffffff', 2), el('foreground_color', '#000000', 2),
                         el('link_cursor', 'false', 2), el('link_scale', 'false', 2), plots,
                         el('track_points', 'false', 2)], 1)
    return '<?xml version="1.0" encoding="UTF-8"?>\n' + el('rqt_multiplot', [table], 0) + '\n'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--warn', type=float, default=0.9, help='second line at this fraction of rated (as ~warn)')
    ap.add_argument('--window', type=float, default=10.0, help='seconds of history shown')
    ap.add_argument('--check', action='store_true', help='only check that the files in cfg/ are up to date')
    a = ap.parse_args()
    files = {os.path.join(CFG, 'hex_calibration_plots.xml'): build(a.warn, a.window),
             os.path.join(CFG, 'hex_calibration_verify.xml'): build_verify(a.window)}
    bad = 0
    for path, xml in files.items():
        if a.check:
            ok = os.path.isfile(path) and open(path).read() == xml
            print('%s %s' % (os.path.relpath(path), 'is up to date' if ok else 'is OUT OF DATE'))
            bad += not ok
        else:
            with open(path, 'w') as f:
                f.write(xml)
            print('wrote ' + os.path.relpath(path))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
