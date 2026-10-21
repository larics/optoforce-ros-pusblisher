# HEX calibration against a reference F/T sensor

The HEX is bolted on top of a reference F/T sensor, so every load on a handle above passes
through both. The reference can be **any sensor that publishes `geometry_msgs/WrenchStamped`**
and is clearly more accurate than the HEX (we use an ATI Gamma). A few minutes of loading the
handle give the HEX a full 6×6 calibration in its own frame: gains and cross-talk, optionally
non-linearity. The theory, limits and error budget are in
[docs/hex_calibration_math.md](docs/hex_calibration_math.md).

**Everything stays in this package.** Nothing is read from or written to anywhere else:

| path | |
|---|---|
| `cfg/hex_calibration_params.yaml` | the settings: the reference sensor (topic, rated range), the HEX topic, targets, model |
| `cfg/hex_calibration.yaml` | **the active calibration**: written by every usable fit, loaded by `hex_calibrated.launch` |
| `calibration/<date_time>/` | one folder per run: recordings `s1.npz …` (not in git), that run's `hex_calibration.yaml` and `hex_calibration_report.txt` |
| `scripts/hex_calibration_node.py` | records, fits, writes the calibration, then publishes the calibrated HEX |
| `scripts/hex_calibrated_node.py` | publishes the calibrated HEX from `cfg/hex_calibration.yaml` |
| `scripts/hex_calibration_monitor.py` | data for the live plots |
| `scripts/hex_calibration_check.py`, `scripts/hex_calibration_fit.py` | check a recording or a bag; refit offline |
| `scripts/ft_crosscal.py`, `scripts/crosscal_ros.py` | the maths and shared ROS code (no ROS needed for the maths) |
| `launch/hex_calibration.launch`, `launch/hex_calibrated.launch` | |
| `cfg/hex_calibration_plots.xml`, `cfg/hex_calibration_verify.xml` | rqt_multiplot layouts |
| `startup/hex_calibration/` | the tmuxinator session |
| `test/` | tests with simulated sensors: `python3 test/test_hex_calibration.py` |

## 1. Say what the reference is

Edit the first block of `cfg/hex_calibration_params.yaml`; that is the only place that knows the
reference sensor:

```yaml
reference_topic: /ft_sensor/wrench                   # any WrenchStamped topic
reference_limits: [65.0, 65.0, 200.0, 5.0, 5.0, 5.0]  # its rated range; [] if unknown
reference_invalid_log: "gauge saturated"             # what its driver logs for invalid readings; "" = none
reference_name: REF                                  # label on the dashboard
```

The tmux session also starts the two drivers. They are set in
`startup/hex_calibration/hex_calibration.sh` (`REFERENCE_DRIVER`, `HEX_DRIVER`; empty = start
them yourself).

## 2. Run it

```bash
cd $(rospack find optoforce_sensor)/startup/hex_calibration
./start.sh
```

| window | panes |
|---|---|
| 0 `roscore` | roscore · reference driver · HEX driver · `rostopic hz` of both |
| 1 `calibration` | **the calibration node: dashboard + keys** · cheat sheet (arrow-up recalls the useful commands) |
| 2 `plots` | monitor · rqt_multiplot: the reference against its rated range (red), 90 % (orange) and what you reached (green) |
| 3 `verify` | opens by itself after the fit: calibrated HEX against the reference, axis by axis · its values as text |

Stop everything: `tmux kill-session -t hex_calibration`. It needs tmuxinator and `rqt_multiplot`
(`apt install ros-noetic-rqt-multiplot`).

Without tmux: `roslaunch optoforce_sensor hex_calibration.launch` (plus your drivers). Under
roslaunch there is no keyboard: use `rosservice call /hex_calibration/{start,stop,fit,zero,discard}`.

| key | |
|---|---|
| `s` | start / stop a recording session |
| `f` | fit with all sessions of this run, write the calibration, start publishing the calibrated HEX |
| `z` | tare (hands off for 1 s) |
| `d` | delete the last session |
| `m` | model for the next fit: linear ↔ quadratic |
| `h` / `q` | help / quit |

Procedure:

1. Hands off until it says *tared*. Let the sensors warm up 20–30 minutes beforehand.
2. Press `s` and load the handle slowly for 3–5 minutes. Bring every axis towards 100 % on the
   dashboard (the orange line in the plots) in + and −, but never past red: sideways low and high
   on the handle, down and up, twist, rock without pushing, hands off now and then. Combined
   loads (pressing down hard while twisting) can overload the reference below 100 % per axis.
3. Press `s` again to stop, and record a second session the same way. When the excitation line
   says *ready to fit*, press `f`.

The fit writes `calibration/<date_time>/hex_calibration.yaml` and the report, makes it the active
`cfg/hex_calibration.yaml`, and starts publishing the calibrated HEX on
`/optoforce_node/OptoForceWrench_calibrated` (window 3 compares it with the reference).

The dashboard after two sessions and the fit (a real capture, with simulated sensors; on real
sensors expect larger errors, set by the HEX's hysteresis and the reference's own accuracy):

```
HEX calibration against REF    state: CALIBRATED: publishing /optoforce_node/OptoForceWrench_calibrated    model: linear
run optoforce_sensor/calibration/2026-10-21_094617
streams: REF 1003 Hz   HEX 1001 Hz (500 new)      tare: ok
sessions: s1 75 s   s2 75 s
load you apply (REF, tared) and how far you got, in % of the orange line (90 % rated):
  axis        now       - reached                + reached
  Fx      -0.00 N    .......#######   48 %   #######.......   47 %
  Fy      +0.00 N    ........######   41 %   #######.......   47 %
  Fz      -0.00 N    .........#####   36 %   #####.........   35 %
  Tx      -0.44 N*m  ##############  107 %   #########.....   66 %
  Ty      -0.78 N*m  ##############  117 %   ############..   83 %
  Tz      -0.01 N*m  ........######   41 %   ######........   44 %
REF load now:  16 % (Ty)   highest while recording: 106 % Ty   over 95 %: 713   invalid (driver log): 0
HEX excitation: condition 7.7 (aim < 10)   enough variety
fit: HEX stamps 4.0 ms late | HEX mount: yaw +23.3, tilt -1.5/-0.7 deg, z +37.8 mm
     report: optoforce_sensor/calibration/2026-10-21_094617/hex_calibration_report.txt
calibrated HEX vs REF (tared; err = RMS over the last 5 s; frame hex_calibrated):
  axis    REF now  HEX cal now    err RMS  factory err   fit cross-val
  Fx       0.000        0.020      0.021        0.479       0.005  N
  Fy       0.096        0.116      0.024        0.949       0.004  N
  Fz       0.003        0.060      0.387        0.618       0.019  N
  Tx     -0.0044      -0.0028     0.0017       0.6994      0.0003  N*m
  Ty      0.0000       0.0004     0.0016       0.5511      0.0003  N*m
  Tz      0.0000       0.0004     0.0002       0.0238      0.0001  N*m
[s] session start/stop  [f] fit  [z] tare  [d] delete last  [m] model  [h] help  [q] quit
> 09:47:56  recording s2: hands off for 3 s, then load the handle slowly in every direction
> 09:49:12  saved optoforce_sensor/calibration/2026-10-21_094617/s2.npz (75 s). [s] another session (2+ recommended), [f]
  fit
> 09:49:15  fitting 2 session(s), linear model ... (about 16 s)
> 09:49:27  calibration active: optoforce_sensor/cfg/hex_calibration.yaml (this run:
  optoforce_sensor/calibration/2026-10-21_094617/hex_calibration.yaml), 11 s
> 09:49:27  publishing /optoforce_node/OptoForceWrench_calibrated: hands off for 1 s (tare). Compare in the verify window.
```

## 3. Use the calibration

```bash
roslaunch optoforce_sensor hex_calibrated.launch
```

`/optoforce_node/OptoForceWrench` → `/optoforce_node/OptoForceWrench_calibrated`, frame
`hex_calibrated` (the HEX's own axes, right-handed). It tares over the first second: keep the HEX
unloaded then. Tare again: `rosservice call /hex_calibrated/zero`. In your own launch file:

```xml
<node name="hex_calibrated" pkg="optoforce_sensor" type="hex_calibrated_node.py" output="screen">
    <param name="calibration"  value="$(find optoforce_sensor)/cfg/hex_calibration.yaml"/>
    <param name="input_topic"  value="/optoforce_node/OptoForceWrench"/>
    <param name="output_topic" value="/optoforce_node/OptoForceWrench_calibrated"/>
</node>
```

If the HEX stops sending, the calibrated topic goes silent (with a warning) instead of repeating
an old value.

## 4. When something is wrong

The dashboard checks both streams all the time:

* **`HEX FROZEN`** (or the reference): messages keep coming, but with exactly the same values. The
  sensor stopped sending and its driver repeats the last packet. Check the sensor cable and
  connectors, the USB cable and the USB passthrough into the container (`dmesg -w` on the host
  while you wiggle the cable), then restart the driver.
* **`NO DATA`**: no messages at all. Is the driver running?
* **`(500 new)`** after a rate: only that many of the messages per second carry new values; the
  others are copies (harmless, they are dropped).

For example, the HEX freezing 40 s into a session (simulated):

```
streams: REF  999 Hz   HEX FROZEN 8 s      tare: ok
  HEX repeats the same values: the sensor stopped sending. Check cable, connector, USB; restart its driver
> 09:51:43  HEX FROZEN: the sensor stopped sending (check its cable/USB/driver) at 00:40 of s1: recording stopped
> 09:51:48  HEX FROZEN for 5 s: not recording. Check the sensor, its cable/USB and driver
```

A recording stops by itself when a sensor freezes or goes silent. The part before it, minus 2 s
(often already wrong), is kept if it is at least 10 s long. A fit that finds no usable data
refuses with the reason. A fit where the HEX does not follow the reference (R² < 0.9 on an axis)
is marked **NOT USABLE** and never replaces the active calibration.

Check any recording or bag before fitting (paths relative to the package folder):

```bash
roscd optoforce_sensor
rosrun optoforce_sensor hex_calibration_check.py calibration/<date_time>/s1.npz --plot check.png
rosrun optoforce_sensor hex_calibration_check.py my.bag --reference-topic /ft_sensor/wrench \
    --reference-limits 65,65,200,5,5,5 --reference-invalid-log "gauge saturated"
```

Refit offline (e.g. with the quadratic model), and make it active with `--activate`:

```bash
roscd optoforce_sensor
rosrun optoforce_sensor hex_calibration_fit.py calibration/<date_time>/s*.npz --model quadratic --activate
```

Every run keeps its own calibration. To go back to an older one:
`cp calibration/<date_time>/hex_calibration.yaml cfg/hex_calibration.yaml`.
