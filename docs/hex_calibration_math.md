# Calibrating the OnRobot HEX against a reference F/T sensor: the math

It is a sound and well-known way to calibrate a force sensor (a *build-up* or
*reference-in-series* calibration). Bolted in series, both sensors carry exactly the same load,
so the reference tells you what the HEX *should* read. Any 6-axis sensor that publishes
`geometry_msgs/WrenchStamped` can be the reference, as long as it is clearly more accurate than
the HEX: the HEX-E's specified cross-talk is up to 5 %, non-linearity and hysteresis up to 2 %.
The examples below use our reference, an ATI Gamma SI-65-5 (about 1 % of full scale). The limits
that matter in practice:

1. The reference's rated range (for the ATI: ±5 N·m torque) caps the sideways force you can apply
   on a tall handle.
2. Hysteresis can't be calibrated out.
3. The HEX's offset drift still needs a tare each time you use it. The reference can remove it live
   while both are mounted.

Contents: [1 frames](#1-frames-and-the-stack) · [2 wrench transform](#2-moving-a-wrench-between-frames) ·
[3 why the stack works](#3-why-the-two-readings-are-related-exactly) ·
[4 the transform of the adapter](#4-the-transform-of-the-adapter) ·
[5 calibration model](#5-calibration-model) · [6 splitting mount vs sensor error](#6-separating-the-mount-from-the-hex-error) ·
[7 data processing](#7-data-processing) · [8 limits and error budget](#8-viability-limits-and-error-budget) ·
[9 using it afterwards](#9-using-the-calibration)

---

## 1 Frames and the stack

```
          handle  (grip ~40-120 mm above the HEX top)
   ┌──────────────┐
   │   HEX-E      │  frame H: the HEX's own axes and measuring origin
   ├──────────────┤  ← HEX mounting face (its ISO 9409-1-50-4-M6 robot adapter)
   │   adapter    │  (ours: two printed plates, 24 mm, joined outside both sensors)
   ├──────────────┤  ← reference tool face = origin of frame A, z = 0, +z up the stack
   │  reference   │    (ours: ATI Gamma; "sensing reference frame origin at surface center
   │  F/T sensor  │    of tool adapter")
   └──────────────┘
        base (fixed)
```

* **A**, the reference frame: what the reference sensor publishes (`reference_topic` in
  `cfg/hex_calibration_params.yaml`), in N and N·m.
* **H**, the HEX frame: the axes and the point about which the HEX reports torques. Its pose in
  A is a rotation $R$ and position $p$ ("where is H, seen from A").
* A wrench is $w = [f;\ \tau] = [F_x\ F_y\ F_z\ T_x\ T_y\ T_z]^\top$.

## 2 Moving a wrench between frames

A force $f_H$ acting at a point $r$ (coordinates in H) produces the torque $\tau_H = r\times f_H$
about H's origin. Seen from A, the same point is at $p + Rr$ and the force is $Rf_H$, so

```math
f_A = R\,f_H,\qquad
\tau_A = (p + Rr)\times R f_H = R\,\tau_H + p\times(R f_H).
```

In matrix form, with $S(p)$ the cross-product matrix ($S(p)\,v = p\times v$):

```math
w_A = X(R,p)\,w_H,\qquad
X(R,p)=\begin{bmatrix} R & 0\\ S(p)\,R & R\end{bmatrix},\qquad
X^{-1}=\begin{bmatrix} R^\top & 0\\ -R^\top S(p) & R^\top\end{bmatrix}.
```

This is the usual "tool transform" of F/T sensors (ATI's C library uses the same `[R 0; S(d)R R]`).
In code: `ft_crosscal.wrench_transform(R, p)`.

## 3 Why the two readings are related exactly

Take the parts above the HEX's sensing element (the HEX top half and the handle) as one free
body, and the parts above the reference's sensing element as another. With the base not moving, the
static balance of each gives:

```math
w_{\text{HEX}} = w_{\text{hand}} + g_{\text{above HEX}}\quad(\text{in H}),\qquad
w_{\text{ref}} = X\,w_{\text{hand}} + g_{\text{above ref}}\quad(\text{in A}).
```

$g$ are the gravity wrenches of those parts, and both are constant. Eliminating the hand load:

```math
\boxed{\;w_{\text{ref}} = X(R,p)\;w_{\text{HEX,true}} + c\;}
\qquad c = g_{\text{above ref}} - X\,g_{\text{above HEX}} = \text{weight of the parts between the two sensors}.
```

This holds for **any** hand load, wherever and however you push, and for any reference sensor.
The constant $c$ (the adapter and the HEX's lower half, a few N) disappears in the offsets.
Nothing about the handle's shape or where you grip enters the equation; that only matters for
*which* loads you can produce (section 8).

Conditions for it to hold:

* The base doesn't move or tilt during a recording. Tilting by 1° changes $c$ by about 0.04 N.
* Accelerations are slow. With a fixed base, only the hand and handle move, and both are above
  the HEX, so they appear in both sensors equally. Inertia doesn't break the relation.
* Both sensors are sampled at the same instants. Their timestamps differ, which is handled in
  section 7.
* The parts are rigid. The HEX is compliant, which gives a second-order effect covered in
  section 8.

## 4 The transform of the adapter

**You don't need it**: section 6 measures the mount from the data. It is only an assembly check.
For our adapter (two 12 mm plates on an ATI Gamma), the HEX's mounting face is **24.0 mm** above
the reference origin. Plate A carries the ATI pattern, with the ATI dowel at 45°. Plate B carries
the ISO pattern for the HEX, with its dowel at 90°. So the HEX pattern is turned **45°** from the
ATI pattern, and the nominal transform is:

```math
R_{\text{nom}} = R_z(\psi),\qquad \psi = 45^\circ + \alpha_{\text{HEX}} - \alpha_{\text{ref}},\qquad
p_{\text{nom}} = \begin{bmatrix}0\\0\\h_{\text{adapter}} + z_{\text{HEX}}\end{bmatrix}
```

* $\alpha_{\text{ref}}$ = angle from the reference's dowel hole to its +x axis, and
  $\alpha_{\text{HEX}}$ = angle from the HEX adapter's dowel pin to the HEX's +x axis. Both are
  measured counter-clockwise looking down the stack. Read them off the axis marks on the two
  sensor bodies.
* $z_{\text{HEX}}$ = height of the point about which the HEX reports torques, above its mounting
  face. OnRobot doesn't publish it.
* **You don't need either number for the calibration.** Section 6 measures $R$ and $p$ from the
  data to a fraction of a degree and millimetre. Pass the nominal values
  (`--nominal-yaw-deg`, `--nominal-z-mm`) and the report prints how far the measured mount is
  from them, which is a good assembly check.
* If the HEX's z axis points *down* (into the sensor) instead, $R$ has an extra 180° turn about
  x. The data shows that too.

For a point higher up (e.g. the handle or the HEX top flange at height $h$ above the reference
origin), use $p = [0, 0, h]^\top$ with the same $R$: `frame: top` and `top_mm: h` in
`cfg/hex_calibration_params.yaml` (or `hex_calibration_fit.py --frame top --top-mm h`).

## 5 Calibration model

The HEX's raw reading $h$ (what `/optoforce_node/OptoForceWrench` publishes) is modelled as an
unknown linear map of the true wrench plus a slowly drifting offset. Combined with section 3, the
reference reading becomes:

```math
w_{\text{ref}}(t) = M\,h(t) \;+\; \underbrace{Q\,q\!\left(h(t)\right)}_{\text{optional}} \;+\; b(t) \;+\; \varepsilon(t)
```

| term | size | meaning |
|---|---|---|
| $M = X(R,p)\,K$ | 6×6 | everything linear: mount transform × HEX gain/cross-talk correction $K$ |
| $Q\,q(h)$ | 6×21 | `model: quadratic`: all products $h_ih_j$ (scaled). Covers non-linearity and the HEX tilting under load (section 8) |
| $b(t)$ | 6 per basis fn | offset drift of both sensors plus the constant weight $c$. `drift: linear` (default) is $b_0+b_1t$; `knots:60` is piecewise linear with a knot every 60 s |
| $\varepsilon$ | | noise, hysteresis, reference error |

Everything is linear in the unknowns, so it is solved by least squares in one step, with:

* columns scaled to unit RMS;
* a tiny ridge term ($10^{-7}$ for linear, $10^{-4}$ for quadratic), applied to the load terms only;
* 4 rounds of Huber re-weighting, so a slip or a knock doesn't pull the fit;
* several recordings at once, each with its own $b(t)$ and delay (and every stretch between two
  sensor outages gets its own $b(t)$, section 7).

Unknowns: 36 for the linear model, 162 for the quadratic one, plus 12 per recording for the
offsets. A 3–5 minute recording at 100 Hz has 18–30 k samples. That is plenty, *if* the loads
vary independently in all six directions (section 8).

## 6 Separating the mount from the HEX error

The fit gives $M$, the map from HEX readings to the reference frame. To get a calibration *of the HEX
in its own frame*, $M$ is split as $M = X(R,p)\,K$:

* **Rotation:** $R$ is the rotation closest to the force block $M_{ff}$, i.e. the polar
  decomposition $M_{ff} = R\,P$ with $P$ symmetric, computed by SVD.
* **Origin:** with $K$'s force-to-torque block small, $M_{\tau f} M_{ff}^{-1} \approx S(p)$, so
  $p = \operatorname{vee}\big(\tfrac12(S - S^\top)\big)$ with $S = M_{\tau f}M_{ff}^{-1}$.
* **HEX correction:** $K = X(R,p)^{-1} M$.

There is a real ambiguity, and this choice resolves it. A small rotation $\delta\theta$ of the
mount looks exactly like *antisymmetric* cross-talk in the HEX ($K_{ff} = I - S(\delta\theta)$). No
experiment can tell them apart: they are the same thing. Putting the antisymmetric part into
$R$ defines the HEX frame as "the axes the HEX actually measures along". That is the natural
frame. In the simulation tests, a mount with symmetric cross-talk is recovered to **0.003°** and
**< 0.01 mm**.

Two other cases are handled:

* **Left-handed HEX readings.** If the HEX reports one axis with the opposite sign
  ($\det M_{ff} < 0$), the report names the axis. The output is made right-handed, assuming the
  HEX's z points up the stack. Your `sensor_filter.py` flips Fz ("PAZI NA PREDZNAK SILE U Z"),
  and this tells you whether that is actually needed.
* **Diagnostics.** The report also prints how well the force and torque axes agree and how much
  torque coupling isn't explained by an origin shift.

The calibrated HEX output in any frame $F$ with pose $(R_F,p_F)$ in A is then

```math
w_F = X(R_F,p_F)^{-1}\big(M\,h + Q\,q(h)\big) - \text{tare}
     = K_F\,h + Q_F\,q(h) - \text{tare}.
```

The default $F = H$ (`frame: native`) makes the calibrated topic a corrected drop-in for the raw
one: same axes, same point.

## 7 Data processing

* **Delay.** The two drivers stamp differently (our ATI with its own sample clock, the HEX node
  with `ros::Time::now()` when a USB packet arrives, a few ms late). The delay $d$
  is found by minimising the fit residual over $d$ (grid search, then refinement). The simulation
  recovers a 4.30 ms delay as 4.31 ms. An error of $\delta d$ costs $\dot w\,\delta d$; at
  100 N/s and 0.5 ms that is 0.05 N.
* **Same filter on both.** Each signal is interpolated onto a 1 kHz grid, block-averaged to 100 Hz,
  then low-passed at 5 Hz with a symmetric (zero-phase) FIR. Both sensors get the *identical*
  filter, so filtering adds no relative delay. Samples near a gap in either stream are dropped.
* **Range checks.** Samples where the reference exceeds 95 % of its rated range
  (`reference_limits`) on any axis are dropped: beyond it its accuracy isn't specified. Seconds in
  which the reference driver logs invalid readings (`reference_invalid_log`, e.g. the ATI's
  "gauge saturated", which happens under combined loads below 100 % per axis) are dropped too, as
  are samples where the HEX exceeds its capacity.
* **Stream health.** A driver that keeps publishing its last packet after the sensor stopped
  sending produces identical messages: repeated copies are dropped before anything else (they
  carry later stamps than the real sample), so a frozen stream counts as a gap. Every outage of
  either sensor (frozen or silent for more than 0.25 s) is cut out, together with the 2 s before
  it, where a failing connection often already gives garbage. The stretches between outages get
  separate offsets, since a reconnected sensor may come back with another zero. A recording with
  less than 10 s left is skipped, and with nothing usable the fit refuses instead of fitting
  against a constant.
* **Sanity.** After the fit, $R^2$ per axis says how much of the reference's motion the HEX
  explains. Below 0.9 on an excited axis the report starts with a warning and the calibration is
  not made active.
* **Validation.** Each recording is cut into 5 time blocks. Each block is predicted by a
  calibration fitted *without* it, then tared. The RMS of that cross-validation error is the
  honest "accuracy on new data" number in the report. The report also shows the factory HEX error
  on the same data, so you see the improvement.

## 8 Viability: limits and error budget

**Load range.** Every axis of the reference must stay in its rated range at once. A sideways
force $F$ on a grip at height $z$ above the reference origin gives $T_{xy} = F z$ there. Example:
our ATI SI-65-5 is rated ±65 N (Fx, Fy), ±200 N (Fz) and ±5 N·m on all torques; with the 24 mm
adapter and a HEX height of about 50 mm:

| grip above the HEX top | height above the reference origin | max sideways push (ATI at 5 N·m) |
|---|---|---|
| 40 mm | 114 mm | 44 N |
| 80 mm | 154 mm | 32 N |
| 120 mm | 194 mm | 26 N |

So the calibrated range is roughly ±25–45 N sideways, ±100 N or more along z, ±2–3 N·m tilting and
±2 N·m twisting. That covers what a hand on a joystick produces. It is a small part of the HEX's
200 N / 10 N·m capacity, and the calibration should only be trusted inside the range it saw (the
report lists it). Keep the handle short. The dashboard warns above 90 % of the reference's range.

**Excitation.** A push at one point always gives torque = lever × force, so $F_x$ and $T_y$ (and
$F_y$, $T_x$) move together, and $M$ can't separate them. You need:

* pushes at two or more grip heights (low and high on the handle);
* pure torques: twisting the handle for $T_z$, and rocking it forward/back with little net push
  for $T_x$, $T_y$.

The dashboard shows a live *condition number* of the excitation and tells you what is missing;
aim for below 10. The simulated protocol reaches 4–5.

**Error budget** (1σ unless noted):

| source | size | handled by |
|---|---|---|
| HEX-E cross-talk (spec < 5 % FS) | up to several N, 0.1–0.5 N·m | **removed**: off-diagonal of $K$ |
| HEX gain error | a few % | **removed**: diagonal of $K$ |
| HEX non-linearity (< 2 % FS) | up to ~1 N in range | partly removed by `model: quadratic` |
| HEX tilt under load: 0.25°/N·m (Txy), 0.77°/N·m (Tz) for HEX-E | 2.5 N·m tilts 0.6°, so 80 N axial shows as 0.9 N sideways | `model: quadratic` (simulation: 0.028 → 0.011 N) |
| HEX hysteresis (< 2 % on Fz) | up to ~1 N | **not removable** by any memoryless model; sets the floor |
| HEX noise | 0.035 N (Fxy), 0.15 N (Fz), 0.002 N·m | averaged by the 5 Hz low-pass in the fit |
| reference error (our ATI: ~1 % FS, exact values on its calibration certificate) | ATI: ≤0.65 N Fxy, ≤2 N Fz, ≤0.05 N·m | inherited, but 5–10× smaller than the HEX errors removed |
| reference force error moved to the HEX origin (~49 mm lever) | ATI: ≤0.032 N·m | inherited |
| printed PETG adapter (E ≈ 2 GPa, 100 % infill) bending | tilt well under 0.1° at 5 N·m (beam estimate), about 10× less than the HEX itself | same as HEX tilt: `model: quadratic`; re-tighten after a day (PETG creeps) |
| reference temperature gain error (ATI: 0.1 % per ±5 °C) | negligible | keep room temperature steady |
| delay, after estimation | < 0.5 ms, i.e. < 0.05 N at 100 N/s | estimated per recording |
| base tilt / moving | 0.04 N per degree | don't touch the base |
| HEX offset drift | ~1 N over minutes (your experience) | $b(t)$ in the fit; tare (or reference tracking) in use |

**Expected result:** the HEX goes from a few percent error with axes mixed together to about
**1–2 % of reading**, limited by its own hysteresis and repeatability. The report's
cross-validation column tells you the real number for your sensor.

## 9 Using the calibration

`hex_calibrated_node.py` publishes $w_F = K_F h\ (+Q_F q(h)) - \text{tare}$:

* **Tare:** averaged over `~zero_seconds` at start and on `~zero`, with the sensor unloaded.
  This removes the drifting offset, which the calibration can't predict.
* **With the reference still underneath** (`reference_topic:=<its topic> drift_tau:=10` in
  `hex_calibrated.launch`): the tare is pulled continuously towards the reference,

  ```math
  \text{tare}_{k+1} = \text{tare}_k + \frac{\Delta t}{\tau}\Big(\big(K_F h_k - \text{tare}_k\big) - \big(X_F^{-1} w_{\text{ref},k} - \text{tare}_{\text{ref}}\big)\Big).
  ```

  This is a complementary filter: below $1/(2\pi\tau)$ (0.016 Hz for 10 s) the output follows
  the reference, and above it the HEX. Drift is gone, and the HEX's own dynamics stay. In the
  simulation, a deliberately large 3–8 N drift shrinks from 6.8 N to 0.26 N.
* **Re-check now and then:** a 3-minute recording, then refit. Gains and cross-talk should stay
  put. If they don't (temperature, ageing, a knock), the report shows it.
