# AHAB mass properties — measurement procedure

Purpose: measure the real aircraft's mass, CG and inertia (and the tilt-wing geometry) for the Gazebo model
(`Tools/simulation/ahab_model/`). Results go into `Tools/simulation/ahab_model/mass_properties.yaml`; run
`python3 Tools/simulation/ahab_model/tools/mass_properties.py` to check and convert them.

Measure **twice**: the **whole aircraft** and the **wing assembly alone**. The hull's properties are
computed by subtraction. The tilt mechanism stays on the hull, so it is counted as hull.

## 0. Configuration and datum

- Flight configuration: flight battery installed and strapped where it flies, all avionics, props on, hatches closed.
  Wing at **level** (tilt 0°).
- **Wing assembly** = everything that comes off with the wing: wing, nacelles, motors, props, ESCs on the wing, ailerons
  and their servos. Whatever stays on the hull when the wing is off (tilt drive, gearbox, encoder) is hull.
- **Datum** — measure every position from this datum:
  - **x** = distance aft of the **nose tip**
  - **y** = to the right (looking forward)
  - **z** = up from the **lowest point of the hull keel**, with the aircraft level (wing chord line / hull deck level)
- Units: metres, kilograms, seconds.

Equipment: kitchen/postal scale (±1 g) and two bathroom/luggage scales or two kitchen scales for CG, tape measure,
square, plumb bob (or weighted string), phone camera, stopwatch (phone is fine), 2 lengths of thin non-stretch line
(braided fishing line), a rigid hanging point ≥ 1.5 m above the work, a light hanging cradle (board/rod frame) if the
aircraft can't be tied directly.

## 1. Mass

| Item | Mass [kg] |
|---|---|
| Whole aircraft (flight config) | |
| Wing assembly alone | |
| Spare aileron (if available) | |
| Spare ruddervator (if available) | |
| Spare prop | |

If a spare part isn't available, leave it blank; small default estimates are used for those child links.

## 2. CG

### x (longitudinal)
Support the aircraft level on two scales: one under the nose region at x = x₁, one under the tail region at x = x₂
(measure both support points from the nose tip). Read W₁, W₂.
x_cg = (W₁·x₁ + W₂·x₂) / (W₁ + W₂). Check W₁ + W₂ ≈ the mass from §1.

### z (vertical) — plumb-line method
Hang the aircraft from point A (e.g. the nose) and let it settle. Hang a plumb line from the same point, then photograph
the side square-on, with a tape along the hull for scale. Repeat from point B, as far from A as you can (e.g. a tail
point). The CG is where the two plumb lines cross. Read its x (cross-check §2 x) and z from the photos.

### y (lateral)
Expect ≈ 0. Balance the aircraft on its keel line (or on two scales placed left and right) and record any offset.

**Repeat §2 for the wing assembly alone.** Give its CG in the aircraft datum **as installed at level**: measure it on
the wing relative to the pivot/spar, then add the pivot location from §4.

## 3. Inertia — bifilar pendulum

Hang the body from **two parallel vertical lines** of equal length **L**, a distance **d** apart, placed
**symmetrically about the CG**. Then the vertical axis midway between the lines passes through the CG. Twist it
gently (< 10°) about that vertical axis and release. It must rotate, not swing. Time **20 full oscillations**,
three times. Ideal values: L ≈ 1.5–2 m, d ≈ 0.4–0.8 m (larger d gives a slower, easier-to-time swing).

I = m · g · d² · T² / (16 · π² · L)  (T = one full period; m includes any cradle)

| Axis measured | How the aircraft hangs | Line attachment points |
|---|---|---|
| **Yaw (Izz)** | level, as in flight | left/right (or fore/aft) of CG, at equal distance d/2 |
| **Roll (Ixx)** | nose **up** (x vertical) | either side of the CG at y = ±d/2 |
| **Pitch (Iyy)** | right wingtip **up** (y vertical) | fore/aft of the CG at x = x_cg ± d/2 |

- If you use a **cradle**: weigh it, and time it alone on the same lines (same L, d). The script subtracts it.
- If the rotation axis is off the CG by a known distance **e** (e.g. you couldn't place the lines symmetrically),
  record e. The script applies the parallel-axis correction (I_cg = I_axis − m·e²).
- Large flat surfaces drag a little air with them, which reads slightly high. Swing slowly, and take the median of
  the three timings.

**Repeat §3 for the wing assembly alone** (3 axes, in the wing's installed orientation: "yaw" = wing flat, etc.).

Record per test: L, d, cradle mass and its 3 timings (if a cradle was used), axis offset e, and the three 20-swing
times.

## 4. Tilt geometry

- **Pivot axis**: x and z (datum) of the tilt shaft centre, and confirm it is parallel to y.
- **Wing seat**: with the wing level, the wing's own CG relative to the pivot (from §2 on the wing alone).

## 5. Surface throws (for the flight-log identification)

With the aircraft powered on and in Manual mode, use QGC Actuators → test sliders, or full stick.
Use an inclinometer phone app or a protractor on the surface chord.

| Surface | Deflection at PWM min [deg] | at center | at PWM max [deg] | which way is positive (TE up/down) |
|---|---|---|---|---|
| Left aileron | | | | |
| Right aileron | | | | |
| Left ruddervator | | | | |
| Right ruddervator | | | | |

## 6. Tilt drive (optional)

- **Full-command slew rate** (deg/s): use `wing_tilt` console holds, or `wing_tilt_status` from a log. The bench
  already showed 60–70 °/s.
- **Stall torque** (N·m), if you can measure it: a spring scale at a known radius on the wing, at full command.

## 7. Enter and check

Fill `Tools/simulation/ahab_model/mass_properties.yaml`, then run:

```bash
python3 Tools/simulation/ahab_model/tools/mass_properties.py
```

The script prints the hull and wing link properties and the recomposed aircraft totals, and flags anything
inconsistent. Check these:
- the scale masses add up
- the CG from the scales matches the plumb lines
- the inertias are positive and satisfy the triangle inequality (Ixx + Iyy ≥ Izz etc.)
- the hull properties come out physical after subtraction
