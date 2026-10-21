---
name: amb-ten-phase-nine-leg-topology-design
description: Design, build, or review the power topology and winding-current control loop of an active magnetic bearing (AMB) drive that runs TEN windings from NINE half-bridge legs by hard-wiring two "star" summing nodes (ten-phase-nine-leg topology) - four radial DOF pairs split into two groups of two pairs, each group's four winding ends joined at one star node so that KCL structurally enforces i_a1+i_c1 = i_a2+i_c2 = 2*i_bias, plus one dedicated leg for the axial pair inserted between any two existing leg midpoints. Covers the tree property of the winding/leg graph (11 nodes, 10 windings, so ten winding-voltage commands invert uniquely into nine leg voltages), the node-voltage back-substitution that produces the leg references, min-max common-mode injection, bias+differential per-winding PI control, the physical range abs(delta) <= 2*i_bias, the bus-voltage design rule U_dc >= [6*R*i_bias + k*(R*delta_max + L*gamma_max) + V_ce]/(1 - 2*Td/Ts) with k = 1 for equal control currents and k = 2 for the worst admissible distribution, and the relocation of the extra leg (bias-free versus bias-carrying placements, and the dead zone at exactly zero leg current). Use when modelling or validating a 9-leg / 10-winding AMB drive; when deciding which effects a topology-verification model must keep versus may safely drop; when moving the axial-pair leg to another midpoint; or when reviewing why legs 4-6 carry no bias current, why a simulation aborts with consecutive zero crossings, or why the bus voltage rather than the bias current limits the control current.
metadata:
  author: AMB ten-phase-nine-leg topology workspace
  version: "1.0"
---

# AMB Ten-Phase Nine-Leg Topology: Minimal Modeling and Verification

A 5-DOF AMB drives 8 radial + 2 axial windings (10 coils). Each coil carries a **unidirectional**
current made of a **bias** term plus a differential control term, and the usable control variable is the
difference of a winding pair, `i_a - i_c`. Independent H-bridges need two legs per winding (20 legs); a
half 3-legs cell needs 1.5 legs per winding (15 legs). The **ten-phase-nine-leg topology** does it
with **nine legs for ten windings (0.9 legs per winding)** by exploiting the bias constraint itself:
because the two windings of a DOF pair always satisfy `i_a + i_c = 2*i_bias`, the two ends of one pair may
be hard-wired to the two ends of another pair - a **star node** - and KCL then *structurally* enforces the
constraint instead of the controller. Two such star nodes carry the four radial DOF pairs, and the axial
pair gets **one dedicated extra leg** inserted between any two existing leg midpoints.

The winding/leg graph has 11 nodes (9 leg midpoints + 2 star nodes) and 10 edges (the windings), i.e. it is
a **tree**. That single fact explains the whole control law: the ten winding-voltage commands determine the
eleven node potentials uniquely up to one common mode, so the nine leg references are obtained by
**back-substituting node voltages along the tree** - one line of MATLAB per leg.

This skill gives the minimum Simulink/Simscape model that verifies such a topology and its current loop,
states explicitly **what may be ignored** while the model is used for topology verification only, names the
simulation mechanics that may **not** be ignored, and lists the checks that prove the topology is modelled
correctly.

**Verified reference implementation**: a 10-winding, 5-DOF model bundled at
`model/Ten_phase_nine_leg_with_RL_load.slx` (md5 `0e5bbb66a6f61da9e0210f63aa7ccffd`, byte-identical to the
engineer's model) together with its parameters at `model/parameters_for_AMB_topology.m`
(md5 `a90c6e390a9c1e14bf015396b2d6054d`, the same file used by the sibling topology skills), verified with
**MATLAB 25.2 (R2025b), Simulink 25.2, Simscape 25.2 and Simscape Electrical 25.2 (Linux `glnxa64`)**.
Two derived models show the extra leg relocated: `model/Ten_phase_nine_leg_relocated_L3_L7.slx`
(md5 `1f051832b3c2760930ffcd561aadd9c4`) and `model/Ten_phase_nine_leg_relocated_L2_L8.slx`
(md5 `36e3503b18cf1c1abf8b819d801ccf7f`).

## When to Use

- Building or reviewing an AMB winding-drive power stage that drives **ten windings from nine legs** by
  hard-wiring summing nodes, and its per-winding current loop, in Simulink/Simscape.
- Deciding how the star nodes are formed, which legs the extra axial-pair leg may use, and what each
  choice does to device currents and to the dead zone.
- Working out the two boundaries that matter: the **physical** limit `abs(i_ref) <= 2*i_bias` and the
  **bus-voltage** limit that follows from the modulator's duty range.
- Deciding which effects a topology-verification model must keep and which it may safely drop.
- Diagnosing a 9-leg/10-winding model: legs 4-6 that carry only about 1 A, a simulation that aborts on zero
  crossings, duty cycles that leave [0,1], leg voltage commands far larger than `R*i/U_dc`, or a control
  current that does not follow its reference after the axial pair was rewired.

## When NOT to Use

- Engineering-grade design: loss and thermal budgets, EMI, protection, bus capacitor sizing, device
  selection, gate-drive design, digital controller implementation. Those need the effects this skill
  deliberately drops.
- Position/force loop tuning or rotor dynamics - a topology-verification model has no motion model.
- Closed-loop bandwidth calibration - that is a controller-design task, not a topology-verification task.
- Building the Simulink model mechanically (block insertion, wiring syntax) - use the
  `building-simulink-models` skill for the mechanics; this skill supplies the design content.

---

## 0. First principle: a topology-verification model keeps only the core

A topology-verification model has exactly one job: answer **"does this wiring let nine legs control ten
winding currents independently, and does the current loop converge?"** Keep only what that question needs:

1. A stiff DC bus - one ideal voltage source, one electrical reference. No bus capacitance, no source
   impedance.
2. **Nine half-bridge legs** on that bus, each driven by two complementary gate signals.
3. **Ten windings**, each a series R-L branch with a current sensor.
4. The **two star nodes**, hard-wired. They are the topology; they are not optional.
5. Reference generation `i_a_ref = i_bias + 0.5*i_ref`, `i_c_ref = i_bias - 0.5*i_ref`.
6. One **per-winding per-unit PI** current controller with an output clamp.
7. **One symmetric triangular carrier plus min-max common-mode injection**, driving the leg references that
   the node-voltage back-substitution produced from the winding commands.

Everything else is deferred. Do not add device `Vf`/`Ron`, dead time, switching or conduction loss, thermal
networks, magnetic saturation, mutual coupling, iron or eddy loss, back-EMF, motion equations, force
models, position loops, bus impedance, protection and fault logic, quantisation, computation delay,
gate-drive level selection, or saturation-limit selection. In a topology-verification model they only slow
the simulation down and obscure the switching behaviour that is being verified.

**Rule of thumb**: if an effect does not change *which devices conduct*, *in which direction*, or *the
average voltage applied to the winding*, it does not belong in a topology-verification model.

Two things are worth stating because they are easy to get wrong:

- A topology-verification model is **not** a reduced-order model of the drive. It is the switching power
  stage plus the current loop, with everything that is not a switching device, a winding or a controller
  removed on purpose.
- A constant `i_ref` is enough to verify the topology. The question "can this topology drive the control
  current to the value I ask for?" is answered by a steady operating point; a sweep of `i_ref` maps the
  range. Dynamic profiles and reversal tests belong to the later controller-design stage.

---

## 1. Topology skeleton (build recipe)

### 1.1 The winding/leg graph

Number the legs L1..L9 and put the ten windings on the graph as edges. In the verified model the edges are

```

w1 : L1 - X1      w5 : L4 - L5      w9  : X2 - L8
w2 : L2 - X1      w6 : L5 - L6      w10 : X2 - L9
w3 : X1 - L3      w7 : L6 - X2
w4 : X1 - L4      w8 : L7 - X2
```

where X1, X2 are the two hard-wired star nodes. **11 nodes, 10 edges, connected, acyclic - a tree.** The
pairing into DOFs is `DOF1 = (w1,w2)`, `DOF2 = (w3,w4)`, `DOF3 = (w5,w6)` (axial),
`DOF4 = (w7,w8)`, `DOF5 = (w9,w10)`.

Consequences of the tree property:

- Ten winding voltages determine the eleven node potentials **uniquely up to one common mode**, so nine leg
  voltages always exist and are unique. That is why nine legs can serve ten windings at all.
- The number of independent legs (9) equals the number of *non-star* nodes (11 - 2 = 9). Any placement that
  keeps the graph a connected tree is admissible - this is the basis of section 6.

### 1.2 The star nodes and the KCL law

At X1 the winding currents satisfy, by KCL alone,

```

i_a1 + i_c1  =  i_a2 + i_c2        (node X1)
i_a4 + i_c4  =  i_a5 + i_c5        (node X2)
```

These identities are **enforced by the wiring, not by the controller**: the measured residual in the
verified model is 5e-15 A (machine precision). Two design consequences:

- The two DOFs sharing a star node **must be commanded the same bias current**, otherwise one of the two
  pairs is forced away from its reference. DOF1/DOF2 must share `i_bias`; DOF4/DOF5 must share it too.
  DOF3 (the axial pair) is not on a star node and is free.
- The sum of the nine leg currents is identically zero (measured 1e-14 A):

```

sum(I_leg) = (i_a1+i_c1) - (i_a2+i_c2) + (i_a4+i_c4) - (i_a5+i_c5) + (i_a3-i_a3) + (i_c3-i_c3) = 0
```

  Hence the **bias current does not flow through the DC bus**: the bus supplies only the losses (measured
  1.376 A) while the nine legs carry about 33 A of circulating current between them. That is the property
  that makes the topology attractive: the source, the bus capacitor and the cable see the control power, not
  the bias power.

### 1.3 What each leg carries

With `delta_d = i_ad - i_cd` and `i_bias` the bias, KCL at each leg midpoint gives

| Leg | Expression | Verified mean | Comment |
|---|---|---|---|
| L1 | `i_a1` | +5.495 A | bias + delta/2 |
| L2 | `i_c1` | +4.496 A | bias - delta/2 |
| L3 | `-i_a2` | -5.494 A | -(bias + delta/2) |
| **L4** | `i_a3 - i_c2` | **+0.997 A** | `= (delta3 + delta2)/2`, **no bias** |
| **L5** | `i_c3 - i_a3` | **-0.997 A** | `= -delta3`, **no bias** |
| **L6** | `i_a4 - i_c3` | **+0.996 A** | `= (delta4 + delta3)/2`, **no bias** |
| L7 | `i_c4` | +4.494 A | bias - delta/2 |
| L8 | `-i_a5` | -5.493 A | -(bias + delta/2) |
| L9 | `-i_c5` | -4.495 A | -(bias - delta/2) |

The three legs that serve the axial pair carry **control current only**, so in the normal operating region
(`abs(delta) << 2*i_bias`) they can be rated far below the other six. This is a property of the
*placement*, not of the topology - see section 6.

### 1.4 Leg count and device count

| Topology | Legs for 10 windings | Legs per winding | Bias through the bus |
|---|---|---|---|
| Independent H-bridges | 20 | 2.0 | yes |
| Half 3-legs cells | 15 | 1.5 | partly |
| Series winding chain (N+1) | 11 | 1.1 | no |
| **Ten-phase nine-leg (this skill)** | **9** | **0.9** | **no** |
---

## 2. What you may ignore (and when it stops being safe)

This is the heart of topology design: most non-ideal effects, and essentially all engineering practice, can
be dropped because they do not change **which devices conduct, in which direction, or the average voltage
applied to the winding**. Add them back only when the question changes.

| Item | Why it is safe to drop in a topology model | When it stops being safe |
|---|---|---|
| Device `Vf`, `Ron`, `Goff` | Shifts the operating duty, not the switching states or the current directions | Loss/thermal budget, efficiency, device selection |
| Dead time | Correctly sized dead time does not change which devices conduct in steady state | It adds a fixed duty overhead (below) and a distortion that depends on the sign of the leg current; only a real gate-drive design can size it |
| Switching transients, `di/dt`, reverse recovery, diode capacitance | Do not change the average-current behaviour | EMI, overvoltage, device stress |
| Thermal network, junction temperature | Far slower than the current loop | Any temperature-dependent study |
| Magnetic saturation, mutual coupling, iron/eddy loss | Change `L` and add cross-coupling; not needed to verify switching states | Position dependence, force linearity, high-frequency loss |
| Back-EMF, motion equation, force model, position loop | A motionless winding is a pure R-L branch | Any study of stiffness, damping or rotor dynamics |
| Bus capacitance, source impedance | An ideal source fixes the bus voltage | Regenerating-energy handling, bus sizing, DC-link resonance |
| Protection, fault logic, EMI filtering | Not part of the topology function | Safety or compliance work. **Note: each star node is a hard-wired joint of four winding ends, so it is a single point of failure - and that IS a topology property.** |
| Feedback filter bandwidth and damping | Sets phase margin, not the switching states | Not ignorable if closed-loop bandwidth is the question |
| Gate-drive level | Any non-zero level turns an ideal switch on | Real gate-drive design, isolation, `dV/dt` |
| Controller and integrator clamps | Only bound large-signal excursions | Fault ride-through, saturation recovery |
| Digital implementation detail | Second-order for switching-state verification | When the loop bandwidth itself matters |
| **Differential reference factor (0.5)** | Definition of the control current | Never - it *is* the control law |
| **Modulator constants (0.5, 0.5)** | Part of the per-unit modulation law | Never - they define the duty mapping |
| **Where the extra axial leg is connected** | Any admissible midpoint placement is the *same* topology; only the leg-voltage back-substitution changes | Never for the topology claim - but the placement decides which legs carry bias current and where the dead zone sits (section 6) |

**The one place where two "droppable" effects come back.** Dead time and device drops do not change the
topology result, but together they add a **fixed duty-cycle overhead**
`sigma0 = 2*Td/Ts + (device-drop term)`, measured at **0.0534** in the verified model (about 10.7 V on a
200 V bus). It appears as a constant in the bus-voltage design rule of section 5.2, and it is directly
measurable: with `i_ref = 0` the modulation spread is 0.2033 with the delivered 1 us dead time and 0.1635
with the dead time removed - a difference of 0.0399, against the theoretical `2*Td/Ts = 0.0400`. You do not
need to model it in order to verify the topology; you do need to allow about 5 % when you predict the
modulation boundary quantitatively. The effect is also visible in the winding voltage commands: for winding
a1 the commanded value is about 8.4 V while the winding only needs `R*i = 5.5 V`, and the integrators absorb
the difference.

---

## 3. What you must NOT ignore: simulation mechanics

### 3.1 Current-feedback sampling

The feedback path is the one place where a modelling shortcut silently biases the result. Sample the
feedback at a defined rate, synchronously with the PWM:

- `Rate Transition` with `OutPortSampleTimeOpt = Specify` and `OutPortSampleTime = SW_Ts`,
  `Integrity = on`, `Deterministic = on`; or a `Zero-Order Hold` at `SW_Ts`.
- Keep a `Memory` block **after** the sampler: a `Rate Transition` has direct feedthrough, so the sampler
  alone does not break the `feedback -> controller -> PWM -> leg -> current -> feedback` algebraic loop.
- A second-order low-pass in the feedback path (`fn` around `4*f_c`, `zeta = 0.707`) is conventional and
  harmless for topology verification, but it costs phase margin. Put it **after** the sampler. The verified
  model uses 6 kHz / 0.707 against a 1500 Hz design bandwidth.

### 3.2 Zero-crossing detection must stay on

Ideal switches plus zero-crossing detection are what resolve the switching edges accurately. Keep
`ZeroCrossControl = UseLocalSettings` and every block's `ZeroCross = on`. Adaptive zero crossing is useful
for getting past a hard start-up, but it must not be used for quantitative results. The verified model uses
`MaxStep = 1e-6` while the narrowest conduction interval is far shorter; the variable-step solver brackets
the comparator zero crossings and the pulse widths come out right.

### 3.3 Solver settings of the verified model

| Setting | Value |
|---|---|
| Solver | `VariableStepAuto` |
| MaxStep / RelTol | `1e-6` / `1e-4` |
| StopTime | 0.05 s as delivered; 20 ms is enough for steady-state topology verification, 6 ms for a single operating point |
| SimulationMode | `accelerator` |
| ZeroCrossControl / IgnoredZcDiagnostic | `UseLocalSettings` / `warning` |
| Solver Configuration | `UseLocalSolver = off` |

### 3.4 Measurement hygiene

Log the ten winding currents, the ten winding voltage commands and the nine duties with `To Workspace`
blocks at a fixed sample time (1 us in the bundled scripts) rather than reading a variable-step time vector,
and evaluate steady-state metrics over a window at the end of the run. Ten currents are enough: the nine leg
currents follow from them by the KCL table of section 1.3, which the scripts do and then verify against
Tellegen's identity.

---

## 4. Control and modulation law

### 4.1 Differential (bias) reference generation

```

i_a_ref = i_bias + 0.5 * i_ref
i_c_ref = i_bias - 0.5 * i_ref
```

The `0.5` is the definition of the control current, not a tuning knob: it makes
`i_a_ref - i_c_ref = i_ref` while `i_a_ref + i_c_ref = 2*i_bias` stays constant, which is what the star
nodes rely on. Start-up staggering (a delayed reference on some DOFs) is a test convenience, not part of
the topology.

### 4.2 Per-winding per-unit PI

```

err = i_ref - LPF2(i_fbk)                      % feedback sampled at SW_Ts, one sample of delay
u   = K_p*err + integral(K_i*err)              % per-unit output, saturate to [-1, +1]
K_p = 2*pi*f_c*L / U_dc
K_i = 2*pi*f_c*R / U_dc
```

With this normalization `K_p*U_dc = 2*pi*f_c*L` is independent of the bus voltage, so the loop bandwidth
does not change when `U_dc` is swept. The PI zero `K_i/K_p = R/L` cancels the winding pole exactly.
Clamp the *integrator state* and freeze integration while the error pushes further into the clamp (classic
clamping anti-windup). The verified model uses `f_c = 1500 Hz`, `R = 1 ohm`, `L = 500 uH`,
`U_dc = 200 V` -> `K_p = 0.0235619`, `K_i = 47.1239`.

### 4.3 Leg-voltage back-substitution (the topology-specific part)

Given the ten winding voltage commands `u_w1..u_w10` (per unit of `U_dc`), walk the tree from a
common-mode reference node - the verified model uses `ub(5) = 0` - and write one line per leg:

```

function ubridge = u_w2b(uw1, uw2, uw3, uw4, uw5, uw6, uw7, uw8, uw9, uw10)
ubias = 0;
ubridge = [0 0 0 0 0 0 0 0 0];
ubridge(5) = ubias;
ubridge(4) = ubridge(5) + uw5;          % w5 runs L4 -> L5
ubridge(3) = ubridge(4) + uw4 - uw3;    % vX1 = V4 + u_w4 = V3 + u_w3
ubridge(2) = ubridge(4) + uw4 + uw2;    % w2 runs L2 -> X1
ubridge(1) = ubridge(4) + uw4 + uw1;    % w1 runs L1 -> X1
ubridge(6) = ubridge(5) - uw6;          % w6 runs L5 -> L6
ubridge(7) = ubridge(6) - uw7 + uw8;    % vX2 = V6 - u_w7 = V7 - u_w8
ubridge(8) = ubridge(6) - uw7 - uw9;    % w9 runs X2 -> L8
ubridge(9) = ubridge(6) - uw7 - uw10;   % w10 runs X2 -> L9
end
```

Each line is one edge of the tree: `V_child = V_parent +/- u_w`, with the sign fixed by the winding's
orientation (the winding voltage is defined positive from its L terminal to its R terminal). **Recipe for
any placement**: list the ten edges, pick any node as the common-mode reference, walk the tree outward and
write `ubridge(k)` for each leg.

The nine outputs are the leg voltage commands **before** common-mode injection. Two invariants make this
easy to check: `ub(i) - ub(j)` must equal the winding voltage of the edge(s) between them, and the mapping
is linear, `ub = M*u` - so the leg currents are the solution of `M'*I = i`, which reproduces the KCL table
of section 1.3 exactly.

### 4.4 Min-max common-mode injection

One carrier, one injection term:

```

duty = ubridge - 0.5*(max(ubridge) + min(ubridge)) + 0.5      % then compare with a triangle 0..1
```

The injection centres the nine duties on 0.5, so the modulator is linear **iff the spread satisfies**

```

S := max(ubridge) - min(ubridge) <= 1
```

That single inequality is the whole modulation constraint; it does not depend on the bias or on the control
currents directly. The verified model runs at `S = 0.2089` with a duty range of 0.2903..0.7097 over the
whole run (0.3956..0.6044 in steady state), i.e. with ample headroom.

---

## 5. Boundaries: physical, modulation, numerical

### 5.1 Physical boundary: `abs(i_ref) <= 2*i_bias`

Because the windings carry unidirectional current, `i_a = i_bias + delta/2 >= 0` gives
`-2*i_bias <= delta <= +2*i_bias`. With `i_bias = 5 A` the usable control-current range is +/-10 A, and at
the extreme one winding of each pair sits at zero current - which is a numerically sensitive point
(section 5.3).

### 5.2 Modulation boundary: bus voltage, bias current and winding resistance

Substituting the steady-state winding voltages `u_wk = R*i_k/U_dc` into the tree map, the worst spread is
always the pair `ub1 - ub8`:

```

ub1 - ub8 = [ 6*R*i_bias + R*(d1 - d2 + d4 + d5)/2 + L*(g1 - g2 + g4 + g5)/2 ] / U_dc
```

with `d` the control currents and `g = d(delta)/dt`. Maximizing over all admissible operating points
(checked exhaustively over all 81 row pairs of `M`) gives

```

S = [ 6*R*i_bias + k*( R*delta_max + L*gamma_max ) ] / U_dc        k = 1 or 2
```

- **k = 1** - all five control currents equal (`delta_d = delta_max`, the symmetric case).
- **k = 2** - the worst admissible distribution `delta = [+, -, *, +, +]*delta_max` (the third entry is free).

Adding dead time (two edges of `Td` per switching period) and the device drops as a constant `V_ce`:

```

S_cmd = [ 6*R*i_bias + k*(R*delta_max + L*gamma_max) + V_ce ] / U_dc + 2*Td/Ts  <=  1

U_dc,min    = [ 6*R*i_bias + k*(R*delta_max + L*gamma_max) + V_ce ] / (1 - 2*Td/Ts)
delta_max(U)= [ U_dc*(1 - 2*Td/Ts) - 6*R*i_bias - k*L*gamma_max - V_ce ] / (k*R)
```

Read the three terms physically:

- `6*R*i_bias` is the **price of circulating the bias current**. It does not flow through the DC bus, but it
  must be driven through three winding resistances on each side of the two star nodes, so six times
  `R*i_bias` of bus voltage is committed **before any control current is produced** (30 V in the verified
  model, i.e. about half of the boundary).
- `k*R*delta_max` is the resistive drop of the control current itself, doubled for the worst distribution.
- `k*L*gamma_max` is what a fast control-current slew costs. It is the only term that a higher switching
  frequency cannot reduce.

Measured against the verified model (`R = 1 ohm`, `i_bias = 5 A`, `2*Td/Ts = 0.04`, `V_ce ~= 1.9 V`):

| Case | `S_ideal` | predicted `U_dc,min` | measured boundary |
|---|---|---|---|
| symmetric `delta = 10 A` | 40 V | 43.6 V | about 44.5 V |
| `delta = [8 -8 10 8 8]` | 46 V | 49.9 V | about 52.5 V |
| `delta = [9.5 -9.5 5 9.5 9.5]` | 49 V | 53.0 V | about 54 V |
| `delta = 0` (bias circulation only) | 30 V | 33.2 V | about 33.5 V |

The same expression predicts the *spread* at every operating point to better than 1 %: at 200 V and
`delta = 10 A` it gives 0.2495 against 0.2588 measured; at 50 V it gives 0.878 against 0.880. Fitted over a
sweep, `dS/d(delta) = 0.00555 /A` against the theoretical `R/U_dc = 0.0050 /A`, and the constant overhead is
0.0534 pu = 10.7 V.

**Design table** (`U_dc,min` in volts, worst-case k = 2, `delta_max = 2*i_bias`, `gamma = 0`):

| R [ohm] \ i_bias [A] | 2 | 3 | 5 | 8 | 10 |
|---|---|---|---|---|---|
| 0.2 | 6.1 | 8.2 | 12.4 | 18.6 | 22.8 |
| 0.5 | 12.4 | 17.6 | 28.0 | 43.6 | 54.1 |
| **1.0** | 22.8 | 33.2 | **54.1** | 85.3 | 106.1 |
| 2.0 | 43.6 | 64.5 | 106.1 | 168.6 | 210.3 |
| 3.0 | 64.5 | 95.7 | 158.2 | 252.0 | 314.5 |

For the symmetric case (k = 1) the resistance-related terms halve, i.e. multiply the table by about 0.8.

**Consequence**: the required bus voltage is proportional to `R*i_bias`. The verified model's 200 V bus has
roughly 4x margin, and the binding limit on the control current is the *physical* `i_a >= 0` constraint.
But a winding with `R = 2 ohm` would need 106 V for the same current range, and `R = 3 ohm` would need
158 V. "Raise the bus" and "lower the bias" are then the only two options - and the second one costs load
capacity. That is the price of circulating the bias inside the machine.

### 5.3 Numerical sensitive region: a leg at exactly zero current

Ideal switches plus dead time are ill-conditioned when a leg current is exactly zero: the conducting device
is undetermined, the zero-crossing detector retriggers, and Simulink aborts with "1000 consecutive zero
crossings". Two operating points of this topology sit exactly there:

- `i_ref = 0`: legs 4, 5 and 6 carry exactly zero, and the residual dead-zone distortion shows up as a
  0.05..0.08 A static offset on the affected DOFs.
- The theoretical worst distribution `delta = [+10, -10, +10, +10, +10]`: legs 2, 3, 4, 7 and 9 all sit at
  zero and the simulation **aborts**.

**This is a solver limitation, not a topology limitation.** Reducing the control current by 5 % (to 9.5 A)
lifts every leg current off zero and the model runs normally with a 1 % error, while the spread only moves
from 0.250 to 0.245:

| Distribution | `S_ideal` | `S_measured` | min abs leg current | result |
|---|---|---|---|---|
| `[10 -10 10 10 10]` | 0.2500 | - | **0.000 A** | solver aborts |
| `[9.5 -9.5 5 9.5 9.5]` | 0.2450 | 0.3028 | 0.248 A | ok, 1 % error |
| `[9.0 -9.0 5 9.0 9.0]` | 0.2400 | 0.2977 | 0.498 A | ok, 1 % error |
| `[9.5 -9.5 9.5 9.5 9.5]` | 0.2450 | 0.3029 | 0.198 A | ok, but 12 % error (leg 4 still near zero) |
| `[8 -8 10 8 8]` | 0.2300 | 0.2872 | 0.994 A | ok, 2 % error |

Note the third entry `delta3` in the worst distribution is **free** as far as the spread is concerned;
choosing it as +5 A instead of +/-9.5 A moves legs 4 and 5 away from zero and recovers the accuracy. That is
the practical mitigation, together with dead-time compensation.

---

## 6. Moving the extra leg

The axial pair plus its dedicated leg form a **series string** `[w_a5 - L_new - w_c5]` whose two ends may be
attached to **any two leg midpoints**, provided the graph stays a connected tree. Nothing else in the model
changes; only the leg-voltage back-substitution of section 4.3 is rewritten. Two relocations are verified
in this skill.

### 6.1 Variant A - axial pair between legs 3 and 7

Rewire `w5.L: leg4 -> leg3` and `w6.R: leg6 -> leg7`, then

```

ubridge(5) = 0;
ubridge(3) = ubridge(5) + uw5;          ubridge(7) = ubridge(5) - uw6;
ubridge(1) = ubridge(3) + uw3 + uw1;    ubridge(2) = ubridge(3) + uw3 + uw2;
ubridge(4) = ubridge(3) + uw3 - uw4;
ubridge(6) = ubridge(7) - uw8 + uw7;    ubridge(8) = ubridge(7) - uw8 - uw9;
ubridge(9) = ubridge(7) - uw8 - uw10;
```

### 6.2 Variant B - axial pair between legs 2 and 8

Rewire `w5.L: leg4 -> leg2` and `w6.R: leg6 -> leg8`, then

```

ubridge(5) = 0;
ubridge(2) = ubridge(5) + uw5;          ubridge(8) = ubridge(5) - uw6;
ubridge(1) = ubridge(2) - uw2 + uw1;    ubridge(3) = ubridge(2) - uw2 - uw3;
ubridge(4) = ubridge(2) - uw2 - uw4;
ubridge(6) = ubridge(8) + uw9 + uw7;    ubridge(7) = ubridge(8) + uw9 + uw8;
ubridge(9) = ubridge(8) + uw9 - uw10;
```

### 6.3 What changes and what does not

All three models track their references, and in all three the nine leg currents still sum to zero (the bias
still never reaches the bus). The placement changes two things:

| | baseline (legs 4-5-6) | variant A (3-5-7) | variant B (2-5-8) |
|---|---|---|---|
| leg currents [A] | 5.49, 4.50, -5.49, **1.00, -1.00, 1.00**, 4.49, -5.49, -4.50 | 5.51, 4.51, **-0.11, -4.48, -0.91, 5.50, -0.06**, -5.49, -4.49 | 5.49, **9.99**, -5.49, -4.49, **-1.00**, 5.49, 4.49, **-9.99**, -4.50 |
| control currents [A] | 0.9992, 0.9970, 0.9965, 0.9993, 0.9988 | 0.9992, **1.0603, 0.9057, 1.0303**, 0.9988 | 0.9995, 0.9989, 0.9985, 0.9991, 0.9983 |
| max abs leg current | 5.9 A | 5.8 A | **11.3 A** |
| sum abs leg currents | 33.0 A | 31.1 A | **50.9 A** |

**Criterion 1 - bias cancellation decides the device rating.** Whether a shared leg carries the bias or
only the control current depends on the *direction* of the co-located radial winding. The series string
always drives its outer windings with a fixed direction (out of the X1-side leg, into the X2-side leg), so
the leg is bias-free only if the radial winding at that node runs the other way. On the X1 side that means
choosing a leg of the *second* pair of the group (the winding that runs from the star out to the leg, i.e.
w3 or w4); on the X2 side a leg of the *first* pair (w7 or w8). Choosing a leg of the other pair makes it
carry `2*i_bias`: variant B puts 9.99 A on legs 2 and 8.

**Criterion 2 - the dead zone decides the accuracy.** Among the bias-free choices, the shared leg current is
either the *sum* or the *difference* of the two adjacent control currents:

- baseline: `leg4 = (d2+d3)/2`, `leg5 = -d3`, `leg6 = (d3+d4)/2` - all three move away from zero when the
  adjacent control currents have the same sign;
- variant A: `leg3 = (d3-d2)/2` and `leg7 = (d3-d4)/2` - both sit at **exactly zero** whenever the two
  adjacent control currents are equal (5.49 - 5.49), which is precisely the verified operating point. The
  result is a 6..9 % steady-state error on DOF2..DOF4 that disappears when the dead time is removed.

With the verified operating point (all five control currents equal to +1 A), the baseline is the **only**
one of the four bias-free placements that satisfies both criteria: it is the unique choice whose two shared
legs carry the *sum* of adjacent control currents. A relocation is therefore legitimate and changes nothing
about the topology, but it must be checked against these two criteria rather than picked arbitrarily.

---

## 7. Verification checklist

Run these on the finished model, in steady state, at least 10 ms after start-up. Reference values are from
the verified 9-leg / 10-winding model at `i_ref = 1 A`, `U_dc = 200 V`, `i_bias = 5 A`.

| # | Check | How | Pass criterion / reference value |
|---|---|---|---|
| 1 | Star-node KCL | `(i_a1+i_c1) - (i_a2+i_c2)` and the X2 pair | residual < 1e-9 A (measured **5.3e-15 / 7.1e-15 A**) |
| 2 | Leg currents sum to zero | `sum(I_leg)` from the KCL table of 1.3 | < 1e-9 A (measured **9.8e-15 A**) |
| 3 | Tellegen identity | `sum(U.*I)*U_dc` against `sum(D.*I_leg)*U_dc` | < 1e-6 W (measured **1.1e-13 W**) |
| 4 | Steady-state accuracy | winding and control currents | `i_a = 5.495 A`, `i_c = 4.496 A`, control current 0.9992 A (-0.08 %) |
| 5 | Bias conservation | pair sum `i_a + i_c` | within 0.05 A of `2*i_bias` (measured 9.9905 A) |
| 6 | Unidirectionality | minimum of every winding current | always > 0 |
| 7 | Leg-current pattern | mean of each leg current | legs 1-2-3-7-8-9: +5.5 / +4.5 / -5.5 / +4.5 / -5.5 / -4.5 A; legs 4-5-6: +1.0 / -1.0 / +1.0 A |
| 8 | Modulation law | mean spread of the leg commands | `S = 0.2089`; duty 0.2903..0.7097 over the whole run, 0.3956..0.6044 in steady state |
| 9 | Duty law | `duty - ubridge` | equals `0.5 - 0.5*(max+min)`, i.e. exactly the injected common mode |
| 10 | Spread model | `S` against `i_ref` at 200 V | slope `R/U_dc = 0.0050 /A` (measured 0.00555), constant overhead 0.0534 pu |
| 11 | No clamping | count samples with `abs(u_k) = 1` | 0 in steady state |
| 12 | Power balance | bus input against `sum(i_k^2*R)` plus device loss | copper loss 252.0 W |
| 13 | Dead-time overhead | spread at `i_ref = 0` with and without dead time | 0.2033 against 0.1635, difference `2*Td/Ts = 0.0400` (measured 0.0399) |
| 14 | Bus margin | `U_dc` against the rule of section 5.2 | 200 V against about 44 V, margin about 4.5x |
| 15 | Relocation | rebuild a variant, rerun | the leg-current pattern moves to the new legs, the control currents still reach 1 A |
| 16 | No algebraic loop | compile the model | no algebraic-loop warning |

---

## 8. Symptom -> cause -> fix

| Symptom | Likely cause | Fix |
|---|---|---|
| Legs 4, 5 and 6 carry only about 1 A while the other legs carry 5 A | **Correct** for this placement: those legs serve the axial pair and carry the difference of adjacent winding currents, so the bias cancels | Nothing. Verify against the KCL table of section 1.3 instead of expecting the bias everywhere |
| Simulation aborts with 1000 consecutive zero crossings | A leg sits at exactly zero current: `i_ref = 0`, or the worst-case distribution, or a relocated leg whose current is a difference of equal control currents | Use a small non-zero control current, or pick the free `delta3` to move the leg current off zero. Do not disable zero-crossing detection |
| Duty cycles leave [0,1] and the current collapses | Bus voltage below `U_dc,min` of section 5.2, or a bad back-substitution that inflates the spread | Raise `U_dc`; re-derive `ubridge` from the tree; check `S <= 1` |
| The commanded leg voltage difference is far larger than `R*i/U_dc` | Dead time plus device drops (the `sigma0` overhead of section 2) | Expected. Do not fix it by retuning; check that the integrators have headroom |
| One pair of windings drifts away from its reference while the other pair in the same group is fine | The two DOFs sharing a star node were given different bias currents; KCL then forces a compromise | Command the same `i_bias` for both DOFs of a star group |
| After relocating the axial pair the current loop will not converge | `winding_voltage_to_bridge` was not rewritten for the new tree | Re-derive the nine lines from the new edges (section 4.3); the rest of the model needs no change |
| The relocated model works but two legs get very hot | The new midpoints were chosen so that the axial-pair winding and the co-located radial winding push current in the *same* direction, so those legs carry `2*i_bias` | Re-apply criterion 1 of section 6.3 |
| Control current is 6..9 % off while the winding currents look plausible | The shared legs carry the *difference* of two equal adjacent control currents and sit in the dead zone | Re-apply criterion 2 of section 6.3, add dead-time compensation, or move the free `delta3` |
| Measured accuracy changes by several percent between runs | Adaptive zero crossing was enabled | Revert to `ZeroCrossControl = UseLocalSettings` with `ZeroCross = on` |
| One open winding loses a whole group of channels | Each star node is a hard-wired joint of four winding ends | Topology property, not a modelling error; design bypass or redundancy |

---

## 9. References and bundled files

Everything below lives inside this skill folder, so the skill is self-contained:

| Path | What it is |
|---|---|
| `model/Ten_phase_nine_leg_with_RL_load.slx` | the verified reference model: 9 legs, 10 windings, 5 DOF (byte-identical to the engineer model, md5 `0e5bbb66a6f61da9e0210f63aa7ccffd`) |
| `model/parameters_for_AMB_topology.m` | the base-workspace parameter script the model depends on (md5 `a90c6e390a9c1e14bf015396b2d6054d`) |
| `model/Ten_phase_nine_leg_relocated_L3_L7.slx` | axial pair moved to legs 3-5-7, `ubridge` rewritten (md5 `1f051832b3c2760930ffcd561aadd9c4`) |
| `model/Ten_phase_nine_leg_relocated_L2_L8.slx` | axial pair moved to legs 2-5-8, `ubridge` rewritten (md5 `36e3503b18cf1c1abf8b819d801ccf7f`) |
| `references/case-ten-phase-nine-leg-amb.md` | full case study: inventory, netlist derivation, star-node KCL, leg-current table, control and modulation law, measured verification, dead-time evidence |
| `references/case-ten-phase-nine-leg-modulation-boundary.md` | the boundary study: physical range, spread algebra, bus-voltage design rule, measured sweeps, numerical sensitive region |
| `references/case-ten-phase-nine-leg-relocating-the-extra-leg.md` | the placement family, the two criteria, the three-model comparison, the rewritten back-substitutions |
| `references/block-and-parameter-map.md` | exact block names, library paths, parameter names and values, port names, netlist edges and Goto/From tags |
| `scripts/verify_model_facts.m` | asserts every block, parameter and netlist entry of the map against the bundled models; 91 checks, no simulation |
| `scripts/run_case_study.m` | reproduces and regresses the numbers quoted above (baseline 13 checks, relocation 11, dead time 4, boundary sweeps) |
| `scripts/make_topology_figure.m` | regenerates `assets/fig1_topology.png` |
| `assets/fig1_topology.png` | schematic of the 9-leg / 10-winding graph |
| `assets/fig2_winding_currents.png` | the ten winding currents with the staggered start-up |
| `assets/fig3_control_currents.png` | the five control currents against the 1 A reference |
| `assets/fig4_leg_currents.png` | the nine leg currents and their identically zero sum |
| `assets/fig5_relocated_leg_currents.png` | leg currents of the baseline and the two relocations |
| `assets/fig6_modulation_boundary.png` | spread vs bus voltage, spread vs control current, achievable control current |
| `assets/fig7_analytic_limit.png` | the analytic boundary against the measured points |

### Running the bundled model

Environment in which the bundled models were built and verified:

| | |
|---|---|
| MATLAB | **R2025b (25.2)**, 25.2.0.3312555 Update 6 |
| Simulink | **25.2** |
| Simscape / Simscape Electrical | **25.2** |
| Platform | Linux (GLNXA64) |

The `.slx` files were saved by that release. Opening them in an older release triggers an upgrade prompt and
may not reproduce every block setting; treat R2025b (or a later release) as the supported environment.
Nothing beyond Simulink + Simscape Electrical is required. The only non-English text in the bundled `.slx`
files is three comment lines inside the `minmax` MATLAB Function chart; they are kept because the engineer
model is bundled byte-identical.

```

% cd to the model/ folder of this skill
run('parameters_for_AMB_topology.m');   % MUST run first - it starts with clearvars
m = 'Ten_phase_nine_leg_with_RL_load';
load_system(m);
out = sim(m);                           % as delivered: StopTime 0.05 s, accelerator mode
```

`parameters_for_AMB_topology.m` defines the base-workspace variables the model needs (`R_value`,
`L_value`, `U_dc`, `f_c`, `K_p`, `K_i`, `SW_frequency`, `SW_Ts`, `half_SW_Ts`, `bias_current`). Run it
before touching any other variable, because its first statement is `clearvars`. Nothing else needs to be on
the MATLAB path - the models and that one script are the only files required to run them.

The scripts are relocatable: they locate the models relative to their own folder, copy them to the system
temp folder before instrumenting them (the bundled `.slx` files are never written to), and print a PASS/FAIL
regression against the numbers quoted above.

```

% from the scripts/ folder of this skill
verify_model_facts               % about 30 s, 91 checks, no simulation
run_case_study                   % about 1 min, baseline
run_case_study('relocation')     % about 1 min, adds the two relocation variants
run_case_study('deadtime')       % about 1 min, dead-time overhead evidence
run_case_study('boundary')       % about 5 min, sweep endpoints, regenerates fig6 and fig7
run_case_study('point', 1.0, 200)% one operating point
```

