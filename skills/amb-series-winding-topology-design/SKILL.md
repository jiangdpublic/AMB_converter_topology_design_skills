---
name: amb-series-winding-topology-design
description: Design, build, or review the power topology and winding-current control loop of an active magnetic bearing (AMB) drive in which all windings are connected in series as one chain and every two adjacent windings share one leg (series winding topology, SWT) - N+1 legs for N windings, the same-direction versus alternating-direction chain variants and their device-count / device-current trade-off, the cumulative-sum plus min-max common-mode-injection modulator, bias plus differential per-winding PI current control, the physical operating range abs(i_ref) <= 2*i_bias, and the two-level boundary analysis (physical force limit versus bus-voltage modulation limit). Use when modelling or validating an AMB (or any series-connected multi-winding drive with shared legs); when choosing between the same-direction and alternating-direction chain wirings; when deciding which effects a topology-verification model must keep versus may safely drop; or when reviewing why such a model shows an unexpectedly small shared-leg current, a modulation spread that does not move with the control current, a simulation that aborts on zero crossings, or a shared leg rated for the wrong current.
metadata:
  author: topology_series_winding workspace
  version: "1.0"
---

# AMB Series-Winding Topology: Minimal Modeling and Verification

A 5-DOF AMB drives 8 radial + 2 axial windings (10 coils). Each coil carries a current made of a
**bias** term plus a differential control term, and the usable control variable is the difference of a
winding pair, `i_a - i_c`. Conventional drives give every winding its own H-bridge (two legs per
winding), or one and a half legs per winding with a half 3-legs cell. The **series-winding topology**
instead connects **all windings end to end in a single chain** and places **one leg at every
junction**: winding k sits between the midpoints of legs k and k+1, so **N windings need only N+1
legs** - 11 legs for 10 windings, 1.1 legs per winding against 1.5 for a half 3-legs cell and 2.0 for
independent H-bridges. All legs hang on **one common DC bus**, so the winding voltages become
differences of leg voltages and the leg references become **cumulative sums** of the winding voltage
commands.

This skill gives the minimum Simulink/Simscape model that verifies such a topology and its current loop,
states explicitly **what may be ignored** while the model is used for topology verification only, names
the simulation mechanics that may **not** be ignored, and lists the checks that prove the topology is
modelled correctly.

**Verified reference implementation**: a 10-winding, 5-DOF model bundled at
`model/Series_winding_with_RL_load.slx` (md5 `e90577ca21added1decb131a64dbe7cf`) together with its
parameters at `model/parameters_for_AMB_topology.m` (md5 `a90c6e390a9c1e14bf015396b2d6054d`), verified
with **MATLAB 25.2 (R2025b), Simulink 25.2, Simscape 25.2 and Simscape Electrical 25.2 (Linux
`glnxa64`)**. The bundled `.slx` is **byte-identical to the engineer's model**; its only non-English text
is three comment lines inside the `MATLAB Function` (`minmax`) chart, kept on purpose. The measurements
are written up in `references/case-series-winding-amb.md` and
`references/case-series-winding-modulation-boundary.md` (a superseded first pass is kept in
`references/case-series-winding-modulation-boundary-first-pass-superseded.md`); every block, parameter
and signal name quoted below is listed in `references/block-and-parameter-map.md` and re-checked by
`scripts/verify_model_facts.m`; the numbers are reproduced and regressed by `scripts/run_case_study.m`.

## When to Use

- Building or reviewing an AMB winding-drive power stage in which **all windings share one series
  chain**, and deciding the chain order and the leg count (`N+1`).
- Choosing between the **same-direction** and the **alternating-direction** chain wiring, and predicting
  what each one puts on the shared legs.
- Designing or reviewing the modulator for a series chain: cumulative sums plus min-max common-mode
  injection, and the duty law that follows from it.
- Deciding which effects a topology-verification model must keep and which it may safely drop.
- Working out the two boundaries that actually matter: the **physical** force limit
  `abs(i_ref) <= 2*i_bias` and the **bus-voltage** modulation limit.
- Diagnosing a series-winding model: shared-leg currents that look far too small, a modulation spread
  that does not move with the control current, a simulation that aborts on zero crossings, or a
  winding-current error at an extreme of the range.

## When NOT to Use

- Engineering-grade design: loss and thermal budgets, EMI, protection, bus capacitor sizing, device
  selection, digital controller implementation. Those need the effects this skill deliberately drops.
- Position/force loop tuning or rotor dynamics - a topology-verification model has no motion model.
- Closed-loop bandwidth calibration of the differential current channel - that is a controller-design
  task, not a topology-verification task.
- Building the Simulink model mechanically (block insertion, wiring syntax) - use the
  `building-simulink-models` skill for the mechanics; this skill supplies the design content.
---

## 0. First principle: a topology-verification model needs only the core chain

**When the question is "does this topology work and can its current loop regulate the control current?",
almost all non-ideal effects and essentially all engineering practice can be left out.** Keep exactly
these seven things and nothing else by default:

1. A stiff DC bus - one ideal voltage source.
2. **N+1 half-bridge legs** on that bus, each driven by a gate signal from the modulator.
3. Each winding as a **series R-L branch** with a current sensor.
4. Reference generation `i_a_ref = bias + 0.5*i_ref`, `i_c_ref = bias - 0.5*i_ref`.
5. One per-winding **per-unit PI** current controller with an output clamp.
6. **One symmetric triangular carrier plus min-max common-mode injection**, driving the cumulative sums
   of the winding voltage commands.
7. **Sampling of the current feedback** (section 3.1) - this one is not optional.

Everything else is deferred. Do not volunteer device `Vf`/`Ron`, dead time, switching or conduction loss,
thermal networks, magnetic saturation, mutual coupling, iron or eddy loss, back-EMF, motion equations,
force models, position loops, bus impedance, protection and fault logic, quantisation, computation delay,
gate-drive level selection, or saturation-limit selection. In a topology-verification model they only
slow the simulation down and obscure the switching behaviour that is being verified.

**Why the control current is the right thing to specify.** The question a topology-verification model
must answer is "can this topology drive the control current to the value I ask for, everywhere in the
valid range?". A constant `i_ref` answers it directly and cheaply; a sweep of `i_ref` then maps the
range. Dynamic profiles, reversal tests and bandwidth measurements belong to the later controller-design
stage, which needs an engineering-grade model anyway.

**Useful degenerate checks.** `i_ref = 0` (both windings at exactly the bias current) is a valid physical
operating point but a numerically hard one - see section 5.3. Use a small non-zero value (`i_ref = 0.6 A`
was the smallest workable value in the verified model) when a sweep needs to reach the low end.

---

## 1. Topology skeleton (build recipe)

### 1.1 The chain

Name the legs `L0 ... LN` and the windings `w1 ... wN`. Winding k connects the midpoint of leg k-1 to the
midpoint of leg k. All legs share one DC bus:

```
Leg0 --w1-- Leg1 --w2-- Leg2 --w3-- Leg3 ... --wN-- LegN
bus: every leg between +U_dc and 0 (all legs in parallel on the same bus)
```

For a 5-DOF AMB with 10 coils the chain used in the verified model is `a1, c1, a2, c2, a3, c3, a4, c4,
a5, c5`, i.e. **the a and c winding of one DOF are adjacent**. That ordering matters: it is what makes the
inner legs carry the *control* current rather than a difference of bias currents (section 1.3).

Each leg is a standard half-bridge leg (two controlled switches with anti-parallel diodes). Unlike the
switch+diode legs of a half 3-legs cell, a series chain built from conventional half-bridge legs **does**
need dead time, because a leg can otherwise short the bus.

### 1.2 The two chain wirings

| | same-direction (verified here) | alternating-direction |
|---|---|---|
| winding k direction | all the same way along the chain | every other winding reversed |
| leg between the two windings of one DOF | `-i_ref` | `i_a + i_c = 2*i_bias` |
| leg between adjacent DOFs | `0.5*(i_ref,j + i_ref,j+1)` | `2*i_bias` |
| inner-leg current direction | **reverses with `i_ref`** | always the same sign |
| inner-leg devices | **two active switches** | one switch may be replaced by a diode |
| inner-leg rating in normal operation | about `i_ref` | `2*i_bias`, always |

The same-direction chain makes the shared legs cheap in **current** (1 A against 10 A in the verified
operating point); the alternating chain makes them cheap in **devices**. The trade is deliberate and worth
stating explicitly in any design review.

### 1.3 The chain KCL law - what the shared legs really carry

With `i_k` the current of winding k flowing from leg k-1 to leg k, KCL at the midpoint of leg m gives

```
I_leg(m) = i_m - i_(m-1)          (inner legs)
I_leg(0) = i_1 ;  I_leg(N) = -i_N (end legs)
```

With the bias-plus-differential definitions `i_a = I_b + di` and `i_c = I_b - di`, and the chain order
a1,c1,a2,c2,... :

| Leg | Current | Measured at `i_ref = 1 A` |
|---|---|---|
| end leg L0 | `i_a1 = I_b + 0.5*i_ref` | **+5.4991 A** |
| between a_j and c_j | `-i_ref,j` | **-1.0000 A** |
| between c_j and a_(j+1) | `0.5*(i_ref,j + i_ref,j+1)` | **+0.9996 A** |
| end leg L10 | `-i_c5` | **-4.4968 A** |

**Only the two end legs carry the bias current; the nine inner legs carry the difference of the two
adjacent winding currents.** Because the a and c windings of one DOF are adjacent, that difference is
exactly the control current of that DOF. Measured KCL residual in the verified model: **1.4e-14 A** (the
check is part of `scripts/run_case_study.m`).

Two consequences worth carrying into a design review:

- The inner legs are cheap in current but **must be bidirectional**, because `i_ref` changes sign in
  normal AMB operation. The alternating wiring is the one that allows a diode.
- In the **worst corner inside the valid range** - two adjacent DOFs driven to opposite limits - an inner
  leg carries up to `2*i_bias` even in the same-direction wiring. Measured at the corner
  `[+10,-10,+10,-10,+10] A` the accuracy of the affected DOFs degrades from -0.04 % to -1.30 %.

### 1.4 The three hard constraints of the series topology

1. **The bus voltage must exceed the sum of all winding resistive drops.** The bias current of every
   winding flows the same way along the whole chain, so `sum(v_k) = sum(R*i_k + L*di_k/dt)`. In the
   verified model `sum(R*i_k) = 50 V` against a 200 V bus (measured total chain duty 0.30258). Copper
   resistance rises with temperature (+0.39 %/K), so this requirement grows.
2. **The modulation cumulative sums must stay inside [0,1].** `b_j = e_j + 0.5 - m` with
   `e_j = sum_(k>=j) u_k`; the requirement is `max(e) - min(e) <= 1`, i.e. `sum(abs(u_k)) <~ 1` and
   therefore `sum(abs(v_k)) <~ U_dc`. This is what limits simultaneous `L*di/dt` transient in all ten
   windings at once.
3. **The chain is a single point of failure.** One open winding opens the whole chain and all ten
   windings lose control. H-bridge or half 3-legs topologies do not have this property; bypass or
   redundancy has to be designed in.

### 1.5 Leg-count comparison

| Scheme | Legs | Legs per winding | Devices for 10 windings |
|---|---|---|---|
| Independent H-bridge | 20 | 2.0 | 40 switches + 40 diodes |
| Half 3-legs cell per pair | 15 | 1.5 | 30 switches + 30 diodes |
| **Series chain (this skill)** | **11** | **1.1** | **22 switches + 22 diodes** |
---

## 2. What you may ignore (and when it stops being safe)

This is the heart of topology design: most non-ideal effects, and essentially all engineering practice,
can be dropped because they do not change **which devices conduct, in which direction, or the average
voltage applied to the winding**. Add them back only when the question changes.

| Item | Why it is safe to drop in a topology model | When it stops being safe |
|---|---|---|
| Device `Vf`, `Ron`, `Goff` | Shifts the operating duty, not the switching states or the current direction | Loss/thermal budget, efficiency, device selection |
| Dead time | Correctly sized dead time does not change which devices conduct in steady state | It adds a fixed duty overhead (below) and distortion; only a real gate-drive design can size it |
| Switching transients, `di/dt`, reverse recovery | Do not change the average-current behaviour | EMI, overvoltage, device stress |
| Thermal network, junction temperature | Far slower than the current loop | Any temperature-dependent study |
| Magnetic saturation, mutual coupling, iron/eddy loss | Change `L` and add cross-coupling; not needed to verify switching states | Position dependence, force linearity, high-frequency loss |
| Back-EMF, motion equation, force model, position loop | A motionless winding is a pure R-L branch | Any study of stiffness, damping or rotor dynamics |
| Bus capacitance, source impedance | An ideal source fixes the bus voltage | Regenerative-energy handling, bus sizing, DC-link resonance |
| Protection, fault logic, EMI filtering | Not part of the topology function | Safety or compliance work. **Note constraint 3 of section 1.4: the chain itself is a single point of failure, and that IS a topology property.** |
| Feedback filter bandwidth and damping | Sets phase margin, not the switching states | Not ignorable if closed-loop bandwidth is the question |
| Gate-drive level | Any non-zero level turns an ideal switch on | Real gate-drive design, isolation, `dV/dt` |
| Controller and integrator clamps | Only bound large-signal excursions | Fault ride-through, saturation recovery |
| **Differential reference factor (0.5)** | Definition of the control current | Never - it *is* the control law |
| **Modulator constants (0.5, 0)** | Part of the per-unit modulation law | Never - they define the duty mapping |
| **Whether `i_a - i_c` is drawn on a scope** | It is a two-signal difference; post-processing gives it exactly | Never for verification; only for readability |
| Digital implementation detail | Second-order for switching-state verification | When the loop bandwidth itself matters |

**The one place where two "droppable" effects come back.** Dead time and device drops do not change the
topology result, but together they add a **fixed duty-cycle overhead**
`sigma0 = 2*Td/Ts + (device-drop term)`, measured at **0.053** in the verified model (about 10.6 V on a
200 V bus). It appears as a constant in the bus-voltage design rule of section 5.2. You do not need to
model it in order to verify the topology; you do need to allow roughly 5 % when you predict the modulation
boundary quantitatively. The effect is directly visible in the verified model: the commanded leg
difference for winding a1 is `0.0785 x 200 = 15.7 V` while the winding only needs `5.5 V`, and the
integrators absorb the difference.

Rule of thumb: if an effect does not change **which devices conduct**, **in which direction**, or **the
average voltage applied to the winding**, it does not belong in a topology-verification model.

---

## 3. What you must NOT ignore: simulation mechanics

### 3.1 Current-feedback sampling

The feedback path is the one place where a modelling shortcut silently biases the result. Sample the
feedback at a defined rate, synchronously with the PWM:

- `Rate Transition` with `OutPortSampleTimeOpt = Specify` and `OutPortSampleTime = Ts`, `Integrity = on`,
  `Deterministic = on`; or a `Zero-Order Hold` at `Ts`.
- Use the **same phase** for the two windings of a pair, so that `i_a - i_c` does not inherit a
  sampling-phase error.
- Keep a `Memory` block **after** the sampler: a `Rate Transition` has direct feedthrough, so the sampler
  alone does not break the `feedback -> controller -> PWM -> leg -> current -> feedback` algebraic loop.
- A second-order low-pass in the feedback path (`fn` around `4*f_c`, `zeta = 0.707`) is conventional and
  harmless for topology verification, but it does cost phase margin. Put it **after** the sampler.

### 3.2 Zero-crossing detection must stay on

Ideal switches plus zero-crossing detection are what resolve the switching edges accurately. In the
verified model `MaxStep = 1e-6` while the narrowest conduction interval is far shorter; the variable-step
solver brackets the comparator zero crossings and the pulse widths come out right. The price is a large
number of sub-nanosecond steps (630 940 steps for 12 ms of simulated time). That is acceptable for
topology verification.

**Quantified warning against switching to adaptive zero crossing.** Re-running the *same* operating point
with `ZeroCrossAlgorithm = Adaptive` moved the control current from 0.9995 A to 0.9464 A, an error of
**-5.4 %**. Adaptive zero crossing is useful for getting past a hard start-up, but it must not be used for
quantitative results, and the model default (`ZeroCrossControl = UseLocalSettings`, every block
`ZeroCross = on`) should be kept.

### 3.3 Measurement hygiene

Always reduce logged data with **time-weighted** averages, `sum(x.*dt)/sum(dt)`, never with `mean(x)`. A
variable-step solver clusters samples around the switching edges, so a plain sample average is biased.

### 3.4 Solver settings of the verified model

| Setting | Value |
|---|---|
| Solver | `VariableStepAuto` |
| MaxStep / RelTol | `1e-6` / `1e-4` |
| StopTime | 0.1 s as delivered; 12 ms is enough for steady-state topology verification |
| SimulationMode | `accelerator` |
| Solver Configuration | `UseLocalSolver = off` |

---

## 4. Control and modulation law

### 4.1 Differential (bias) reference generation

```
i_a_ref = bias_current + 0.5 * i_ref
i_c_ref = bias_current - 0.5 * i_ref
```

The 0.5 factor is the definition of the control current and must not be changed. The linearity limit of
the pair is `abs(i_ref) < 2*bias_current`.

### 4.2 Per-winding per-unit PI

Design the loop in per-unit voltage (controller output = `V_winding / U_dc`), which makes the gains
independent of the bus voltage:

```
K_p = 2*pi*f_c*L/U_dc          K_i = 2*pi*f_c*R/U_dc
```

With this choice the PI zero cancels the winding pole exactly, the open loop becomes `K_p*U_dc/(L*s)`, the
crossover is `f_c` and the closed-loop time constant is `L/(K_p*U_dc)` - about 106 us in the verified
model, where `f_c = 1500 Hz`, `K_p = 0.023562`, `K_i = 47.1239`. Choose `f_c` around `f_sw/10` to `f_sw/13`.

Add **conditional-integration anti-windup**: when the integrator state is at its limit and the error would
push it further (`abs(x) >= 1` AND `x*err >= 0`), force the integrator input to zero; keep the
integrator output clamp as the second line of defence.

> Naming trap seen in practice: a logic block **named** `AND` may actually be configured with
> `Operator = OR` - that is exactly what makes `abs(x) >= 1` work. Always read the `Operator` parameter,
> never the block name. `scripts/verify_model_facts.m` asserts the operator.

### 4.3 The modulator: cumulative sums plus min-max injection

**Step 1 - cumulative sums.** This is the essence of the series topology. Take the per-winding voltage
commands `u_0 ... u_(N-1)` and form

```
e_j = sum_(k >= j) u_k           j = 1..N+1 ,   e_(N+1) = 0
```

Leg m is driven by `e_(m+1)`, so the winding voltage is `v_k = (e_(k+1) - e_(k+2))*U_dc = u_k*U_dc`.
**The leg references are the cumulative sums of the winding voltage commands** - that is what "series"
means here.

**Step 2 - min-max zero-sequence injection.**

```
cmin = min(e);  cmax = max(e);
b_j  = e_j + 0.5 - 0.5*(cmin + cmax)
```

With a 0->1->0 symmetric triangular carrier the comparator duty equals the threshold `b_j`. This injection
squeezes the N+1 cumulative sums symmetrically into `[0,1]` and is the step that makes the series chain
usable at all.

**Measured verification** (steady state, window 10.5-12 ms):

| Quantity | Measured |
|---|---|
| threshold range | `b_min = 0.34871`, `b_max = 0.65129` |
| spread `max(b) - min(b)` | **0.30258** |
| gate duty vs threshold | `d_j = b_j - 0.01994` |
| total chain duty `d_0 - d_10` | 0.30258 (equals the spread) |
| largest `abs(u_k)` | 0.0785, with **0** clamped samples |

The duty offset `b_j - d_j = 0.01994 = Td/Ts = 1 us / 50 us` says that the dead time removes exactly one
turn-on delay from each gate. With a carrier that rises 0->1->0 the comparator output is one contiguous
interval across the period boundary, so each gate loses one `Td_on` per period; the two gates together
give 2 us of both-off time, i.e. 4 %.
---

## 5. Boundaries: physical, modulation, numerical

Three different limits get confused with one another. Keep them apart.

### 5.1 Physical boundary: `abs(i_ref) <= 2*i_bias`

The AMB force depends on the **magnitude** of the current, not its direction, so the force-producing
quantity is `abs(i_a) - abs(i_c)`. At `i_ref = 12 A` the windings carry `+11 A` and `-1 A`, and
`abs(11) - abs(-1) = 10 A` - exactly what `i_ref = 10 A` already gives. Beyond `2*i_bias` a larger control
current produces **no additional force**.

Measured in the verified model:

| i_ref (A) | i_a (A) | i_c (A) | `i_a - i_c` (A) | `abs(i_a) - abs(i_c)` (A) |
|---|---|---|---|---|
| 9.5 | 9.7460 | 0.2500 | 9.4960 | 9.4960 |
| **10.0** | 9.9960 | 0.0010 | 9.9957 | **9.9950** |
| 10.5 | 10.2460 | -0.2490 | 10.4957 | **9.9970** |
| 11.0 | 10.4960 | -0.4990 | 10.9953 | **9.9970** |
| 12.0 | 10.9535 | -0.9825 | 11.9360 | **9.9710** |

The loop itself keeps tracking beyond 10 A (the error is still only -0.53 % at 12 A), but the winding
currents are no longer unidirectional and the premise of a biased AMB is gone. **This is the boundary that
matters for operation.**

### 5.2 Modulation boundary: bus voltage, not control current

The modulation margin is almost independent of `i_ref`. The reason is analytical and worth remembering: the
total chain voltage is set by the **sum** of all winding currents, and `i_a + i_c = 2*i_bias` is independent
of the control current, so **the volt-second demand of the chain is constant**.

Measured spread over the whole range: 0.30258 from 0.6 A to 8 A, growing only to 0.31133 at 11 A - a
factor of **3.2** below the limit of 1.0.

Lowering the **bus voltage** is what finally saturates the modulator:

| U_dc (V) | 200 | 100 | 70 | 60 | **55** | **50** |
|---|---|---|---|---|---|---|
| b_min / b_max | 0.349 / 0.651 | 0.219 / 0.781 | 0.109 / 0.891 | 0.037 / 0.963 | **0.014 / 0.986** | **-0.373 / 1.373** |
| spread | 0.303 | 0.562 | 0.781 | 0.926 | **0.972** | **1.747 (out of range)** |
| control-current error | -0.05 % | -0.25 % | -0.35 % | -0.72 % | -0.26 % | **-81.8 % (collapse)** |

**Design rule:**

```
static:      U_dc > sum_k(R_k * i_bias,k) / (1 - sigma0)      sigma0 ~= 2*Td/Ts + device-drop term ~= 0.05
dynamic:     U_dc > [ sum_k(R_k*i_k) + sum_k(L_k*|di_k/dt|) ] / (1 - sigma0)
recommended: U_dc >= 2 * sum_k(R_k * i_bias,k)                 % one full margin for dynamics
```

For the verified model `sum(R*i_bias,k) = 50 V`, hence `U_dc,min = about 53 V`; the measurement falls
between 55 V (works) and 50 V (collapses). The model runs at 200 V, a margin of **3.8x**, which is why the
modulator never saturates anywhere in the physical range. At 200 V the remaining modulation range is
`1 - 0.303 = 0.697`, about 139 V of volt-seconds, enough for all ten windings to slew together at a
combined rate of about **27.8 A/ms**.

### 5.3 Numerical sensitive region: an inner leg at zero current

Because `I_leg(m) = i_m - i_(m-1)`, **an inner leg carries zero current whenever the two adjacent winding
currents are equal** - for example at `i_ref = 0`, or with two adjacent DOFs driven to opposite signs. This
is a perfectly normal physical operating point; the difficulty is in the solver.

| Case | Behaviour with the model default zero crossing |
|---|---|
| `i_ref = 0` | **simulation aborts**: an IGBT produces 1000 consecutive zero crossings at t = 3.66 ms. With adaptive zero crossing it completes and gives the correct steady state (`i_a - i_c = -0.0051 A`), but takes about 7x longer |
| `abs(i_ref) <= 0.5 A` | aborts at start-up |
| `i_ref = 0.6 A` (smallest workable value) | completes; spread and tracking identical to the rest of the range |
| adjacent DOFs with opposite signs | completes, but accuracy degrades from -0.04 % to **-1.30 %** |

Guidance: use `i_ref = 0.6 A` (or any value comfortably above the ripple band) as a substitute when a
sweep must reach the low end; do not change the model default zero-crossing setting to work around it; and
treat a zero-current inner leg as a numerical sensitivity, **not** as a topological defect.

---

## 6. Verification checklist

Run these on the finished model, in steady state, at least 10 ms after start-up. Values are from the
verified 11-leg / 10-winding model at `i_ref = 1 A`, `U_dc = 200 V`.

| # | Check | How | Pass criterion / reference value |
|---|---|---|---|
| 1 | Chain KCL | `I_leg(m)` against `i_m - i_(m-1)` | residual < 1e-3 A (measured **1.4e-14 A**) |
| 2 | Modulation law | time-weighted mean of `b_j` | `b_min = 0.34871`, `b_max = 0.65129`, spread 0.30258 |
| 3 | Duty law | `b_j - d_j` from the gate signals | equals `Td/Ts` = 0.01994 |
| 4 | Chain duty | `d_0 - d_10` | equals the spread, 0.30258 |
| 5 | Steady-state accuracy | winding and control currents | `i_a = 5.4980 A`, `i_c = 4.4990 A`, `i_a - i_c = 0.9995 A` (-0.05 %), bias 4.9985 A |
| 6 | Unidirectionality | minimum of every winding current | always > 0 (measured 4.359 A over the whole run) |
| 7 | Leg currents | mean of the device currents per leg | end legs `+5.4991 A` / `-4.4968 A`; inner legs `+0.9996 A` / `-1.0000 A` |
| 8 | No clamping | count samples with `abs(u_k) = 1` | 0 |
| 9 | Power balance | bus input against `sum(i_k^2 R)` plus device loss | 274.8 W = 252.3 W + 22.5 W |
| 10 | Step response | step `i_ref` by 0.5 A | rise time about 0.10 ms, overshoot about 5 % |
| 11 | Common-mode decoupling | `i_a + i_c` during a differential step | stays within 1e-3 A of `2*i_bias` |
| 12 | Crosstalk between DOFs | neighbour control current during a step | dip <= 1 % (measured 0.5 %) |
| 13 | Physical limit | `abs(i_a) - abs(i_c)` at `i_ref = 10, 11, 12 A` | saturates at 9.995 / 9.997 / 9.971 A |
| 14 | Bus margin | `U_dc` against `sum(R*i_k)/(1 - sigma0)` | 200 V against about 53 V, margin 3.8x |
| 15 | No algebraic loop | compile the model | no algebraic-loop warning |
| 16 | Anti-windup | read the `Operator` parameter of the logic blocks | `abs(x) >= 1` AND `x*err >= 0` freezes the integrator |

---

## 7. Symptom -> cause -> fix

| Symptom | Likely cause | Fix |
|---|---|---|
| Inner-leg currents are ~1 A while the windings carry ~5 A | This is **correct** for the same-direction chain: the inner legs carry the difference of adjacent winding currents, not the winding current | Nothing. Verify against the KCL table instead of expecting `2*i_bias` |
| The modulation spread does not change when `i_ref` changes | **Correct**: `sum(i_a + i_c) = 2*i_bias` is constant, so the chain volt-second demand is constant | Nothing. Use the bus voltage to find the boundary |
| Simulation aborts with 1000 consecutive zero crossings | An inner leg is at zero current (`i_ref` near 0, or adjacent DOFs with opposite signs), where the ideal-switch voltage discontinuity retriggers the detector | Use a small non-zero `i_ref` (0.6 A in the verified model); do not disable zero-crossing detection |
| Duty cycles run outside [0,1] or the current collapses | Bus voltage below `sum(R*i_k)/(1 - sigma0)` | Raise `U_dc`; check the design rule of section 5.2 |
| The commanded leg difference is far larger than `R*i/U_dc` | Dead time plus device drops (the `sigma0` overhead) | Expected. Do not fix it by retuning; check that the integrators have headroom |
| Steady-state winding current is a fraction of a percent off while the controller feedback sits on the reference | Un-sampled feedback held across large variable steps | Add the `Rate Transition` of section 3.1, keep the `Memory` after it |
| Measured accuracy changes by several percent between runs | Adaptive zero crossing was enabled | Revert to `ZeroCrossControl = UseLocalSettings` with `ZeroCross = on` |
| Time-weighted duty does not satisfy the duty law | Duty computed as a sample average instead of a time-weighted average | Use `sum(x.*dt)/sum(dt)` |
| A large control-current step takes milliseconds to settle although the loop is designed for 0.1 ms | Bias charging of all ten windings through one chain, plus the zero-current inner-leg sensitivity | Expected in this topology; add a reference ramp for large-signal moves |
| One open winding loses all ten channels | The chain is a single series path | Topology property, not a modelling error; design bypass or redundancy |

---

## 8. References and bundled files

Everything below lives inside this skill folder, so the skill is self-contained:

| Path | What it is |
|---|---|
| `model/Series_winding_with_RL_load.slx` | the verified reference model: 11 legs, 10 windings, 5 DOF (byte-identical to the engineer model, md5 `e90577ca21added1decb131a64dbe7cf`) |
| `model/parameters_for_AMB_topology.m` | the base-workspace parameter script the model depends on (md5 `a90c6e390a9c1e14bf015396b2d6054d`) |
| `references/case-series-winding-amb.md` | full case study: inventory, netlist derivation, control and modulation law, measured verification, modelling-quality assessment |
| `references/case-series-winding-modulation-boundary.md` | the modulation-boundary study: physical range, bus-voltage boundary, and the zero-current numerical anomaly |
| `references/case-series-winding-modulation-boundary-first-pass-superseded.md` | the superseded first pass, kept for the audit trail |
| `references/block-and-parameter-map.md` | exact block names, library paths, parameter names and values, port names, netlist edges and Goto/From tags |
| `scripts/verify_model_facts.m` | asserts every block, parameter and netlist entry of the map against the bundled model; 130 checks, no simulation |
| `scripts/run_case_study.m` | reproduces and regresses the numbers quoted above (16 regression checks in the default mode) |
| `scripts/make_topology_figure.m` | regenerates `assets/fig1_topology.png` |
| `assets/fig1_topology.png` | schematic of the 11-leg / 10-winding chain |
| `assets/fig2_startup.png` | staggered start-up of the five DOFs |
| `assets/fig3_steady_state_zoom.png` | steady-state zoom: winding currents, control current, gate signals |
| `assets/fig4_leg_currents.png` | mean current per leg: only the two end legs carry the bias |
| `assets/fig5_modulation_boundary.png` | first-pass boundary figure: tracking, spread vs `i_ref`, headroom vs `U_dc`, boundary fit |
| `assets/fig6_physical_range_and_boundary.png` | final boundary figure: force-current saturation at 10 A, spread over the range, tracking error, bus-voltage boundary |

### Running the bundled model

Environment in which the bundled model was built and verified:

| | |
|---|---|
| MATLAB | **R2025b (25.2)**, 25.2.0.3312555 Update 6 |
| Simulink | **25.2** |
| Simscape / Simscape Electrical | **25.2** |
| Platform | Linux (GLNXA64) |

The `.slx` was saved by that release. Opening it in an older release triggers an upgrade prompt and may not
reproduce every block setting; treat R2025b (or a later release) as the supported environment. Nothing
beyond Simulink + Simscape Electrical is required.

```matlab
% cd to the folder containing the bundled .slx (the model/ folder of this skill)
run('parameters_for_AMB_topology.m');   % MUST run first - it starts with clearvars
m = 'Series_winding_with_RL_load';
load_system(m);
out = sim(m);                           % as delivered: StopTime 0.1 s, accelerator mode
```

`parameters_for_AMB_topology.m` defines the base-workspace variables the model needs (`R_value`,
`L_value`, `U_dc`, `f_c`, `K_p`, `K_i`, `SW_frequency`, `SW_Ts`, `half_SW_Ts`, `bias_current`). Run it
before touching any other variable, because its first statement is `clearvars`. Nothing else needs to be on
the MATLAB path - the model and that one script are the only files required.

The three verification scripts are relocatable: they locate the model relative to their own folder, copy it
to the system temp folder before instrumenting it (the bundled asset is never written to), and print a
PASS/FAIL regression against the numbers quoted above.

```matlab
% from the scripts/ folder of this skill
verify_model_facts         % about 20 s, 130 checks, no simulation
run_case_study             % about 2 min, baseline case
run_case_study('boundary') % about 8 min, adds the sweep endpoints
```
