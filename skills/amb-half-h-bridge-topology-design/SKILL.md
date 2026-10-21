---
name: amb-half-h-bridge-topology-design
description: Design, build, or review the power topology and winding-current control loop of an active magnetic bearing (AMB) drive in Simulink and Simscape - the half-H bridge topology (one two-switch H-bridge per winding, two legs per bridge), bias plus differential current references, per-winding PI current control, three-level PWM from one triangular carrier, and RL winding loads. Use when modeling or validating an AMB (or any bias-current, differential-winding) power stage; when deciding which effects a topology-verification model must keep versus may safely drop; or when reviewing why such a model shows a steady-state winding-current offset, unexpected current ripple, or a simulation far slower than expected.
metadata:
  version: "1.1"
---

# AMB Half-H Bridge Topology: Minimal Modeling and Verification

A 5-DOF AMB drives 8 radial + 2 axial windings (10 coils). Each coil carries a
**unidirectional** current made of a bias term plus a differential control term, and the
usable control variable is the difference of a winding pair, `i_a - i_c`. The **half-H bridge
topology** gives each winding one complete two-switch H-bridge (two legs). This skill gives the
minimum Simulink/Simscape model that verifies such a topology and its current loop, states
explicitly what may be ignored while the model is used for **topology verification only**, and
lists the checks that prove the topology is modelled correctly.

**Verified reference implementation**: a 10-bridge (20-leg), 5-DOF model bundled at
`assets/Half_H_Bridge_with_RL_load.slx` (md5 `c14d7d1984439a5fa3de7977664b5482`) together with its
parameters in `assets/parameters_for_AMB_topology.m` (md5 `a90c6e390a9c1e14bf015396b2d6054d`),
verified with **MATLAB/Simulink R2025b (25.2), Simscape and Simscape Electrical 25.2** - see
`references/case-half-h-bridge-amb.md` and section 8 for how to re-run the checks.

## When to Use

- Building or reviewing an AMB winding-drive power stage based on the half-H bridge topology
  (a two-switch H-bridge per winding, or a conventional H-bridge) and its per-winding current
  loop in Simulink/Simscape.
- Deciding whether a modelling detail (device drops, dead time, magnetics, motion, position
  loop, bus non-idealities) belongs in a topology-verification model.
- Diagnosing a topology model: steady-state current offset, duty-cycle statistics that do not
  match the modulation law, unexplained ripple, very small solver steps, or an algebraic loop.

## When NOT to Use

- Engineering-grade design: loss/thermal budgets, EMI, protection, bus capacitor sizing,
  digital controller implementation, sensor selection. Those need the effects this skill drops.
- Position/force loop tuning, rotor dynamics, or AMB controller (displacement) design - those
  require a plant model that a topology-verification model deliberately omits.
- Building the Simulink model mechanically (block insertion, wiring syntax) - use the
  `building-simulink-models` skill for the mechanics; this skill supplies the design content.

## 0. First principle: a topology-verification model needs only the minimal loop

Keep exactly these six things, and nothing else by default:

1. A stiff DC bus (one ideal voltage source).
2. One half-H bridge per winding (two legs per bridge), with ideal switches.
3. The winding as a series R-L branch (plus a current sensor).
4. A wiring that makes the winding current **unidirectional by construction**.
5. Differential reference generation (bias +/- half of the control current).
6. A per-winding PI current controller and a PWM modulator in per-unit form.

Everything else is deferred. **Do not volunteer engineering-grade additions** - dead time,
device `Vf`/`Ron`, switching or conduction loss, thermal networks, magnetic saturation, mutual
coupling, iron/eddy loss, back-EMF, motion equations, force models, position loops, bus
impedance, protection and fault logic - unless the user explicitly asks for an engineering-grade
model. In a topology-verification model these only slow the simulation and obscure the
switching-state behaviour that is being verified.

The **only** two items that must never be dropped are simulation mechanics, because they
silently corrupt the very quantity being verified (the winding current): current-feedback
**sampling** and solver **step size** - see section 3.

## 1. Topology skeleton (build recipe)

### 1.1 Half-H bridge: four nodes, two legs, two switches, two diodes

A **half-H bridge** is one complete H-bridge structure around one winding, with **two legs**.
Only two of the four devices are controlled switches; the other two are freewheel diodes, so
each leg carries one switch and one diode. Name the four electrical nodes: **P** (bus positive),
**N** (bus negative), **M** (upper switching node), **A** (lower switching node). The winding is
connected **between M and A** (floating); it is never returned to the bus.

| Leg | Device | Terminals | Connects |
|---|---|---|---|
| Upper (M) | Upper IGBT, e.g. `IGBT (Ideal, Switching)` | C / E | P -> M (C to bus +, E to M) |
| Upper (M) | Diode `Diode` (upper freewheel) | anode / cathode | A -> P (anode at A, cathode at bus +) |
| Lower (A) | Lower IGBT, e.g. `IGBT (Ideal, Switching)1` | C / E | A -> N (C to A, E to bus -) |
| Lower (A) | Diode `Diode1` (lower freewheel) | anode / cathode | N -> M (anode at bus -, cathode at M) |
| - | Winding branch | in series | M -> `Resistor` -> `Inductor` -> `Current Sensor` -> A |
| - | Gate drive | | `Simulink-PS Converter` -> IGBT G; `Current Sensor` I -> `PS-Simulink Converter` -> subsystem output |

Both diodes point **upward** (anode on the lower-potential node). That single fact is what makes
the winding current unidirectional; reversing either diode breaks the topology.

Expose the bus as Simscape connection ports on the bridge subsystem - `Vcci`/`Vddi` (left) and
`Vcco`/`Vddo` (right) - so that bridges can be daisy-chained and share one bus.

### 1.2 Switching states, output voltage, and the unidirectional-current proof

| State | Upper switch | Lower switch | Current path | Winding voltage `V_MA` |
|---|---|---|---|---|
| Energise | on | on | P -> upper switch -> M -> winding -> A -> lower switch -> N | `+U_dc - 2*V_sw` (about `+U_dc`) |
| Freewheel (upper) | on | off | M -> winding -> A -> `Diode` -> P -> upper switch -> M | `-(V_sw + V_D)` (about 0) |
| Freewheel (lower) | off | on | N -> `Diode1` -> M -> winding -> A -> lower switch -> N | `-(V_D + V_sw)` (about 0) |
| Regenerative | off | off | M -> winding -> A -> `Diode` -> P -> source -> N -> `Diode1` -> M | `-U_dc` |

Consequences that should be quoted when justifying the topology:

- **Current is unidirectional in all four states** (always M -> A through the winding). This is
  exactly what a biased AMB winding needs, and it is why a two-switch half-H bridge is sufficient.
- **No dead time is required**: the two controlled switches sit in different legs, one on each side
  of the winding, and are never a complementary pair across the bus, so there is no shoot-through
  path. There is also no dead-time distortion to model.
- The output is **three-level** (`+U_dc`, `0`, `-U_dc`). With the modulation of section 1.6 the
  effective current-ripple frequency is `2*f_sw`, not `f_sw`.
- Cost of the topology: every freewheel path contains **two** devices in series (one switch plus
  one diode), so the "0" state sits at roughly `-3 V` with real devices. The controller must
  supply extra duty to hold the bias current, and the freewheel decay sets the current ripple.
  In an ideal-switch topology-verification model this only shifts the operating duty; it does not
  change any topology conclusion.

### 1.3 Scaling to N windings

Instantiate one half-H bridge (two legs) per winding - `N = 2 x DOF`: 8 radial + 2 axial = 10
bridges, 20 legs - chain all bridges across one bus, and keep the windings floating and
independent. There is no magnetic or electrical coupling between windings in this model, so the
N current loops are decoupled and can each be tuned independently.

### 1.4 Differential (bias) reference generation

For every DOF, generate the two winding references from one control-current reference:

```
i_a_ref = bias_current + 0.5 * i_ref
i_c_ref = bias_current - 0.5 * i_ref
```

In Simulink this is a `Gain` (0.5) feeding two `Sum` blocks: `Add` with `Inputs = '++'`
(+0.5*i_ref + bias) and `Add1` with `Inputs = '-+'` (-0.5*i_ref + bias). Reference currents must
stay positive: `|i_ref| < 2*bias_current` is the linearity limit of the differential pair.

### 1.5 Per-winding per-unit PI current controller

Design the loop in per-unit voltage (controller output = `V_winding / U_dc`), which makes the
gains independent of the bus voltage:

```
Kp = 2*pi*f_c*L/U_dc          Ki = 2*pi*f_c*R/U_dc
```

With this choice the PI zero `Ki/Kp = R/L` cancels the winding pole exactly, and the open-loop
gain becomes `Kp*U_dc/(L*s)`, i.e. a crossover at `f_c` with a first-order closed loop (about
`tau = L/(Kp*U_dc)`). Choose `f_c` around `f_sw/10` to `f_sw/13`; the sampling delay analysed in
section 3.1 then decides how much phase margin is actually left.

Controller structure (per winding):

```
err      = ref - fbk_filtered            (Sum, Inputs '+-')
U_raw    = Kp*err + integral_term        (Gain + Sum '++')
U_cmd    = clamp(U_raw, -1, +1)          (Saturation, LowerLimit -1, UpperLimit +1)
```

Add **conditional-integration anti-windup** on the integrator: when the integrator state is at
its limit and the error would push it further (`|x| >= 1` AND `x*err >= 0`), force the
integrator input to zero. Keep the integrator's own output clamp (`LimitOutput = on`,
`UpperSaturationLimit = 1`, `LowerSaturationLimit = -1`) as the second line of defence. Note that
with per-unit gains a bias-current step-up transient rarely reaches the clamp - the anti-windup
is a safety net for large-signal or fault cases, not part of the nominal response.

### 1.6 Three-level PWM from one triangular carrier

One carrier per half-H bridge (a `Repeating Sequence` with `rep_seq_t = [0 Ts/2 Ts]` and
`rep_seq_y = [0 1 0]`, i.e. a symmetric triangle of period `Ts = 1/f_sw`), and two duty commands:

```
d1 = 0.5 + 0.5*U      d2 = 0.5 - 0.5*U
sw1 = Vg * (d2 <= tri)          sw2 = Vg * NOT(d1 <= tri)      (Vg = gate level, e.g. 15 V)
```

Because the triangle is symmetric, the time-weighted duties are
`duty(sw1) = duty(sw2) = 0.5 + 0.5*U` and the "both switches on" fraction is exactly `U`
(for `U > 0`; the "both off" fraction is `|U|` for `U < 0`). Hence the design relation

```
U = 2*duty(sw1) - 1        and        V_avg = U*(U_dc - 2*V_sw) + (1-U)*(-V_fw)
```

Each carrier period contains **two** `+U_dc` pulses (one on the rising, one on the falling
carrier slope), with the two freewheel states interleaved between them. That is the source of the
`2*f_sw` ripple frequency and of the halved ripple amplitude compared with a two-level modulator.

## 2. What you may ignore (and when it stops being safe)

| Item | Why it is safe to drop in a topology model | When it stops being safe |
|---|---|---|
| Device `Vf`, `Ron`, `Goff`; use `Ron = 0`, `Vf = 0` | Shifts the operating duty, not the switching states or the current direction | Loss/thermal budget, efficiency, device selection |
| Dead time (none is needed here) | No shoot-through path exists | Only if you replace a switch+diode leg with a conventional two-switch half-bridge leg |
| Switching transients, `di/dt`, reverse recovery | Do not change the average-current behaviour | EMI, overvoltage, device stress |
| Thermal network | Far slower than the current loop | Any temperature-dependent study |
| Magnetic saturation, mutual coupling, iron/eddy loss | Change `L` and add cross-coupling; not needed to verify switching states | Position-dependence, force linearity, high-frequency loss |
| Back-EMF, motion equation, force model, position loop | A motionless winding makes the load a pure R-L branch | Any study of stiffness, damping, or rotor dynamics |
| Bus capacitance, source impedance, pump-up | An ideal source fixes the bus voltage | Regenerative-energy handling, bus sizing |
| Protection, fault logic, EMI | Not part of the topology function | Safety or compliance work |
| Digital implementation detail: quantisation, computation delay, interrupt jitter | Second-order for switching-state verification | **Not ignorable if the loop bandwidth matters** - see section 3.1 |

Rule of thumb: if an effect does not change **which devices conduct**, **in which direction**, or
**the average voltage applied to the winding**, it does not belong in a topology-verification
model.

## 3. What you must NOT ignore: simulation mechanics

### 3.1 Current-feedback sampling

The feedback path is the one place where a modelling shortcut silently biases the result.

**Counter-example (measured).** Feeding the winding current straight into a `Memory` block (used
to break the algebraic loop) with a variable-step solver and no maximum step: the Memory
zero-order-holds the value from the *previous major step*, and the solver takes large steps
(10-12 us) exactly where the current ramps slowly. The held value systematically over-estimates
the current. Measured effect: the controller's feedback settled at `5.4996 A` while the true
time-weighted winding current was `5.4306 A` - a **+1.27 % measurement bias**, which pushed the
true bias current **1.25 % low** (a 1 % force-gain error in the AMB).

**Correct construction.** Sample the feedback at a defined rate, synchronously with the PWM:

- Insert a `Rate Transition` with `OutPortSampleTimeOpt = 'Specify'` and
  `OutPortSampleTime = Ts` (the switching period), with `Integrity = 'on'` and
  `Deterministic = 'on'`; or use a `Zero-Order Hold` with `SampleTime = Ts`.
- Use the **same phase** for the two windings of a differential pair, so that the difference
  `i_a - i_c` does not inherit a sampling-phase error.
- Keep the `Memory` block **after** the sampler: a `Rate Transition` has direct feedthrough, so
  the sampler alone does **not** break the
  `feedback -> controller -> PWM -> bridge -> current -> feedback` algebraic loop. Verified: deleting
  the `Memory` block raises exactly one algebraic-loop diagnostic.
- Sampling exactly at the ripple average point is a refinement, not a requirement: sampling once
  per `Ts` leaves a residual of about `0.13 %` in this model (the ripple period is `Ts/2`, so the
  sample always lands at the same ripple phase).

### 3.2 Solver step size and tolerance

- Bound the step: `MaxStep <= Ts/20` (for `f_sw = 20 kHz`, `MaxStep = 1e-6`) and tighten
  `RelTol` to `1e-4` or smaller. Alternatively enable the Simscape local solver
  (`Solver Configuration`: `UseLocalSolver = 'on'`, `LocalSolverSampleTime = 1e-6`).
- Counter-example: with `MaxStep = 'auto'` and `RelTol = 1e-3`, 34 % of the solver steps were
  shorter than 10 ns and the smallest was `2.2e-25 s` - numerically meaningless work around every
  switching edge, and coarse steps in between.
- Expect the cost: bounding `MaxStep` to `1e-6` made a 100 ms run take about 40 s (versus 7 s
  unbounded) for a 10-bridge (20-leg) model. That is acceptable for topology verification; do not "optimise"
  it away by loosening the step, because the sampling bias of section 3.1 returns.

### 3.3 Measurement hygiene

Always reduce logged data with **time-weighted** averages, `sum(x.*dt)/sum(dt)`, never with
`mean(x)`. A variable-step solver clusters samples around the switching edges; a plain sample
average of the duty cycle reads `0.748` where the time-weighted value is `0.521`.

## 4. Verification checklist

Run these on the finished model (steady state, at least 10 ms after start-up). Each check has a
pass criterion; all values below are from the verified 10-bridge (20-leg) / 5-DOF reference model.

| # | Check | How | Pass criterion / reference value |
|---|---|---|---|
| 1 | Duty law | time-weighted duty of both gates, and the "both on" fraction | `duty(sw1) = duty(sw2) = 0.5 + 0.5*U` and both-on `= U` (measured `0.5212`, `U = 0.0425`) |
| 2 | State sequence | list the `sw1*2 + sw2` states and their durations over 2 carrier periods | 24 us freewheel -> 1.04 us energise -> 24 us freewheel -> 1.04 us energise per 50 us |
| 3 | Current slope | `dI/dt` inside each state | energise `+0.386 A/us` (= 198 V / 500 uH); freewheel `-0.017 A/us` |
| 4 | Average-voltage law | `V_avg = U*(U_dc - 2*V_sw) + (1-U)*(-V_fw)` vs `i*R` | agree within ~1 % (measured 5.55 V vs 5.51 V) |
| 5 | Unidirectionality | winding current never changes sign, in steady state and during steps | always positive; no zero-crossing |
| 6 | Steady-state accuracy | time-weighted `i_a`, `i_c`, `i_a - i_c` vs references | within 0.5 % (measured 5.507 / 4.506 / 1.0012 A against 5.5 / 4.5 / 1.0 A) |
| 7 | PI residual | steady-state `err` and integrator state | `err -> 0`; integrator well inside its clamp (measured `err ~ 0`, `|x| = 0.042` of 1.0) |
| 8 | Ripple and spectrum | peak-to-peak current and FFT of one winding | dominant component at `2*f_sw` (measured 39 973 Hz, 2.5 % of DC; 0.41 A p-p on 5.5 A) |
| 9 | Differential ripple | peak-to-peak of `i_a - i_c` | small compared with the winding ripple (measured 0.062 A p-p on a 1 A control current) |
| 10 | No algebraic loop | compile the model / read diagnostics | no algebraic-loop warning |

## 5. Reference numbers from the verified case

| Quantity | Value |
|---|---|
| Bus / winding | `U_dc = 200 V`, `R = 1 ohm`, `L = 500 uH` |
| Switching / bias | `f_sw = 20 kHz` (`Ts = 50 us`), `bias_current = 5 A`, `i_ref = 1 A` |
| Controller | `f_c = 1500 Hz`, `Kp = 0.023562`, `Ki = 47.1239`, output clamp +/-1 |
| Feedback filter | second-order low-pass, `fn = 6000 Hz`, `Zeta = 0.707` |
| Steady state | `i_a = 5.507 A`, `i_c = 4.506 A`, `i_a - i_c = 1.0012 A` |
| Winding ripple | 0.41 A p-p (7.4 %), dominant harmonic at `2*f_sw` |
| Closed-loop dynamics | with no sampling delay: `t63 = 76 us`, no overshoot; with a 50-75 us sampling delay: about 9 % overshoot |
| Phase margin vs feedback delay | `0 us -> 69 deg`, `50 us -> 42 deg`, `75 us -> 29 deg`, `100 us -> 16 deg` |
| Runtime | about 40 s per 100 ms of simulated time with `MaxStep = 1e-6` on a 10-bridge (20-leg) model |

## 6. Symptom -> cause -> fix

| Symptom | Likely cause | Fix |
|---|---|---|
| Steady-state winding current is a percent or so off while the controller's feedback sits exactly on the reference | Un-sampled (`Memory`-only) feedback held across large variable steps | Add the `Rate Transition`/`Zero-Order Hold` of section 3.1 |
| Time-weighted duty does not satisfy `U = 2*duty - 1`, or both-on fraction is about 0.5 at `U ~ 0` | Duty computed as a sample average instead of a time-weighted average | Use `sum(x.*dt)/sum(dt)` |
| Tens of thousands of sub-nanosecond solver steps, very slow run | `MaxStep = 'auto'` with ideal switches | Set `MaxStep <= Ts/20`, or use the Simscape local solver |
| Algebraic-loop diagnostic after adding a sampler | `Rate Transition` has direct feedthrough | Keep a `Memory` (or a `Zero-Order Hold`, which has none) in the feedback path |
| Current never reaches zero or reverses during a step | Correct behaviour - the winding current is unidirectional by design | Do not "fix"; verify the differential current `i_a - i_c` instead |
| Large step-up of the bias current overshoots | Sampling/PWM delay reduced the phase margin | Lower `f_c`, or accept it for topology verification |

## 7. References

- `references/block-and-parameter-map.md` - exact block names, library paths, parameter names and
  values, port names, Goto/From tags, and model configuration of the verified reference model.
- `references/case-half-h-bridge-amb.md` - the full case study: model inventory, the feedback-sampling
  defect and its diagnosis, the before/after measurements, and how to reproduce them.
- `assets/` - the verified model and its parameter script (section 8).
- `scripts/` - the two runnable checks, `verify_model_facts.m` and `run_case_study.m` (section 8).

## 8. Bundled reference implementation

The verified model ships with this skill, so every check can be re-run without any external file.

| File | Purpose |
|---|---|
| `assets/Half_H_Bridge_with_RL_load.slx` | the verified 10-bridge (20-leg) / 5-DOF model (md5 `c14d7d1984439a5fa3de7977664b5482`) |
| `assets/parameters_for_AMB_topology.m` | its base-workspace parameters (md5 `a90c6e390a9c1e14bf015396b2d6054d`); the file starts with `clearvars` |
| `scripts/verify_model_facts.m` | asserts every block, library path, parameter, port, Goto tag and channel-map entry quoted in `references/block-and-parameter-map.md` against the bundled model - 238 checks |
| `scripts/run_case_study.m` | reproduces every number in section 5 and in `references/case-half-h-bridge-amb.md`, ending with a pass/fail regression table (10 checks) |

**How to run** - from any working folder; both scripts resolve their own location:

```matlab
skillRoot = '<folder containing SKILL.md>';
run(fullfile(skillRoot,'scripts','verify_model_facts.m'));   % ~10 s, prints "238 passed, 0 failed"
run(fullfile(skillRoot,'scripts','run_case_study.m'));       % ~20 s, ends with "10/10 checks passed"
```

Order and side effects:

- Both scripts run `assets/parameters_for_AMB_topology.m` first: the model resolves its block
  parameters from the base workspace, and that script begins with `clearvars`, so nothing works
  before it and it clears the base workspace.
- Every simulation runs on an instrumented **copy** in the system temp folder (`tempdir`), which is
  removed afterwards - the bundled `.slx` is never modified.
- Both scripts print a toolchain banner and add a note (not an error) when the release is not R2025b.

**Toolchain the case study was verified with**

| Component | Version |
|---|---|
| MATLAB | 25.2.0.3312555 (R2025b) Update 6 |
| Simulink | 25.2 (R2025b) |
| Simscape | 25.2 (R2025b) |
| Simscape Electrical | 25.2 (R2025b) |
| OS | Linux (WSL2) |

Only long-standing blocks and library paths are used, so earlier releases will most likely work;
`verify_model_facts.m` is the authoritative check for whichever release is in use.
