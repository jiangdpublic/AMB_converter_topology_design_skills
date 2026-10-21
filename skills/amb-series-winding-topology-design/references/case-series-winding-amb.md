# Series-Winding AMB Topology: Detailed Analysis of the Simulation Model

**Model**: `Series_winding_with_RL_load.slx` (R2025b / Simscape Electrical)  
**Parameters**: `parameters_for_AMB_topology.m`  
**Method**: static structure/parameter export + reproducible simulation on an instrumented copy + measured data. **The delivered model was not modified.**

> Translated from the Chinese working report `Series_winding_topology_analysis.md` produced during the
> development of this skill, with two editorial adjustments agreed with the model's author:
> (1) the dead time, the freewheel-diode on-resistance and the un-ramped start-up are presented as
> modelling choices retained on purpose rather than as defects, because they do not affect the
> topology-principle verification; (2) the anti-windup logic block, originally named `AND` while its
> operator was `OR`, has since been renamed to `OR` in the model, and the text reflects the corrected state.

---

## 0. Summary (the eight headline results)

1. **The model implements an 11-leg / 10-winding same-direction series chain**: all windings are connected end to end in one chain, winding k sits between the midpoints of legs k and k+1, and all 11 legs are paralleled on **one common 200 V DC bus** (`Vcci`==`Vcco`, `Vddi`==`Vddo`, `Vsi`==`Vso` are feed-through ports, internally the same node). Each leg is a **standard half bridge (2 IGBTs + 2 anti-parallel diodes + 1 us dead time)**.
2. **Key topological result (measured)**: by KCL the current of leg m is always the difference of the two adjacent winding currents, `I_leg(m) = i_m - i_(m-1)`. With the chain order a1,c1,a2,c2,...,a5,c5 used here:
   - **the 9 inner legs carry only the control current, +/-i_ref (measured +/-1.0000 A)**;
   - **only the two end legs carry the bias current (measured +5.4991 A and -4.4968 A)**.
   The measured KCL residual is below 5e-5 A, exactly as predicted.
3. **Control accuracy is very good**: steady-state control current i_a - i_c = 0.9983...0.9995 A against a 1.0 A reference (error -0.06 % ... -0.17 %), bias 4.9964...4.9985 A (error <= 0.07 %); every winding current stays positive (minimum 4.359 A), so unidirectionality holds.
4. **The modulation law is correct**: the 11 leg thresholds are taken from the **cumulative sums of the per-winding voltage commands**, followed by **min-max zero-sequence (common-mode) injection**. Measured `b_j = e_j + 0.5 - 0.5(min e + max e)` with `max(b) - min(b) = 0.3026`, never leaving the linear range.
5. **The dead time is large**: Td = 1 us against Ts = 50 us. The measured gate duty is `d_j = b_j - 0.01994` (each switch loses about 1 us of on-time; both devices are off for 2 us per period = 4 %), which on a 200 V bus produces a **current-direction-dependent voltage error of the order of +/-8 V**. The controllers absorb it with a duty offset of about 0.05 (for example the commanded leg difference for winding a1 is 0.0785 x 200 = 15.7 V while the winding only needs 5.5 V).
6. **One device parameter is unrealistic**: the freewheel diode uses `Ron = 0.3 Ohm` (far too large for a 5 A model), giving 0.6 + 1.5 = 2.1 V at 5 A of freewheeling current and visibly distorting duty cycle and loss; the IGBT `Ron = 1 mOhm` is normal. A value of 10-50 mOhm would be appropriate. This is a modelling choice retained on purpose here (see the note in section 5) - it does not affect the topology-principle verification.
7. **Dynamic behaviour (measured step)**: for a control-current step 1.0 -> 1.5 A the rise time is about 0.1 ms, the overshoot about 5 %, the common-mode (bias) disturbance below 1e-3 A and the crosstalk into the neighbouring DOF only -0.5 %. **Differential and common mode are well decoupled, and both the control law and the modulation law work.** There is, however, a slow tail of about 1 ms.
8. **Start-up is the weak point**: the five DOFs are enabled at 0/1/2/3/4 ms and every enable produces a differential-current kick of the order of +/-2.5 A; everything settles to within 1 % only after about 8-10 ms, far beyond the 0.1 ms time scale of a single loop. A reference ramp (soft-start) is recommended. As with item 6, this is retained deliberately and is not a topology defect.

---

## 1. Model inventory and structure

### 1.1 Top-level composition

| Category | Count | Notes |
|---|---|---|
| `Bridge_module` ... `Bridge_module10` | **11** | half-bridge leg (feed-through DC bus + two-switch leg) |
| `RL_load` ... `RL_load9` | **10** | winding (R + L + current sensor) |
| `AMB_ref_gen` ... `AMB_ref_gen4` | 5 | bias + differential reference generation (one per DOF) |
| `PI_ctrl` ... `PI_ctrl9` | 10 | one PI current controller per winding |
| `PWM_gen` ... `PWM_gen10` | 11 | triangular-carrier comparison + dead time + gate level |
| `ref_delay` ... `ref_delay3` | 4 | reference enable switch (staggered start-up) |
| `Step`/`Step1`/`Step2`/`Step3` | 4 | enable instants 4/3/2/1 ms (for DOF5/4/3/2) |
| `MATLAB Function` | 1 | `[cmin,cmax]=minmax(u)`, min/max over the 11 channels |
| `Add`...`Add12`, `Mux1`, `Demux`, `Product`, `Gain1` | - | cumulative sums + min-max injection + demux |
| `Voltage Source` / `Electrical Reference` / `Solver Configuration` | 1 each | 200 V DC bus + ground + solver configuration |
| `Open Circuit` x4 | 4 | on `Bridge_module.Vsi` and `Bridge_module10.Vso/Vcco/Vddo`, to silence unconnected-port warnings |
| `winding_currents` (Scope) | 1 | the 10 winding currents (**note: DataLogging = off**) |

### 1.2 Inside `Bridge_module` (standard half-bridge leg)

Six connection ports, named in pairs, each pair internally the same node:

| Port | Internal node | Notes |
|---|---|---|
| `Vcci`/`Vcco` | IGBT1 collector = Diode1 cathode | DC bus **positive** (feed-through) |
| `Vddi`/`Vddo` | IGBT2 emitter = Diode2 anode | DC bus **negative** (feed-through) |
| `Vsi`/`Vso` | IGBT1 emitter = IGBT2 collector = Diode1 anode = Diode2 cathode | **leg midpoint** (feed-through) |

Device connections (verified edge by edge against the netlist):

```
+U_dc --+-- IGBT1(C->E) --+-- leg midpoint --+-- IGBT2(C->E) --+-- 0
        |                 |                  |                 |
      Diode1(up) --------+               Diode2(up) ----------+
   (anode=midpoint,             (anode=0,
    cathode=+)                   cathode=midpoint)
```

- The IGBTs use `ee_lib/.../IGBT (Ideal, Switching)` with `diode_param = external`, so the **anti-parallel diodes are provided externally as Diode1/Diode2**, in the correct orientation.
- **`Vsi` and `Vso` are internally the same node**, so leaving `Vsi` of `Bridge_module` open (tied to an Open Circuit block) has no electrical effect; module 10 brings its midpoint out on `Vsi` while the others use `Vso`. That is a drawing habit, not an error.

### 1.3 `RL_load` (the winding)

Series order: `L(port) -> Resistor2 -> Inductor2 -> Current Sensor2 -> R(port)`.
The current sensor is positive for current flowing **from the L port to the R port**, i.e. from module k to module k+1. Every winding connects its L end to the midpoint of module k and its R end to the midpoint of module k+1 - **this is the same-current-direction series arrangement**.

### 1.4 Electrical netlist (the 10-winding series chain)

```
Leg0 --w1(a1)-- Leg1 --w2(c1)-- Leg2 --w3(a2)-- Leg3 --w4(c2)-- Leg4
     --w5(a3)-- Leg5 --w6(c3)-- Leg6 --w7(a4)-- Leg7 --w8(c4)-- Leg8
     --w9(a5)-- Leg9 --w10(c5)-- Leg10
Bus: Leg0..Leg10 all in parallel on the same 200 V bus (Vcci/Vcco and Vddi/Vddo chained through)
```

See `assets/fig1_topology.png`.

### 1.5 Key parameters (all read from the model, not defaults)

| Parameter | Value | Location |
|---|---|---|
| Bus voltage `U_dc` | 200 V | `Voltage Source.dc_voltage` |
| Winding resistance `R_value` | 1 Ohm | `RL_load/Resistor2.R` |
| Winding inductance `L_value` | 500 uH (`r = 0`) | `RL_load/Inductor2.l` |
| Switching frequency `SW_frequency` | 20 kHz (`Ts = 50 us`) | parameter script |
| Carrier | symmetric triangle `rep_seq_t=[0,25us,50us]`, `rep_seq_y=[0,1,0]`, comparator `>=` | each `PWM_gen/Repeating Sequence` |
| Dead time | `Td_on = 1 us`, `Td_off = 0` (**applied to both gates**) | `PWM_gen/On-Off Delay(1)` |
| Gate level | 15 V (IGBT `Vth = 6 V`) | `PWM_gen/Gain(1)` |
| Current-loop bandwidth `f_c` | 1500 Hz | parameter script |
| `K_p` / `K_i` | 0.023562 / 47.1239 | `2*pi*f_c*L/U_dc`, `2*pi*f_c*R/U_dc` |
| Bias current `bias_current` | 5 A | parameter script |
| Control-current reference `i_ref` | 1 A (all five DOFs) | `Constant6`, `Constant1..4` |
| Feedback sampling | Rate Transition, `OutPortSampleTime = SW_Ts = 50 us`, Integrity/Deterministic on | `PI_ctrl/Rate Transition` |
| Feedback filter | second-order low-pass, `fn = 6000 Hz`, `zeta = 0.707`, `Ts = 0` (continuous) | `PI_ctrl/Second-Order Filter` |
| Output clamp | +/-1 (per unit) | `PI_ctrl/Saturation` |
| Integrator clamp | +/-1, `LimitOutput = on` | `PI_ctrl/Ictrl_anti_windup/Integrator` |
| Anti-windup | `abs(x) >= 1` **AND** `x*err >= 0` -> integrator input forced to 0 | `Ictrl_anti_windup` (block correctly named `OR` in the current model) |
| IGBT | `Vf = 0.8 V`, `Ron = 1 mOhm`, `Goff = 1e-5 S` | `Bridge_module/IGBT...` |
| **Freewheel diode** | **`Vf = 0.6 V`, `Ron = 0.3 Ohm`**, `Goff = 1e-8 S` | `Bridge_module/Diode1(2)` |

### 1.6 Solver and simulation configuration

| Item | Value |
|---|---|
| Solver | `VariableStepAuto` (variable step) |
| MaxStep / RelTol | `1e-6` / `1e-4` |
| StopTime / StartTime | **0.1 s** / 0 |
| SimulationMode | `accelerator` |
| ZeroCrossControl | UseLocalSettings (every block has `ZeroCross = on`, **zero-crossing detection retained, which is correct**) |
| AlgebraicLoopMsg | warning |
| Solver Configuration | `UseLocalSolver = off`, `IndexReductionMethod = DerivativeReplacement`, `AutomaticFiltering = on` (tau = 1 ms) |
| Measured step count | **630 940 steps for a 12 ms run** (average step about 19 ns; many ~9 ns steps during start-up) |

---

## 2. Topology analysis (including the comparison with the alternating-direction chain)

### 2.1 Chain KCL: what the inner legs actually carry

Let i_k be the current of winding k flowing from module k to module k+1 (the current-sensor positive direction). KCL at the midpoint of module m gives:

```
I_leg(m) = i_m - i_(m-1)          (m = 1..9, the inner legs)
I_leg(0) = i_0 ;  I_leg(10) = -i_9               (the two end legs)
```

Substituting the AMB bias-plus-differential definitions `i_a = I_b + di` and `i_c = I_b - di`, and noting that the chain order here is a1,c1,a2,c2,...:

| Leg position | Current expression | Measured in this operating point |
|---|---|---|
| end leg Leg0 | `i_a1 = I_b + 0.5*i_ref,1` | **+5.4991 A** |
| between a_j and c_j (Leg1,3,5,7,9) | `i_cj - i_aj = -i_ref,j` | **-1.0000 A** |
| between c_j and a_(j+1) (Leg2,4,6,8) | `0.5*(i_ref,j + i_ref,j+1)` | **+0.9996 A** |
| end leg Leg10 | `-i_c5 = -(I_b - 0.5*i_ref,5)` | **-4.4968 A** |

**Every measured KCL residual is below 5e-5 A**, in exact agreement with the expressions above.

> **One point worth stating precisely**: it is sometimes said that in the same-direction series wiring the inner legs carry "the corresponding winding current". By netlist and by measurement they do **not** carry the winding current itself (about 5 A); they carry **the difference of the two adjacent winding currents**. Because the a and c windings of one DOF are adjacent in this chain, that difference is exactly **the control current of that DOF, i_ref (measured +/-1 A)**, while the bias current is carried by the two end legs. Two independent pieces of evidence: (a) KCL is topologically enforced; (b) the measured average currents of all 11 legs match term by term.
>
> The comparison intended by that statement is nevertheless correct in direction: the alternating-direction chain puts **2*I_b (10 A here)** on its inner legs, whereas the same-direction chain puts only **i_ref (1 A here)** there - genuinely smaller. The precise wording is "the difference of the two adjacent winding currents", i.e. the control current of the DOF.

### 2.2 Quantitative comparison with the alternating-direction series chain

| Item | **This model: same-direction** | Alternating-direction series chain |
|---|---|---|
| Leg between a and c of one DOF | `-i_ref` (**small**, 1 A here) | `i_a + i_c = 2*I_b` (**large**, 10 A here) |
| Leg between adjacent DOFs | `0.5(i_ref,j + i_ref,j+1)` | `2*I_b` |
| End-leg current | `I_b +/- 0.5*i_ref` (about 5 A) | same |
| Leg current direction | **reverses with i_ref -> both devices must be active switches** | **always one direction (2*I_b > 0) -> one switch may be replaced by a diode** |
| Inner-leg device rating | about `i_ref` in normal operation; **up to 2*I_b = 10 A in the worst corner** (two adjacent DOFs driven to opposite limits) | always 2*I_b = 10 A |
| Device count | 11 legs x 2 devices = 22 IGBT + 22 D | slightly more legs, but one switch per inner leg can be saved |

**Conclusion**: the same-direction chain reduces the inner-leg current from 2*I_b to i_ref in normal operation (10 A -> 1 A, a factor of ten). The price is that the inner legs must be bidirectional and the end legs still carry the full bias current. The alternating chain saves devices but its inner legs carry 2*I_b continuously. This is a clear device-current versus device-count trade-off.

### 2.3 The three hard constraints of the series topology (implicit in this model, must be checked in any real design)

1. **The bus voltage must exceed the sum of all winding resistive drops.** The bias current flows one way along the whole chain, so the total chain voltage is `sum(v_k) = sum(R*i_k + L*di_k/dt)`. Here `sum(R*i_k) = 1 Ohm x (5.498+4.499) x 5 = about 50 V`, i.e. 25 % of the 200 V bus (measured total chain duty `d_0 - d_10 = 0.3026`). Because copper resistance rises with temperature (+0.39 %/K), this requirement grows and **margin must be reserved**.
2. **The modulation cumulative sums must stay inside [0,1].** `b_j = e_j + 0.5 - m` with `e_j = sum_(k>=j) u_k`; the requirement is `max(e) - min(e) <= 1`, i.e. **`sum(abs(u_k)) <~ 1` -> `sum(abs(v_k)) <~ U_dc`**. This directly limits how much simultaneous `L*di/dt` transient the ten windings can produce. The largest spread measured here is 0.5098, at the start-up instant - ample margin, but not negligible.
3. **The chain is a single point of failure.** If any winding opens, the whole chain opens and all ten windings lose control. H-bridge and half 3-legs topologies do not have this property; bypass or redundancy must be designed in.

### 2.4 Leg-count comparison

| Scheme | Legs | Legs per winding | Devices for 10 windings |
|---|---|---|---|
| Independent H-bridge (one per winding) | 20 | 2 | 40 IGBT + 40 D |
| Half 3-legs (one cell per winding pair) | 15 | 1.5 | 30 IGBT + 30 D |
| **This model: same-direction series chain** | **11** | **1.1** | **22 IGBT + 22 D** |

The series chain is the most economical in device count. The price is the three constraints of section 2.3 plus the fact that the whole bias current must be conducted by the two end legs and the chain itself.

---

## 3. Control and modulation

### 3.1 Reference generation (`AMB_ref_gen`)

```
ia_ref = bias_current + 0.5 * i_ref
ic_ref = bias_current - 0.5 * i_ref
```
A `Gain(0.5)` feeds a `Sum('++')` and a `Sum('-+')`. **The 0.5 factor is the control law itself and must not be changed**; the linear range is `abs(i_ref) < 2*bias_current = 10 A`.

### 3.2 Per-winding PI (`PI_ctrl`, per-unit voltage form)

```
fbk_f = LPF2( Memory( RateTransition(fbk, Ts=50us) ) )   % sample -> hold -> 2nd-order LPF (fn=6k, zeta=0.707)
err   = ref - fbk_f
U_raw = K_p*err + x                                      % x = integrator state
U_cmd = sat(U_raw, -1, +1)
dx/dt = K_i*err  if NOT( abs(x) >= 1 AND x*err >= 0 ), else 0    % conditional-integration anti-windup
```

- `K_p = 2*pi*f_c*L/U_dc = 0.023562`, `K_i = 2*pi*f_c*R/U_dc = 47.1239`, hence `K_i/K_p = R/L`: the **PI zero cancels the winding pole exactly**, the open loop is `K_p*U_dc/(L*s)`, the crossover is `f_c = 1500 Hz` and the closed-loop time constant is `L/(K_p*U_dc) = about 106 us`.
- The feedback chain is built **correctly**: the Rate Transition (`OutPortSampleTime = SW_Ts`) samples first, the `Memory` block breaks the algebraic loop and the second-order filter comes last; the a and c channels of one DOF sample **in phase**.
- The anti-windup logic is correct: `CompareToConstant(<=-1)` and `(>=1)` are combined by a logic block that computes `abs(x) >= 1` (this block was originally misnamed `AND` although its operator is `OR`; it has since been renamed to `OR`), then ANDed with `x*err >= 0` to drive a `Switch(u2 ~= 0)` that forces the integrator input to zero.

### 3.3 The modulation law (the most distinctive part of this model)

**Step 1 - cumulative sums (the essence of the series topology).** The ten PI outputs `u_0...u_9` are turned by `Add`...`Add12` into the cumulative-sum vector

```
Mux1 = [ sum_(k=0..9)u_k , sum_(k=1..9)u_k , ... , u_9 , 0 ] = [e_1, e_2, ..., e_11]
```
Leg m is driven by `e_(m+1)`, so that the voltage of winding k is `v_k = (e_(k+1) - e_(k+2))*U_dc = u_k*U_dc`. **This is the direct expression of "series": the leg voltages are the cumulative sums of the winding voltages.**

**Step 2 - min-max zero-sequence injection.**

```
cmin = min(e);  cmax = max(e);                        % MATLAB Function
b_j  = e_j - ( 0.5*(cmin+cmax) - 0.5 ) = e_j + 0.5 - 0.5*(cmin+cmax)
```
i.e. standard min-max common-mode injection `b_j = 0.5 - m + e_j` with `m = 0.5(max e + min e)`. For a 0->1->0 symmetric triangular carrier the comparator duty equals the threshold `b_j`. **This injection squeezes the eleven cumulative sums symmetrically into [0,1] and is the step that makes the series chain usable at all.**

**Measured verification** (steady state 10.5-12 ms, 630 940 full-resolution samples):

| Leg j | 0 | 1 | 2 | ... | 10 |
|---|---|---|---|---|---|
| modulation threshold `b_j` | 0.65129 | 0.57284 | 0.59885 | ... | 0.34871 |
| measured gate duty `d_j` | 0.63133 | 0.55288 | 0.57890 | ... | 0.32878 |
| `b_j - d_j` | 0.01996 | 0.01995 | 0.01996 | ... | 0.01993 |

- `max(b) - min(b) = 0.3026` (largest over the whole run 0.5098); the linear range is never exceeded.
- `b_j - d_j = 0.01994 = Td/Ts = 1 us / 50 us`: **the dead time removes exactly 1 us of on-time from each gate.** With a symmetric triangular carrier the comparator output is one contiguous interval across the period boundary, so only one Td is lost per gate; the two gates together give 2 us of both-off time per period, i.e. 4 %.
- The winding voltage commands recovered from b, `u_k = b_k - b_(k+1)`, are `[0.07845, -0.02602, 0.07595, -0.02602, 0.07595, -0.02602, 0.07594, -0.02602, 0.07594, 0.02441]`.

### 3.4 Why the a-winding command is +0.078 while the c-winding command is -0.026

In steady state the winding voltage must equal `i*R` (no DC drop across the inductance): **+5.5 V** for the a winding and **+4.5 V** for the c winding. The commanded differences, however, are +15.7 V and -5.2 V. The difference comes from **device drops plus dead time**:

- The Simscape log gives the instantaneous leg-midpoint voltage: about `200 - 0.8 - 0.001*i` = 199.2 V with the upper device on, and about `-0.6 - 0.3*abs(i)` with the lower device or the lower diode conducting. Leg10 measures **201.95 V = 200 + 0.6 + 0.3 x 4.5**, exactly matching the diode `Ron = 0.3 Ohm`.
- During the dead time the leg voltage is set by the **direction of the leg current**: positive current is clamped by D2 to `-0.6 - 0.3*i`, negative current is clamped by D1 to `200 + 0.6 + 0.3*abs(i)`.
- Each leg therefore deviates by about +/-5 V, and after differencing the error reaches **+/-8 to +/-10 V, i.e. 4-5 % of U_dc**, all of which the integrators absorb.

---

## 4. Simulation results (measured)

### 4.1 Steady-state accuracy (window 10.5-12 ms, time-weighted average)

| Winding | i_a1 | i_c1 | i_a2 | i_c2 | i_a3 | i_c3 | i_a4 | i_c4 | i_a5 | i_c5 |
|---|---|---|---|---|---|---|---|---|---|---|
| mean (A) | 5.4982 | 4.4988 | 5.4979 | 4.4989 | 5.4974 | 4.4988 | 5.4968 | 4.4986 | 5.4961 | 4.4967 |
| min/max | 5.283/5.710 | 4.417/4.579 | 5.301/5.691 | 4.424/4.572 | 5.318/5.673 | 4.427/4.569 | 5.308/5.682 | 4.420/4.576 | 5.289/5.700 | 4.359/4.632 |
| peak-to-peak ripple (A) | 0.427 | 0.162 | 0.391 | 0.148 | 0.355 | 0.142 | 0.375 | 0.156 | 0.411 | 0.273 |

| DOF | control current (A) | error | ripple pp (A) | bias (A) |
|---|---|---|---|---|
| 1 | 0.9995 | -0.055 % | 0.541 | 4.9985 |
| 2 | 0.9990 | -0.100 % | 0.518 | 4.9984 |
| 3 | 0.9986 | -0.141 % | 0.497 | 4.9981 |
| 4 | 0.9983 | -0.171 % | 0.475 | 4.9977 |
| 5 | 0.9994 | -0.060 % | 0.556 | 4.9964 |

**Unidirectionality holds**: over the whole run (0-12 ms) the minimum of every winding current is 4.359 A, far above zero.

### 4.2 Leg currents and the KCL check (the central result)

| Leg m | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| measured `I_leg` (A) | **+5.4991** | -1.0000 | +0.9996 | -0.9995 | +0.9989 | -0.9991 | +0.9983 | -0.9986 | +0.9976 | -0.9995 | **-4.4968** |
| `i_m - i_(m-1)` (A) | +5.4991 | -1.0000 | +0.9996 | -0.9995 | +0.9989 | -0.9991 | +0.9983 | -0.9986 | +0.9976 | -0.9995 | -4.4968 |
| residual | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

Device-level current sharing inside a leg (time-weighted average):

| Leg | IGBT upper | Diode1 | IGBT lower | Diode2 | Note |
|---|---|---|---|---|---|
| 0 | 3.4895 | about 0 | 0.0013 | 2.0108 | current always positive -> upper device / D2 conduct, effective duty 0.635 |
| 1,3,5,7,9 | about 0 | 0.397...0.596 | 0.454...0.604 | about 0 | current always negative -> D1 / lower device conduct |
| 2,4,6,8 | 0.433...0.583 | about 0 | 0.0011 | 0.418...0.566 | current always positive |
| 10 | 0.0012 | 1.6716 | 2.8264 | about 0 | current always negative, effective duty 0.372 |

Inner-leg current extremes over 10.5-12 ms: Leg1 in [-1.269, -0.728], Leg2 in [0.766, 1.231]. In this operating point **every leg current keeps a fixed direction**. As soon as i_ref reverses, however, all inner-leg currents reverse with it, so **the inner legs must be bidirectionally controllable**.

### 4.3 Power balance

- measured mean bus current 1.374 A -> input power 200 x 1.374 = **274.8 W**
- winding copper loss `sum(i_k^2 R)` = 5 x (5.498^2 + 4.499^2) = **252.3 W**
- device loss = 274.8 - 252.3 = **22.5 W (8.2 %)**, of which the diode `Ron = 0.3 Ohm` is a visibly large contributor.

### 4.4 Dynamic step response (i_ref of DOF2 stepped from 1.0 A to 1.5 A at t = 8 ms)

| t - 8 ms (ms) | 0.00 | 0.05 | 0.10 | 0.15 | 0.20 | 0.30 | 0.50 | 1.0 | 2.0 |
|---|---|---|---|---|---|---|---|---|---|
| i_a2 - i_c2 (A) | 0.9996 | 1.2304 | 1.4224 | 1.5157 | **1.5250** | 1.4738 | 1.4791 | 1.4916 | 1.4986 |

- **Rise time about 0.10 ms, overshoot about 5 %** for a 0.5 A step, consistent with the `f_c = 1500 Hz` design.
- **Common-mode decoupling is excellent**: `i_a2 + i_c2` stays between 10.0000 and 10.0008 A throughout (disturbance below 1e-3 A).
- **Crosstalk between DOFs is small**: the control current of DOF1 dips by only 0.5 % (0.9994 -> 0.9944) and recovers in about 1 ms.
- **There is a slow tail of about 1 ms** (98.7 % at 8.4 ms, 99.8 % only at 10 ms).

### 4.5 Start-up (staggered enable)

`ref_delay` uses a `Step` to drive a `Switch` that forces the reference to zero before enable. Order: DOF1 with no delay (t = 0), DOF2/3/4/5 at **1 / 2 / 3 / 4 ms**. Measured (see `assets/fig2_startup.png`):

- every enable produces a **differential-current kick of the order of +/-2.5 A**;
- the control current of each DOF settles to within 1 % only after 8-10 ms;
- already-enabled DOFs are repeatedly pushed off by later enables; for example the control current of DOF1 at t = 1/2/3/4 ms is 0.435 / 0.429 / 0.638 / 0.940 A;
- the winding currents themselves never undershoot below zero, so unidirectionality is preserved.

**Mechanism**: the bias current of all ten windings has to be established through one and the same chain, whose total voltage is limited by `sum(abs(u_k)) <~ 1`; and the dead-time error is largest when the inner-leg current is near zero (section 3.4), entering the differential mode with double weight (`v_a - v_c = V_(2k) - 2*V_(2k+1) + V_(2k+2)`). The differential mode therefore recovers slowly under large signals.

### 4.6 Ripple

| Quantity | Peak-to-peak |
|---|---|
| winding current | 0.142-0.427 A pp (2.8-7.8 % of the bias) |
| control current `i_a - i_c` | **0.475-0.556 A pp (48-56 % of a 1 A control current)** |
| common mode `i_a + i_c` | 0.479-0.667 A pp |

**The large differential ripple is inherent to the series half-bridge structure**: the two legs that bound the a winding are (2k, 2k+1) and those that bound the c winding are (2k+1, 2k+2), so the two windings share only the middle leg, and that middle-leg voltage **enters the differential mode twice** (`v_a - v_c = V_(2k) - 2*V_(2k+1) + V_(2k+2)`). Its switching ripple is amplified by two and the two winding ripples cannot cancel.

---

## 5. Modelling-quality assessment

### What is done right

1. **Cumulative-sum modulation of the series chain**: `e_j = sum_(k>=j) u_k` followed by min-max injection. This is the only practical modulator for a series topology and the measurements match it exactly.
2. **Feedback sampling chain**: the order `Rate Transition(Ts = Ts_sw) -> Memory -> second-order filter` is correct; the Memory block really does break the algebraic loop (no algebraic-loop warning at compile time) and the sampling rate is synchronous with the PWM.
3. **Zero-crossing detection retained** (`ZeroCross = on`, no Simscape local solver). This is the right trade-off for an ideal-switch model; the price is a very large number of sub-nanosecond steps (630 940 steps in 12 ms).
4. **Anti-windup**: conditional integration plus an integrator clamp, both correct.
5. **Device models**: the IGBT anti-parallel diodes are external and correctly oriented; device drops and on-resistances are included, which is what makes the duty-cycle offset observable.
6. **Bus and solver**: one ideal voltage source plus Solver Configuration and Electrical Reference - the minimal complete set needed for topology verification.

### Modelling choices retained on purpose (not defects)

| # | Choice | Evidence | Comment |
|---|---|---|---|
| 1 | **Freewheel diode `Ron = 0.3 Ohm`** (unrealistically large; 10-50 mOhm would be typical) | Leg10 instantaneous voltage 201.95 V = 200 + 0.6 + 0.3 x 4.5; 2.1 V drop at 5 A of freewheeling current | Kept as-is. It contributes to the device loss figure but does not change which devices conduct, in which direction, or the average winding voltage law. |
| 2 | **Dead time 1 us** (2 % of Ts; 4 % of both-off time per period) | `b_j - d_j = 0.01994`; produces an +/-8 to +/-10 V differential voltage error absorbed by a duty offset of about 0.05 | Kept as-is. It is the physical origin of the constant `sigma0` used in the bus-voltage design rule (section 2.3), but it does not affect the topology-principle verification. |
| 3 | **No start-up ramp** (all DOFs enabled at 0/1/2/3/4 ms at full reference) | `assets/fig2_startup.png`; differential current reaches +/-2.5 A | Kept as-is for a topology-verification model. A production controller would add a reference ramp. |

### Genuine improvement opportunities, by priority

| # | Item | Evidence | Suggestion |
|---|---|---|---|
| **1** | **Control-current ripple of about 50 %** | 0.475-0.556 A pp against 1 A | Raise `f_sw`, or experiment with phase interleaving of the 11 carriers, or optimise the injection; at minimum quantify the resulting force ripple. |
| **2** | **Eleven independent Repeating Sequence blocks** | identical parameters and simultaneous start, so synchronisation is currently implicit | Replace with **one shared carrier source** distributed to the 11 comparators, so that a later parameter change cannot desynchronise them. |
| **3** | **No automated verification or logging in the model** | Scope has `DataLogging = off`; no To Workspace blocks | Add logging (as done by `scripts/run_case_study.m`) and assert the checklist of section 6. |
| **4** | **`StopTime = 0.1 s` although start-up ends within 5 ms** | 12 ms takes about 50 s in accelerator mode; 0.1 s takes about 6-7 minutes | 15-20 ms is enough for topology verification; for longer runs lower the carrier frequency rather than loosening the solver. |
| **5** | **Inconsistent midpoint port usage** | Leg0 uses `Vso`, Leg10 uses `Vsi`; the two are internally the same node | Standardise on one port, or document `Vsi == Vso` explicitly. |
| **6** | **`parameters_for_AMB_topology.m` starts with `clearvars`** | it wipes the caller's workspace | Use `clearvars -except ...` or wrap it in a function when combining with other scripts. |
| **7** | **No margin check for the series topology** | there is no `U_dc >= sum(R*i_k)` or `sum(abs(u_k)) <= 1` check | Add a margin calculation and print it from the parameter script. |
| **8** | Content omitted on purpose | no bus capacitance or source impedance, no junction temperature, no mutual coupling or saturation, no motion or position loop | Correct trade-off for topology and current-control verification; **do not** add these for the sake of realism. They are required only when device selection or thermal design becomes the question, and that needs a different model. |

---

## 6. Verification checklist (ready to use)

| # | Check | Criterion | Measured here |
|---|---|---|---|
| 1 | chain KCL: `I_leg(m) = i_m - i_(m-1)` | residual < 1e-3 A | **< 5e-5 A - PASS** |
| 2 | modulation law `b_j = e_j + 0.5 - 0.5(min e + max e)` | `max(b) - min(b) <= 1` | 0.3026 (max 0.5098) - PASS |
| 3 | gate duty `d_j = b_j - Td/Ts` | deviation = Td/Ts | 0.01994 = 1 us / 50 us - PASS |
| 4 | winding voltage command `u_k = b_k - b_(k+1)` | consistent with `(R*i_k + device drops)/U_dc` | verified term by term in section 3.4 - PASS |
| 5 | steady-state accuracy | error < 1 % | control current <= 0.17 %, bias <= 0.07 % - PASS |
| 6 | unidirectionality | all winding currents > 0 | minimum 4.359 A - PASS |
| 7 | common/differential decoupling | common-mode disturbance < 1 % during a differential step | < 1e-4 % - PASS |
| 8 | crosstalk between DOFs | < 1 % | 0.5 % - PASS |
| 9 | step response | consistent with `f_c`, overshoot < 10 % | tr = 0.1 ms, overshoot 5 % - PASS |
| 10 | inner-leg current equals the control current | `+/-i_ref` | +/-1.0000 A - PASS |
| 9b | end-leg current equals the bias | `I_b +/- 0.5*i_ref` | +5.4991 / -4.4968 A - PASS |
| 11 | power balance | input = copper loss + device loss | 274.8 = 252.3 + 22.5 W - PASS |
| 12 | algebraic loop | no warning | none - PASS |
| 13 | reverse control current (`i_ref < 0`) | unidirectionality, inner legs reverse, no loss of control | verified separately: -1 A gives -0.9994 A (-0.06 %) - PASS |
| 14 | modulation boundary (`i_ref` towards `+/-2*I_b`) | observe the linearity limit and waveform distortion | see `case-series-winding-modulation-boundary.md` |

---

## 7. How to reproduce

```matlab
% cd to the folder containing the bundled .slx (the model/ folder of this skill)
run('parameters_for_AMB_topology.m')   % MUST run first - it starts with clearvars
m = 'Series_winding_with_RL_load';
load_system(m); out = sim(m);          % as delivered: StopTime = 0.1 s
```

The instrumented copy used for every measurement in this report is created on demand by
`scripts/run_case_study.m`, which adds the To Workspace taps (11 modulation thresholds, 11 gate
signals, 10 winding currents) and enables full Simscape logging. **The bundled model itself is never
modified**: the script works on a renamed copy in the system temp folder.

| Path | Content |
|---|---|
| `model/` | the bundled model and its parameter script |
| `assets/` | fig1 topology sketch, fig2 start-up, fig3 steady-state zoom, fig4 leg currents |
| `scripts/verify_model_facts.m` | asserts every block, parameter and netlist entry quoted here |
| `scripts/run_case_study.m` | reproduces and regresses the numbers of section 4 |

---

## 8. One-sentence summary

The model correctly and completely implements an **11-leg same-direction series chain with per-winding bias/differential PI control and cumulative-sum min-max common-mode-injection modulation**; the measured control-current error is at most 0.17 %, the KCL relation matches point by point, and together they demonstrate the core value of this topology - **one extra leg replaces nine switches, and the inner legs carry only the control current**. The main improvement opportunities are the roughly 50 % control-current ripple, the eleven independently instantiated carriers, and the absence of automated verification and logging.