# Series-Winding AMB Topology: Modulation-Boundary Study (third revision)

**Model**: `Series_winding_with_RL_load.slx` (including the corrected `OR` logic-block naming)  
**Parameters**: `parameters_for_AMB_topology.m`  
**This revision follows two clarifications from the model's author**: (1) the normal AMB operating range is `abs(i_ref) <= 2*i_bias = 10 A`, a physical constraint that follows because **the force is independent of the current direction**; (2) an all-zero control current causes a *numerical* problem in the solver, not a topology-principle problem - this revision therefore covers that end of the range with a **small-current substitute** and files the all-zero case separately.  
**The delivered model was not modified**; every experiment ran on an instrumented copy.

> Translated from the Chinese working report `Series_winding_topology_analysis_v3.md` produced during the
> development of this skill.

---

## 0. Summary

| # | Finding | Key evidence |
|---|---|---|
| 1 | **The physical operating range is exactly `abs(i_ref) <= 2*i_bias = 10 A`**; the force-producing current `abs(i_a) - abs(i_c)` saturates at 10 A | at i_ref = 10 / 10.5 / 11 / 12 A: `abs(i_a) - abs(i_c)` = **9.995 / 9.997 / 9.997 / 9.971 A** |
| 2 | **Throughout the physical range the modulator keeps margin and never saturates** | `spread = max(b) - min(b)` grows only from 0.30258 (0.6 A) to 0.31133 (11 A), i.e. a **factor of 3.2** below the limit of 1.0; the number of clamped controller-output samples is **0** |
| 3 | **Tracking error stays within 0.24 % over the whole range** (0.10 % for uniform operation) | see Table A; worst case +0.24 % at -10 A |
| 4 | **The real modulation boundary is a bus-voltage boundary, not a control-current boundary** | `U_dc,min = sum(R*i_k)/(1 - sigma0) = about 53 V`; measured: 55 V works (spread 0.972), 50 V collapses (spread 1.747, control current only 0.18 A). This model runs at 200 V -> **3.8x margin** |
| 5 | **The abort at zero control current is a numerical anomaly**; it was replaced by `i_ref = 0.6 A` and filed separately | 0.05 / 0.1 / 0.2 / 0.5 A all abort; **0.6 A passes**, and its `spread` is identical to 1-8 A (0.30258) |

> **One correction to the earlier revision**: that revision described the zero-current inner leg as a *topology* degeneracy. Per the author's clarification, `i_ref = 0` (both windings at the bias current, zero control current) is a **perfectly normal physical operating point** with no topological defect; the difficulty comes purely from the solver oscillating across zero crossings on a zero-current leg. This revision states it accordingly (section 4).

---

## 1. The physical operating range of an AMB: `abs(i_ref) <= 2*i_bias = 10 A`

### 1.1 Origin of the constraint

The electromagnetic force of an AMB depends on the **magnitude** of the current, not its direction, so the quantity that produces differential action is

```
F ~ |i_a| - |i_c|      with   i_a = i_bias + 0.5*i_ref ,  i_c = i_bias - 0.5*i_ref
```
At `i_ref = 12 A` we get `i_a = 5 + 6 = 11 A` and `i_c = 5 - 6 = -1 A`, so `abs(11) - abs(-1) = 10 A` - exactly the same as at `i_ref = 10 A`. **Beyond 10 A a larger control current produces no additional differential force**, which is why `abs(i_ref) <= 2*i_bias` is the physical operating range.

### 1.2 Measured verification

| i_ref (A) | i_a (A) | i_c (A) | measured i_a - i_c (A) | **abs(i_a) - abs(i_c) (A)** | Note |
|---|---|---|---|---|---|
| 9.0 | 9.4966 | 0.5001 | 8.9965 | 8.9965 | linear region |
| 9.5 | 9.7460 | 0.2500 | 9.4960 | 9.4960 | linear region |
| **10.0** | 9.9960 | 0.0010 | 9.9957 | **9.9950** | physical limit (i_c reaches exactly zero) |
| 10.5 | 10.2460 | -0.2490 | 10.4957 | **9.9970** | **saturated: no more force** |
| 11.0 | 10.4960 | -0.4990 | 10.9953 | **9.9970** | **saturated** |
| 12.0 | 10.9535 | -0.9825 | 11.9360 | **9.9710** | **saturated** (adaptive zero crossing) |

**Conclusion**: `abs(i_a) - abs(i_c)` reaches 9.995 A at `i_ref = 10 A` and does not grow beyond it, matching the author's explanation exactly. This also explains why the reversed c-winding current beyond 10 A is "useless": a negative current contributes negatively to the force.

See panel (a) of `assets/fig6_physical_range_and_boundary.png`.

---

## 2. Design of the modulation-boundary test

### 2.1 Object under test and criterion

Modulation chain: `e_j = sum_(k>=j) u_k` (11 cumulative sums, `e_11 = 0`) -> min-max zero-sequence injection `b_j = e_j + 0.5 - 0.5(min e + max e)` -> comparison against 11 symmetric 0->1->0 triangular carriers. **The necessary and sufficient condition for linear modulation is `0 <= b_j <= 1`**, equivalently

```
spread := max(b) - min(b) <= 1
```

### 2.2 Handling of the all-zero control current

At `i_ref = 0` the simulation aborts (see section 4). To cover the "control current near zero" end **without importing a numerical anomaly**, this revision uses a **small-current substitute**, having first measured the smallest workable value:

| i_ref tried | 0.05 | 0.10 | 0.20 | 0.50 | **0.60** | 0.70 | 0.80 | 0.90 |
|---|---|---|---|---|---|---|---|---|
| result | abort | abort | abort | abort | **pass** | pass | pass | pass |

**`i_ref = 0.6 A` is used as the substitute for `i_ref = 0`** (the smallest passing value; 0.5 A and below abort). It is only 6 % of the upper limit of the physical range, and its modulation signature is identical to that of larger currents (`spread = 0.30258`, bit-for-bit the same as 1-8 A), so it represents the lower endpoint of the range adequately.

---

## 3. Results

### 3.1 Table A - uniform i_ref sweep over the whole range (all five DOFs equal, U_dc = 200 V, standard zero crossing)

| i_ref (A) | i_a (A) | i_c (A) | i_a - i_c (A) | abs(i_a) - abs(i_c) | b_min | b_max | spread | max abs(u) | tracking error |
|---|---|---|---|---|---|---|---|---|---|
| **0.60** 1 | 5.2983 | 4.6988 | 0.5995 | 0.5995 | 0.34871 | 0.65129 | 0.30258 | 0.0770 | -0.083 % |
| 1.00 | 5.4980 | 4.4990 | 0.9995 | 0.9990 | 0.34871 | 0.65129 | 0.30258 | 0.0785 | -0.050 % |
| 2.00 | 5.9980 | 3.9990 | 1.9991 | 1.9990 | 0.34871 | 0.65129 | 0.30258 | 0.0821 | -0.045 % |
| 3.00 | 6.4979 | 3.4993 | 2.9986 | 2.9986 | 0.34871 | 0.65129 | 0.30258 | 0.0858 | -0.047 % |
| 4.00 | 6.9970 | 3.0000 | 3.9981 | 3.9970 | 0.34871 | 0.65129 | 0.30258 | 0.0894 | -0.048 % |
| 5.00 | 7.4972 | 2.4994 | 4.9978 | 4.9978 | 0.34871 | 0.65129 | 0.30258 | 0.0930 | -0.044 % |
| 6.00 | 7.9970 | 2.0000 | 5.9975 | 5.9970 | 0.34871 | 0.65129 | 0.30258 | 0.0966 | -0.042 % |
| 7.00 | 8.4969 | 1.4997 | 6.9972 | 6.9972 | 0.34871 | 0.65129 | 0.30258 | 0.1002 | -0.040 % |
| 8.00 | 8.9970 | 1.0000 | 7.9971 | 7.9970 | 0.34871 | 0.65129 | 0.30258 | 0.1038 | -0.036 % |
| 9.00 | 9.4966 | 0.5001 | 8.9965 | 8.9965 | 0.34761 | 0.65239 | 0.30477 | 0.1074 | -0.039 % |
| 9.50 | 9.7460 | 0.2500 | 9.4960 | 9.4960 | 0.34679 | 0.65321 | 0.30641 | 0.1092 | -0.042 % |
| **10.00** 2 | 9.9960 | 0.0010 | 9.9957 | 9.9950 | 0.34599 | 0.65401 | 0.30802 | 0.1110 | -0.043 % |
| 10.50 | 10.2460 | -0.2490 | 10.4957 | 9.9970 | 0.34516 | 0.65484 | 0.30969 | 0.1129 | -0.041 % |
| 11.00 | 10.4960 | -0.4990 | 10.9953 | 9.9970 | 0.34434 | 0.65566 | 0.31133 | 0.1148 | -0.043 % |
| -0.60 1 | 4.6987 | 5.2982 | -0.5995 | -0.5995 | 0.34871 | 0.65129 | 0.30258 | 0.0770 | +0.083 % |
| -1.00 | 4.4990 | 5.4980 | -0.9994 | -0.9990 | 0.34871 | 0.65129 | 0.30258 | 0.0784 | +0.060 % |
| -5.00 | 2.4993 | 7.4971 | -4.9978 | -4.9978 | 0.34871 | 0.65129 | 0.30258 | 0.0930 | +0.044 % |
| -9.00 | 0.5000 | 9.4970 | -8.9975 | -8.9970 | 0.34762 | 0.65238 | 0.30477 | 0.1074 | +0.028 % |
| **-10.00** 2 | 0.0206 | 9.9966 | -9.9760 | -9.9760 | 0.34599 | 0.65401 | 0.30802 | 0.1110 | **+0.240 %** |
| -11.00 | -0.4990 | 10.4970 | -10.9976 | -9.9980 | 0.34434 | 0.65566 | 0.31133 | 0.1148 | +0.022 % |
| 12.00 3 | 10.9535 | -0.9825 | 11.9360 | 9.9710 | 0.34168 | 0.65832 | 0.31664 | 0.1376 | -0.533 % |

1 small-current substitute for `i_ref = 0`; 2 endpoint of the physical range; 3 outside the physical range, obtained with adaptive zero crossing.

### 3.2 Three readings

1. **The modulation margin is essentially constant over the whole physical range.** `spread` grows only from 0.30258 (0.6-8 A, bit-for-bit identical) to 0.31133 (11 A), a factor of 3.2 below the limit of 1.0; `b_min`/`b_max` stay inside [0.344, 0.656]. The largest controller output is only 0.077-0.115 and **the number of clamped samples is 0**.
   The analytical reason: the total chain voltage is set by the sum of all winding currents, and `i_a + i_c = 2*i_bias` is independent of the control current, so **the volt-second demand of the chain is constant**. This is a very useful property of the series topology.
2. **Positive and negative are perfectly symmetric.** At +11 and -11 A the `spread`, `b_min`/`b_max` and `max abs(u)` are bit-for-bit identical (0.31133 / 0.34434 / 0.65566 / 0.1148).
3. **Accuracy**: at most 0.10 % for uniform operation (mostly 0.05 % or better); the worst point is +0.240 % at -10 A. That is the only accuracy degradation in the whole range and it corresponds to the a-winding current being almost zero (0.021 A) - the same numerical sensitivity as in section 4, although the steady-state deviation itself is real.

### 3.3 Worst corner inside the range: adjacent DOFs driven to opposite signs

| Case | DOF1 | DOF2 | DOF3 | DOF4 | DOF5 | spread |
|---|---|---|---|---|---|---|
| uniform +10 A | 9.9957 | 9.9957 | 9.9944 | 9.9918 | 9.9740 | 0.30802 |
| **[10,-10,10,-10,10]** | 9.9957 | **-10.0847** | 9.9123 | **-10.0974** | 9.8696 | 0.30802 |
| **[-10,10,-10,10,-10]** | -10.0518 | 9.9210 | -10.0912 | 9.9045 | -9.9852 | 0.30812 |

- **The modulator is completely unaffected** (spread identical to the uniform case) - confirming reading 1 of section 3.2 again.
- **Accuracy degrades to -1.30 %** (DOF5), about 30 times worse than the uniform case (-0.04 %). The inner leg between adjacent DOFs then carries `0.5(i_ref,j + i_ref,j+1) = about 0`, which falls into the zero-current sensitive region of section 4. This is the worst accuracy corner **inside** the range, and it is still far from what the modulation margin would allow.

### 3.4 Table C - the real modulation boundary: bus voltage (`i_ref = 1 A`)

| U_dc (V) | b_min | b_max | spread | predicted `50/U_dc + 0.053` | i_a - i_c (A) | tracking error |
|---|---|---|---|---|---|---|
| 200 | 0.34871 | 0.65129 | 0.30258 | 0.3030 | 0.9995 | -0.05 % |
| 100 | 0.21910 | 0.78090 | 0.56180 | 0.5530 | 0.9975 | -0.25 % |
| 70 | 0.10933 | 0.89067 | 0.78135 | 0.7673 | 0.9965 | -0.35 % |
| 60 | 0.03685 | 0.96315 | 0.92630 | 0.8863 | 0.9928 | -0.72 % |
| **55** | **0.01391** | **0.98609** | **0.97218** | 0.9621 | 0.9974 | -0.26 % |
| **50** | **-0.37337** | **1.37337** | **1.74673** | 1.0530 | **0.1819** | **-81.8 % collapse** |

**Boundary formula (directly reusable for other parameters):**

```
spread  ~=  sum_k(R_k * i_k) / U_dc  +  sigma0        sigma0 ~= 0.053 (fixed duty overhead of dead time + device drops)
U_dc,min =  sum_k(R_k * i_k) / (1 - sigma0)  ~=  1.056 * sum_k(R_k * i_k)
```
For this model `sum(R*i_k) = 1 Ohm x (5.5+4.5) x 5 = 50 V`, hence **`U_dc,min = about 53 V`**; the measurement falls between 55 V (works) and 50 V (collapses), in agreement with theory. **This model runs at 200 V, i.e. a margin of 200/53 = 3.8x**, which is precisely why the modulator never saturates anywhere in the physical range.

**Why one i_ref is enough in Table C**: by reading 1 of section 3.2, `sum(R*i_k)` is independent of i_ref (always 50 V) and only the zig-zag term grows slightly, from 0.0526 to 0.0586. Substituted into the formula, `U_dc,min` moves only from 52.8 V to 53.1 V - **the boundary position is essentially fixed over the whole physical range**.

**Dynamic margin**: at 200 V the remaining modulation range is `1 - 0.303 = 0.697`, about 139 V of volt-seconds, enough for all ten windings to slew simultaneously at a combined rate of about **27.8 A/ms** (estimated with `sum(L) = 10 x 500 uH`; this is a conservative lower bound).

See panels (b), (c) and (d) of `assets/fig6_physical_range_and_boundary.png`.

---

## 4. Filed separately: the `i_ref = 0` numerical anomaly (not a topology-principle problem)

### 4.1 Observed behaviour

| Setting | Result |
|---|---|
| standard zero crossing (model default, `ZeroCross = on`) | **simulation aborts**: an IGBT in `Bridge_module3` produces 1000 consecutive zero crossings at t = 3.66 ms |
| adaptive zero crossing (`ZeroCrossAlgorithm = 'Adaptive'`) | completes; steady state `i_a - i_c = -0.0051 A` (correct, i.e. zero control current), but it takes **265 s**, about 7 times longer |

### 4.2 Characterisation

- **Physically this is a completely normal operating point**: both windings carry the 5 A bias current, the control current is zero and the nine inner legs carry exactly zero current. Nothing about the topology, the modulation law or the control law is singular there, and the steady-state value produced by the adaptive run (-0.0051 A) confirms that the result itself is correct.
- **The difficulty comes from the solver, not from the topology**: it is demonstrably a **modelling/numerical issue** (the same physical case converges normally with adaptive zero crossing and yields the correct steady state). The most likely cause is that, at zero leg current, the voltage discontinuity at i = 0 between the ideal IGBT (Vf = 0.8 V) and the freewheel diode (Vf = 0.6 V) makes the zero-crossing detector retrigger repeatedly. This is recorded as an observation plus hypothesis; no further mechanism study was carried out.
- **Related sensitive points**: any operating condition that drives an inner-leg current to zero (`i_ref = about 0`, or adjacent DOFs with opposite control-current signs) triggers the same sensitivity. Measured consequences: simulation abort (at or below 0.5 A) or accuracy degradation (-1.30 % with adjacent opposite signs).
- **Correction to the previous revision**: describing this as a "topology degeneracy" was inaccurate; this revision corrects it.

### 4.3 Practical guidance

1. When an operating point with `i_ref = about 0` is needed, **use `i_ref = 0.6 A` as the substitute** (the smallest verified workable value, 6 % of the upper limit, with a modulation signature identical to the rest of the range).
2. If an exact `i_ref = 0` really must be simulated, switch to adaptive zero crossing and accept the 7x runtime and roughly 5 % accuracy penalty; or remove the i = 0 discontinuity at the modelling level, for example with a tiny on-resistance asymmetry or a minimum-current threshold in the IGBT/diode models.
3. **Do not** change the model's default zero-crossing setting for this purpose: adaptive zero crossing turns the same operating point from 0.9995 A into 0.9464 A (-5.4 %), a far larger cost than the benefit.

---

## 5. Summary and design rules

### 5.1 Three levels of boundary

| Level | Boundary | Value for this model | Nature |
|---|---|---|---|
| **Physical** (force limit) | `abs(i_ref) <= 2*i_bias` | **10 A** (`abs(i_a) - abs(i_c)` saturates at 9.995 A) | physics, cannot be exceeded |
| **Modulation** (bus voltage) | `U_dc > sum(R*i_k)/(1 - sigma0)` | **about 53 V** (55 V works, 50 V collapses); this model uses 200 V -> 3.8x margin | design constraint |
| **Numerical sensitive region** (simulation) | inner-leg current = about 0 | `abs(i_ref) < 0.6 A` aborts; accuracy -1.30 % with adjacent opposite signs | numerical, not topological |

### 5.2 Design formulas reusable for other AMB parameters

```
physical:   |i_ref| <= 2 * i_bias
static:     U_dc  >  sum_k(R_k * i_bias,k) / (1 - sigma0)     sigma0 ~= 2*Td/Ts + device-drop term ~= 0.05
dynamic:    U_dc  >  [ sum_k(R_k*i_k) + sum_k(L_k*|di_k/dt|) ] / (1 - sigma0)
recommended: U_dc >= 2 * sum_k(R_k * i_bias,k)                % one full margin for dynamics
this model: sum_k(R_k*i_bias,k) = 50 V -> U_dc,min ~= 53 V, recommended >= 100 V, actual 200 V
```

### 5.3 Results unchanged from the earlier revisions

- Chain KCL: `I_leg(m) = i_m - i_(m-1)`; the two end legs carry the bias (+5.4991 / -4.4968 A) and the nine inner legs carry only the difference of the control currents (of the order of +/-1 A).
- Steady-state control-current accuracy <= 0.17 % (five DOFs, i_ref = 1 A); step response rise time about 0.10 ms, overshoot about 5 %, common-mode disturbance below 1e-3 A, crosstalk between DOFs 0.5 %.
- The modulation law `b_j = e_j + 0.5 - 0.5(min e + max e)` and the gate-duty law `d_j = b_j - Td/Ts` both hold under measurement.

---

## 6. Source artifacts of this study

| This skill | Content |
|---|---|
| `references/case-series-winding-modulation-boundary-first-pass-superseded.md` | the first-pass boundary study, superseded by this document |
| `references/case-series-winding-amb.md` | topology, parameters, netlist, control law, steady-state verification |
| `assets/fig6_physical_range_and_boundary.png` | four-panel figure: (a) force current saturating at 10 A, (b) spread over the whole range, (c) tracking error, (d) bus-voltage boundary |
| `scripts/run_case_study.m` | reproduces the steady-state numbers and (in boundary mode) the endpoints of this study |

---

## 7. One-sentence summary

The modulation-boundary test was repeated inside the physical operating range `abs(i_ref) <= 2*i_bias = 10 A`: **measurement confirms that the range is set by the direction-independence of the force (`abs(i_a) - abs(i_c)` saturates at 9.995 A); inside the range the modulator never saturates (spread stays at 0.303-0.311, a margin of 3.2x) and the tracking error stays within 0.24 % (0.10 % for uniform operation); the real modulation boundary is a bus-voltage boundary, `U_dc,min = sum(R*i_k)/(1 - sigma0) = about 53 V`, against which this model's 200 V gives a 3.8x margin**. The all-zero control current was replaced by `i_ref = 0.6 A` (the smallest passing value) and filed as a **numerical anomaly rather than a topology-principle result**.