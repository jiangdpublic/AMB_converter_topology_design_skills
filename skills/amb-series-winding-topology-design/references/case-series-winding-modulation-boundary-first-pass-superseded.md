> **SUPERSEDED.** This first-pass boundary study is replaced by
> `case-series-winding-modulation-boundary.md`. Its central claim - that a zero-current inner leg is a
> *topology* degeneracy - is wrong: measurement on the same model shows that `i_ref = 0` is a perfectly
> normal physical operating point and that the difficulty is purely numerical, coming from the solver
> oscillating across zero crossings. The document is kept unchanged below for the audit trail; read it
> together with the correction in section 4 of the replacement document.

# Series-Winding AMB Topology: Modulation-Boundary Study and Analysis (second revision)

**Model**: `Series_winding_with_RL_load.slx` (updated with the author's correction, R2025b / Simscape Electrical)  
**Parameters**: `parameters_for_AMB_topology.m`  
**New in this revision**: the **modulation-boundary test** performed on the corrected model (control-current sweep +/-11 A plus a bus-voltage sweep)  
**Note**: the delivered model was not modified; every experiment ran on an instrumented copy.

> Translated from the Chinese working report `Series_winding_topology_analysis_v2.md` produced during the
> development of this skill.

---

## 0. Confirmation of the model correction

| Item | Status |
|---|---|
| naming of the logic blocks inside `PI_ctrl/Ictrl_anti_windup` | corrected: `AND` (Operator = AND) + `OR` (Operator = OR) |
| the same subsystem in the other nine controllers `PI_ctrl1..9` | 10 of 10 now use the correct `AND`/`OR` naming (counted one by one by script) |
| anti-windup logic | unchanged, still `(abs(x) >= 1) AND (x*err >= 0) -> freeze the integrator`; semantics and naming now agree |
| topology, modulation and controller parameters | unchanged (the baseline reproduction of this sweep matches the previous revision exactly: i_a = 5.4982, i_c = 4.4988, b in [0.34871, 0.65129]) |

> The three items previously listed as "improvement opportunities" - the diode Ron, the 1 us dead time
> and the start-up kick - are **no longer listed as problems**, following the author's instruction. In this
> revision the **dead time and the device drops** are carried along as **existing properties** of the
> model: they are exactly what produces the fixed offset term sigma0 in the modulation boundary
> (section 3.3).

---

## 1. Summary

### 1.1 The three results of the modulation-boundary test

1. **The modulator margin is almost independent of i_ref.** Over the whole control-current range (`abs(i_ref) <= 11 A`) the thresholds always satisfy `b` in [0.344, 0.656] and `spread = max(b) - min(b)` grows only from **0.3026 to 0.3113** (+2.9 %), still **3.2 times** below the limit of 1.0. The controller output is **never clamped** (zero samples with `abs(u) = 1`).
   **Reason (analytical)**: the total chain voltage is set by the sum of all winding currents, and `i_a + i_c = 2*i_bias` is independent of the control current, so the volt-second demand of the chain is constant - a very attractive property of the series topology.

2. **The real modulation boundary is a bus-voltage boundary, not a control-current boundary.** Lowering the bus voltage is what finally produces saturation:

   | U_dc | 200 V | 100 V | 70 V | 60 V | **55 V** | **50 V** |
   |---|---|---|---|---|---|---|
   | spread | 0.303 | 0.562 | 0.781 | 0.926 | **0.972** | **1.747 - out of range** |
   | b_min / b_max | 0.349 / 0.651 | 0.219 / 0.781 | 0.109 / 0.891 | 0.037 / 0.963 | **0.014 / 0.986** | **-0.373 / 1.373** |
   | control-current error | -0.05 % | -0.25 % | -0.35 % | -0.72 % | **-0.26 %** | **-81.8 % (collapse)** |

   **Boundary formula: `spread = sum(R*i_k)/U_dc + sigma0` with sigma0 = 0.053, giving `U_dc,min = sum(R*i_k)/(1 - sigma0) = about 53 V`.** The measurement falls between 55 V and 50 V, in agreement with theory. Here `sum(R*i_k) = 50 V` (ten windings x about 5 A x 1 Ohm), so the bus margin is **200/53 = about 3.8x**.

3. **The linear-tracking boundary of the control current is the bias linearity limit, `abs(i_ref) < 2*i_bias = 10 A`, while the loop itself can track further.** Measured error of `i_a - i_c` against its reference: **-0.02 % to -0.06 %** over the whole `abs(i_ref) <= 11 A` range (including the author-verified `i_ref = -1 A`: -0.9994 A, error -0.06 %). Beyond 10 A the c-winding current reverses (10.5 A -> -0.249 A; 11 A -> -0.499 A; 12 A -> -0.982 A) and the loop still tracks (error -0.53 % at 12 A), but **the premise of a biased AMB is broken** because the winding currents are no longer unidirectional.

### 1.2 Three "degenerate cases" found along the way

*[Superseded: the framing below is wrong. These are numerical difficulties at a normal physical operating point, not topology degeneracies. See section 4 of `case-series-winding-modulation-boundary.md`.]*

| Case | Symptom | Suggested mechanism |
|---|---|---|
| **i_ref = 0** | with the standard zero-crossing setting the **simulation aborts** (1000 consecutive zero crossings); with adaptive zero crossing it completes but takes 265 s (about 7x) | here `I_leg = -i_ref = 0`, i.e. **the nine inner legs carry exactly zero current**, and the ideal switches plus dead time keep toggling at zero current |
| **i_ref = +/-12** | with the standard setting the run aborts immediately at start-up (t = 0.575 ms, Leg6); adaptive zero crossing completes | during start-up the not-yet-enabled DOFs have a zero reference, so their adjacent inner-leg currents are zero; the larger i_ref makes the transient more violent and pushes those zero-current legs into zero-crossing oscillation |
| **adjacent DOFs with opposite control-current signs** (e.g. [10,-10,10,-10,10]) | the simulation runs, but accuracy degrades: worst error **-1.30 %** against -0.04 % for a uniform 10 A | with opposite signs the inner leg carries `I_leg = 0.5(i_j + i_(j+1))`, i.e. **around 0 A**, and the same zero-current mechanism amplifies the error roughly 30-fold |

**Combined conclusion**: the inner-leg current of this topology is set by **the difference of the adjacent winding currents**, so **the inner-leg current is zero whenever the adjacent winding currents are equal**. A zero-current leg is (a) the hardest operating point to simulate and (b) the most fragile in accuracy. In practice one should keep a deliberate control-current bias or avoid simultaneous zero crossings on adjacent DOFs, and not operate for long near `i_ref = about 0`.

---

## 2. Topology and leg currents (re-check, consistent with the first revision)

### 2.1 Chain KCL

`I_leg(m) = i_m - i_(m-1)`; the chain order here is a1,c1,a2,c2,...,a5,c5 (a and c of one DOF adjacent), hence:

| Leg position | Current | Measured (i_ref = 1 A) |
|---|---|---|
| end leg Leg0 | `i_a1 = I_b + 0.5*i_ref` | **+5.4991 A** |
| Leg1,3,5,7,9 (between a_j and c_j) | `-i_ref` | **-1.0000 A** |
| Leg2,4,6,8 (between c_j and a_(j+1)) | `0.5(i_ref,j + i_ref,j+1)` | **+0.9996 A** |
| end leg Leg10 | `-i_c5` | **-4.4968 A** |

KCL residual below 5e-5 A. **Only the two end legs carry the bias current (about 5 A); the nine inner legs carry only the control current (+/-1 A).**

### 2.2 Comparison with the alternating-direction series chain (corrected wording)

| Item | **Same-direction (this model)** | Alternating-direction series chain |
|---|---|---|
| inner-leg current | **difference of the two adjacent winding currents**: `-i_ref` between a and c, `0.5(i_ref,j + i_ref,j+1)` between adjacent DOFs | sum of the two adjacent winding currents: `2*I_b` |
| typical value (I_b = 5 A, i_ref = 1 A) | **1 A** (up to 10 A in the extreme opposite-sign corner) | **10 A** (independent of i_ref) |
| current direction | reverses with i_ref -> **must be bidirectionally controllable** | always positive -> **one switch can be replaced by a diode** |
| zero-current degenerate point | **exists** (i_ref = 0 or adjacent opposite signs) | does not exist (current is always 2*I_b) |
| end legs | `I_b +/- 0.5*i_ref` (about 5 A) | same |

### 2.3 Hard constraints of the series topology

1. **The bus voltage must cover the resistive drop of the whole chain**: `U_dc > sum(R*i_k)/(1 - sigma0)`. Here sum(R*i_k) = 50 V -> U_dc,min = about 53 V (measured in this revision, see section 3).
2. **The modulation cumulative sums must stay inside [0,1]**: `spread = max(e) - min(e) <= 1`; for this model it is always 0.30-0.31 (see section 3).
3. **The chain is a single point of failure**: if one winding opens, all ten windings lose control.

---

## 3. Modulation-boundary test (the core of this revision)

### 3.1 Criterion

Modulator structure: `e_j = sum_(k>=j) u_k` (11 cumulative sums, `e_11 = 0`) -> min-max zero-sequence injection `b_j = e_j + 0.5 - 0.5(min e + max e)` -> comparison against 11 symmetric triangular carriers. For a 0->1->0 symmetric triangle **the comparator duty equals the threshold `b_j`**, so the necessary and sufficient condition for linear modulation is

```
0 <= b_j <= 1   for all j      <=>      spread := max(b) - min(b) <= 1
```

The test also records `b_min`, `b_max`, `spread`, the largest `spread` inside the window (including ripple), the largest `abs(u_k)`, the number of clamped controller-output samples, and the measured error of `i_a - i_c` against its reference.

### 3.2 Control-current sweep (U_dc = 200 V, all five DOFs equal, +/-11 A)

| i_ref (A) | i_a (A) | i_c (A) | i_a - i_c (A) | b_min | b_max | spread | max abs(u) | clamped | error |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 5.4980 | 4.4990 | 0.9995 | 0.34871 | 0.65129 | 0.30258 | 0.0785 | 0 | -0.050 % |
| 2 | 5.9980 | 3.9990 | 1.9991 | 0.34871 | 0.65129 | 0.30258 | 0.0821 | 0 | -0.045 % |
| 4 | 6.9970 | 3.0000 | 3.9981 | 0.34871 | 0.65129 | 0.30258 | 0.0894 | 0 | -0.048 % |
| 6 | 7.9970 | 2.0000 | 5.9975 | 0.34871 | 0.65129 | 0.30258 | 0.0966 | 0 | -0.042 % |
| 8 | 8.9970 | 1.0000 | 7.9971 | 0.34871 | 0.65129 | 0.30258 | 0.1038 | 0 | -0.036 % |
| 9.5 | 9.7460 | 0.2500 | 9.4960 | 0.34679 | 0.65321 | 0.30641 | 0.1092 | 0 | -0.042 % |
| **10 = 2*i_bias** | 9.9960 | **0.0010** | 9.9957 | 0.34599 | 0.65401 | 0.30802 | 0.1110 | 0 | -0.043 % |
| 10.5 | 10.2460 | **-0.2490** | 10.4957 | 0.34516 | 0.65484 | 0.30969 | 0.1129 | 0 | -0.041 % |
| 11 | 10.4960 | **-0.4990** | 10.9953 | 0.34434 | 0.65566 | 0.31133 | 0.1148 | 0 | -0.043 % |
| -1 | 4.4990 | 5.4980 | -0.9994 | 0.34871 | 0.65129 | 0.30258 | 0.0784 | 0 | +0.060 % |
| -9 | 0.5000 | 9.4970 | -8.9975 | 0.34762 | 0.65238 | 0.30477 | 0.1074 | 0 | +0.028 % |
| -11 | -0.4990 | 10.4970 | -10.9976 | 0.34434 | 0.65566 | 0.31133 | 0.1148 | 0 | +0.022 % |

**How to read it**:

- **Positive and negative are perfectly symmetric**: at i_ref = +11 and -11 the `spread`, `b_min`/`b_max` and `max abs(u)` are identical (0.31133 / 0.34434 / 0.65566 / 0.1148).
- **The spread hardly moves with i_ref**: 0.3026 -> 0.3113. The reason is that the two ends of `b_j` are set by the total chain voltage and by the spread respectively, and the total chain voltage `sum(R*i_k)/U_dc = 50/200 = 0.25` is independent of i_ref because `i_a + i_c = 2*i_bias`.
- **Nothing is ever clamped**: over the whole `abs(i_ref) <= 11 A` range the controller output never reaches +/-1.
- **The linear-region boundary is 10 A**: at i_ref = 10 A, i_c = 0.001 A, exactly zero; above 10 A, i_c < 0 and the c-winding current reverses. This is the **principle boundary of a biased AMB**, not a modulator boundary.

See panels (a) and (b) of `assets/fig5_modulation_boundary.png`.

### 3.3 Bus-voltage sweep (i_ref = 1 A) - locating the real modulation boundary

| U_dc (V) | b_min | b_max | spread | predicted `50/U_dc + 0.053` | i_a - i_c (A) | error |
|---|---|---|---|---|---|---|
| 200 | 0.34871 | 0.65129 | 0.30258 | 0.3030 | 0.9995 | -0.05 % |
| 100 | 0.21910 | 0.78090 | 0.56180 | 0.5530 | 0.9975 | -0.25 % |
| 70 | 0.10933 | 0.89067 | 0.78135 | 0.7673 | 0.9965 | -0.35 % |
| 60 | 0.03685 | 0.96315 | 0.92630 | 0.8863 | 0.9928 | -0.72 % |
| **55** | **0.01391** | **0.98609** | **0.97218** | 0.9621 | 0.9974 | -0.26 % |
| **50** | **-0.37337** | **1.37337** | **1.74673** | 1.0530 | **0.1819** | **-81.8 %** |

**Conclusions**:

- `spread` is proportional to `1/U_dc`, matching the analytical form `spread = sum(R*i_k)/U_dc + sigma0`; fitted with sigma0 = 0.053 the largest deviation between 200 V and 55 V is below 8 %.
- **Physical origin of sigma0** (an existing property of this model): the dead time (Td = 1 us, Ts = 50 us, two both-off intervals per period) plus the IGBT/diode drops produce a voltage-command offset measured at about 0.053 (about 10.6 V) at U_dc = 200 V. It is approximately proportional to U_dc, so it appears as a **fixed duty-cycle overhead independent of the bus voltage**.
- **55 V still works** (spread 0.972, b only just inside the range); **50 V collapses**: `b` reaches [-0.373, 1.373], the actual duty is clipped to 0/1, the winding voltage can no longer be established, and the control current falls from 1 A to 0.18 A.
- **Boundary formula**: `U_dc,min = sum(R*i_k)/(1 - sigma0) = 1.056 * sum(R*i_k) = 52.8 V`. Here `sum(R*i_k) = 1 Ohm x (5.5+4.5) x 5 = 50 V`.
- **Bus margin of this model**: 200/52.8 = **3.8x** - the fundamental reason why the modulator never saturates anywhere in the operating range.

**Engineering design rule (directly reusable for other AMB parameters)**:

```
static:      U_dc > sum_k(R_k * i_bias,k) / (1 - sigma0)          sigma0 = 2*Td/Ts + device-drop term
dynamic:     U_dc > sum_k(R_k*i_k + L_k*|di_k/dt|) / (1 - sigma0)  % all windings slewing together
recommended: U_dc >= 2 * sum_k(R_k * i_bias,k)                     % one full margin for dynamics
```
Following the recommendation, 2 x 50 = 100 V would be enough for this model; 200 V is used, which leaves extra dynamic margin: at 200 V the remaining modulation range is 0.697, about 139 V of volt-seconds, enough for all ten windings to slew together at a combined rate of about **27.8 A/ms**.

See panels (c) and (d) of `assets/fig5_modulation_boundary.png`.

### 3.4 The worst corner: adjacent DOFs with opposite signs

| Case | DOF1 | DOF2 | DOF3 | DOF4 | DOF5 | spread |
|---|---|---|---|---|---|---|
| uniform +10 | 9.9957 | 9.9957 | 9.9944 | 9.9918 | 9.9740 | 0.30802 |
| **alternating [10,-10,...]** | 9.9957 | **-10.0847** | 9.9123 | **-10.0974** | 9.8696 | 0.30802 |
| **alternating [-10,10,...]** | -10.0517 | 9.9210 | -10.0912 | 9.9045 | -9.9852 | 0.30812 |

- **The spread is completely unaffected** (0.30802 vs 0.30802) because the total chain current is unchanged - confirming section 3.2 again.
- **But the accuracy degrades to -1.30 %** (DOF5), about 30 times worse than the uniform case (-0.04 %). The reason is that with opposite signs the inner leg between the two DOFs carries `0.5(i_j + i_(j+1)) = about 0`, i.e. it falls into the zero-current region of section 1.2.

### 3.5 Numerical boundary versus physical boundary (adaptive zero-crossing comparison)

To separate "modulation saturation" from "solver difficulty", four cases were re-run with `ZeroCrossAlgorithm = 'Adaptive'`:

| Case | standard zero crossing | adaptive zero crossing | Verdict |
|---|---|---|---|
| i_ref = 1, U_dc = 200 | diff = 0.9995 (-0.05 %) | diff = **0.9464 (-5.4 %)** | the adaptive algorithm itself introduces a 5 % error, so it **must not be used for quantitative conclusions** |
| i_ref = 0, U_dc = 200 | simulation aborts | completes, diff = -0.0051, takes **265 s** | numerically hard, steady state correct |
| i_ref = 12, U_dc = 200 | aborts at start-up | completes, diff = 11.936 (-0.53 %), i_c = -0.98 A | **numerically hard, not physical saturation** |
| i_ref = 1, U_dc = 60 | simulation aborts | spread = 0.926, diff = 0.9928 (-0.72 %) | numerically hard, physically still inside the range |

**Conclusions**:

- **The failures at i_ref = +/-12 and U_dc = 60 V are numerical** (adaptive zero crossing completes with sensible results), not modulation saturation;
- **The failure at U_dc = 50 V is physical** (adaptive zero crossing reproduces the same out-of-range b and current collapse, with `b` in [-0.373, 1.373]);
- **Adaptive zero crossing significantly reduces accuracy** (0.9995 -> 0.9464 for the same case), so **the model's default settings (`ZeroCrossControl = UseLocalSettings`, every block `ZeroCross = on`) should be kept**. This is now supported quantitatively.

---

## 4. Re-check of steady-state and dynamic behaviour

### 4.1 Steady state (i_ref = 1 A, window 10.5-12 ms)

| Quantity | Measured |
|---|---|
| the 10 winding currents | i_a = 5.4961...5.4982 A, i_c = 4.4967...4.4989 A (current-sensor and inductor-current readings agree) |
| control current (five DOFs) | 0.9983 / 0.9986 / 0.9990 / 0.9994 / 0.9995 A (error -0.06 % to -0.17 %) |
| bias | 4.9964...4.9985 A (error <= 0.07 %) |
| unidirectionality | whole-run minimum 4.359 A, far above 0 (at i_ref = 1 A) |
| winding ripple | 0.142-0.427 A pp |
| control-current ripple | 0.475-0.556 A pp |
| total chain duty `d_0 - d_10` | 0.3026 (= spread, consistent with section 3) |

### 4.2 Dynamics (i_ref of DOF2 stepped from 1.0 to 1.5 A at t = 8 ms)

| Metric | Measured |
|---|---|
| rise time | about 0.10 ms (consistent with the f_c = 1500 Hz design) |
| overshoot | about 5 % (for a 0.5 A step) |
| common-mode decoupling | `i_a + i_c` stays between 10.0000 and 10.0008 A (disturbance < 1e-3 A) |
| crosstalk between DOFs | DOF1 dips by 0.5 %, recovers in about 1 ms |
| slow tail | about 1 ms time constant (98.7 % at 8.4 ms, 99.8 % at 10 ms) |

---

## 5. Suggested follow-up verification (incremental to this boundary test)

| # | Item | Purpose |
|---|---|---|
| 1 | re-run every case at U_dc = 100 V (twice the recommended margin) | test whether the "margin >= 2x" rule is sufficient |
| 2 | lower U_dc while recomputing the gains with `K_p proportional to 1/U_dc` | separate the effect of modulation saturation from that of reduced loop gain (the two are coupled here) |
| 3 | sweep in the neighbourhood of i_ref = about 0 (+/-0.05 A) | quantify the width of the numerical/accuracy degradation band around a zero-current leg |
| 4 | dynamic boundary: peak spread during a full-scale step (i_ref from +2*i_bias to -2*i_bias) | validate the dynamic design formula of section 3.3 |
| 5 | whether a minimum-pulse or zero-current-avoidance strategy can be added to the modulator | mitigate the zero-current inner-leg degradation |

---

## 6. Source artifacts of this study

| This skill | Content |
|---|---|
| `references/case-series-winding-amb.md` | first revision: structure, parameters, netlist, control law |
| `assets/fig5_modulation_boundary.png` | four-panel boundary figure of this revision (tracking linearity / spread vs i_ref / modulator headroom vs U_dc / boundary fit) |
| `scripts/run_case_study.m` | reproduces the steady-state numbers; its boundary mode covers the sweep endpoints |

---

## 7. One-sentence summary

The modulation-boundary test was completed on the corrected model: **the modulator never saturates over the whole control-current range (`abs(i_ref) <= 11 A`, spread always 0.30-0.31, a margin of 3.2x) and the real modulation boundary is set by the bus voltage - `spread = sum(R*i_k)/U_dc + sigma0`, giving a boundary of about 53 V for this model (55 V works, 50 V collapses) while the model runs at 200 V, a margin of 3.8x**; linear tracking of the control current stays within 0.06 % up to +/-11 A, and the principle boundary of bias unidirectionality is `abs(i_ref) = 2*i_bias = 10 A`. The study also located a previously unnoticed **zero-current inner-leg region** (`i_ref = about 0`, adjacent DOFs with opposite signs) which is the common root of both the numerical difficulty and the accuracy degradation. *[Superseded: that region is a numerical sensitivity, not a topology degeneracy - see the replacement document.]*