# Case study: the modulation boundary of the ten-phase-nine-leg topology

Companion to `case-ten-phase-nine-leg-amb.md`. This reference derives, measures and regresses the two limits
on the control current: the **physical** limit and the **bus-voltage** limit. All measurements come from
`model/Ten_phase_nine_leg_with_RL_load.slx` (or a copy in the system temp folder) with `R = 1 ohm`,
`L = 500 uH`, `i_bias = 5 A`, `U_dc = 200 V`, `Td = 1 us`, `Ts = 50 us` unless stated otherwise. Scripts:
`scripts/run_case_study.m` (modes `boundary`, `point`, `deadtime`).

Environment: MATLAB R2025b (25.2.0.3312555 Update 6), Simulink 25.2, Simscape 25.2, Simscape Electrical
25.2, Linux glnxa64.

---

## 1. The constraint, exactly

The min-max common-mode injection centres the nine duties on 0.5:

```

duty_k = ubridge_k - 0.5*(max(ubridge) + min(ubridge)) + 0.5
```

so all nine duties lie in [0, 1] **iff**

```

S := max(ubridge) - min(ubridge) <= 1
```

"The maximum duty-cycle range" is therefore exactly the question "does the spread of the nine leg voltage
commands stay below one per-unit". The bias and the control currents do not appear explicitly; they only set
the value of S.

## 2. Closed-form spread

The leg commands are a linear map of the winding commands, `ub = M*u`, with the rows listed in the case
study. At steady state `u_wk = R*i_k/U_dc` with `i_ad = i_bias + delta_d/2` and `i_cd = i_bias - delta_d/2`.
Writing `rho = R/U_dc`:

```

ub1 = rho*[3*i_bias + (d1 - d2 + d3)/2]      ub6 = rho*[-i_bias + d3/2]
ub2 = rho*[3*i_bias + (-d1 - d2 + d3)/2]     ub7 = rho*[-i_bias + (d3 - 2*d4)/2]
ub3 = rho*[i_bias + (d3 - 2*d2)/2]           ub8 = rho*[-3*i_bias + (d3 - d4 - d5)/2]
ub4 = rho*[i_bias + d3/2]                    ub9 = rho*[-3*i_bias + (d3 - d4 + d5)/2]
ub5 = 0
```

The worst pair is always `ub1 - ub8`:

```

ub1 - ub8 = [ 6*R*i_bias + R*(d1 - d2 + d4 + d5)/2 + L*(g1 - g2 + g4 + g5)/2 ] / U_dc
```

where the `L*g` terms appear as soon as the control currents move (`g = d(delta)/dt`). Maximizing over the
admissible box - done exactly, by iterating over all 81 ordered row pairs of `M` and taking the vertex of
the resulting linear form - gives

```

S = [ 6*R*i_bias + k*( R*delta_max + L*gamma_max ) ] / U_dc

k = 1 : all five control currents equal          (delta_d = delta_max)
k = 2 : worst distribution delta = [+, -, *, +, +]*delta_max
```

The three terms have distinct physical origins:

| Term | Origin | Value in the verified model |
|---|---|---|
| `6*R*i_bias` | the bias current must be driven through three winding resistances on each side of the two star nodes | 30 V |
| `k*R*delta_max` | resistive drop of the control current itself | 10 V (k=1) / 20 V (k=2) |
| `k*L*gamma_max` | the volt-seconds needed to slew the control current | 0 in steady state |

Note that `6*R*i_bias` is **independent of the control current**: even with `delta = 0` the bus must supply
six times `R*i_bias` before the bias can circulate.

## 3. Adding dead time and device drops

Dead time removes `2*Td` of conduction time per switching period, i.e. it costs `2*Td/Ts` of the bus
voltage; the device forward drops add a roughly bus-independent `V_ce`. Both land directly on the
*commanded* spread:

```

S_cmd = [ 6*R*i_bias + k*(R*delta_max + L*gamma_max) + V_ce ] / U_dc + 2*Td/Ts  <=  1

U_dc,min     = [ 6*R*i_bias + k*(R*delta_max + L*gamma_max) + V_ce ] / (1 - 2*Td/Ts)
delta_max(U) = [ U_dc*(1 - 2*Td/Ts) - 6*R*i_bias - k*L*gamma_max - V_ce ] / (k*R)
```

Fitted to the measurements, `V_ce ~= 1.9 V` and `2*Td/Ts = 0.04`.

## 4. Measured spread versus control current (U_dc = 200 V)

All five DOFs driven with the same `i_ref`, references applied from t = 0, 6 ms runs, metrics over the last
1 ms:

| `i_ref` [A] | 0 | 2 | 4 | 6 | 8 | 10 |
|---|---|---|---|---|---|---|
| measured S | 0.2033 | 0.2145 | 0.2256 | 0.2367 | 0.2477 | 0.2588 |
| ideal S (no dead time, no drops) | 0.1500 | 0.1600 | 0.1700 | 0.1800 | 0.1900 | 0.2000 |
| duty range | 0.398..0.602 | 0.393..0.607 | 0.387..0.613 | 0.382..0.618 | 0.376..0.624 | 0.371..0.629 |
| max control-current error [A] | 0.075 | 0.002 | 0.003 | 0.005 | 0.007 | 0.024 |

- The slope of S versus `i_ref` is **0.00555 /A** (linear fit) against the theoretical `R/U_dc = 0.0050 /A` -
  a 10.9 % excess that the dead-time compensation explains.
- The constant part of the fit exceeds the ideal 0.15 by **0.0534 pu = 10.7 V**, i.e. the `sigma0` overhead.
- At `i_ref = 0` the control currents sit 0.05..0.08 A off zero: legs 4, 5 and 6 carry exactly zero current
  there, so the dead-time error has no defined sign. This is the dead-zone effect described in section 7.

## 5. Measured boundary versus bus voltage

### 5.1 Symmetric case, `i_ref = +10 A` (= `2*i_bias`, the physical limit)

| U_dc [V] | 200 | 120 | 100 | 80 | 60 | 50 | 47 | 46 | 45 | 42 | 40 | 35 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| measured S | 0.259 | 0.401 | 0.472 | 0.576 | 0.747 | 0.880 | 0.931 | 0.949 | 0.968 | 3.19 | 3.87 | 3.92 |
| duty range | .371-.629 | .299-.701 | .212-.788 | .127-.873 | .060-.940 | .034-.966 | .025-.975 | .016-.984 | outside | outside | outside |
| max error [A] | 0.100 | 0.075 | 0.056 | 0.042 | 0.032 | 0.071 | 0.129 | 0.312 | saturated | saturated | saturated |

- Full range is reachable down to **50 V** with S = 0.88 and errors below 0.03 A.
- 45 V is marginal (S = 0.968, error 0.31 A).
- At 42 V and below, S jumps above 3: once the duty leaves [0,1] the integrators wind up, the commands grow
  further and the modulator saturates hard (duties reach [-1.4, 2.4], i.e. the comparator is pinned).
- The analytic boundary is 43.6 V; the measured one is about 44.5 V.
- @scripts/run_case_study.m@ (mode @boundary@) reproduces the 200 / 100 / 60 / 50 / 45 / 40 V rows and
  asserts the 50 V, 45 V and 40 V verdicts; the other rows come from the finer exploratory sweeps of the
  same study.

### 5.2 Worst-case-like distribution `delta = [9.5 -9.5 5 9.5 9.5]` (ideal S = 49/U_dc)

| U_dc [V] | 200 | 100 | 70 | 60 | 57 | 55 | 53 | 50 |
|---|---|---|---|---|---|---|---|---|
| measured S | 0.303 | 0.559 | 0.773 | 0.890 | 0.932 | 0.963 | 1.79 | 3.78 |

The regression script reproduces the 200 / 70 / 60 / 55 / 50 V rows.
| max error [A] | 0.011 | 0.013 | 0.011 | 0.059 | 0.178 | 0.467 | saturated | saturated |

Measured boundary between 55 V and 53 V (about 54 V) against the analytic 53.0 V.

### 5.3 Zero control current (bias circulation only)

S = 0.2033 at 200 V and 0.9458 at 35 V; 32 V already saturates. The analytic boundary is 33.2 V. This is the
"no control current at all" case - a useful sanity limit, because it shows that the bus is committed by the
bias alone.

## 6. Achievable control current versus bus voltage

Sweeping `i_ref` at a reduced bus voltage exposes the trade-off directly:

| U_dc | 0 A | 2 A | 4 A | 6 A | 8 A | 10 A |
|---|---|---|---|---|---|---|
| 40 V | - | ok (S=0.886) | ok (S=0.934) | saturated | saturated | saturated |
| 35 V | ok (S=0.946, error 0.07 A) | saturated | saturated | saturated | saturated | saturated |
| 32 V | saturated even with no control current | - | - | - | - | - |

With `U_dc,min = [6*R*i_bias + k*(R*delta_max + V_ce)]/(1 - 2*Td/Ts)` the achievable control current is

```

delta_max(U_dc) = [ U_dc*(1 - 2*Td/Ts) - 6*R*i_bias - V_ce ] / (k*R)  ~=  0.95*U_dc - 31.8   [A]   (k = 1)
```

which gives 6.2 A at 40 V and 1.4 A at 35 V, against 4..5 A and about 0 A measured.

## 7. Numerical sensitive region: a leg at exactly zero current

Ideal switches plus dead time are ill-conditioned when a leg current is exactly zero: which device conducts
is undetermined, the zero-crossing detector retriggers and Simulink stops with "1000 consecutive zero
crossings". The theoretical worst distribution

```

delta = [+10, -10, +10, +10, +10]      legs 2, 3, 4, 7, 9 all carry exactly 0 A
```

does exactly that. **This is a solver limitation, not a topology limitation** - the same wiring with the
control current reduced by 5 % runs normally:

| Distribution | `S_ideal` | `S_measured` | min abs leg current | max control error | result |
|---|---|---|---|---|---|
| `[10 -10 10 10 10]` | 0.2500 | - | 0.000 A | - | solver aborts |
| `[9.5 -9.5 5 9.5 9.5]` | 0.2450 | 0.3028 | 0.248 A | 0.011 A | ok |
| `[9.0 -9.0 5 9.0 9.0]` | 0.2400 | 0.2977 | 0.498 A | 0.011 A | ok |
| `[9.5 -9.5 9.5 9.5 9.5]` | 0.2450 | 0.3029 | 0.198 A | 0.116 A | ok, degraded |
| `[8.0 -8.0 10 8.0 8.0]` | 0.2300 | 0.2872 | 0.994 A | 0.016 A | ok |

Two observations worth keeping:

- The third entry `delta3` is **free** in the worst-case distribution (it cancels out of `ub1 - ub8`).
  Choosing it as +5 A instead of +/-9.5 A keeps legs 4 and 5 away from zero and restores the accuracy, while
  the spread is unchanged - a practical mitigation.
- The literal scaled distribution `[9.5 -9.5 9.5 9.5 9.5]` still leaves leg 4 at `(d3+d2)/2 = 0`, and the
  control-current error is 12 % - the same dead-zone mechanism at work.

## 8. Design rule and margin

```

U_dc >= [ 6*R*i_bias + k*(R*delta_max + L*gamma_max) + V_ce ] / (1 - 2*Td/Ts)

worst case (k = 2, delta_max = 2*i_bias, gamma = 0):
    U_dc,min = [ 10*R*i_bias + V_ce ] / (1 - 2*Td/Ts)
symmetric  (k = 1):
    U_dc,min = [ 8*R*i_bias + V_ce ] / (1 - 2*Td/Ts)
```

| R [ohm] \ i_bias [A] | 2 | 3 | 5 | 8 | 10 |
|---|---|---|---|---|---|
| 0.2 | 6.1 | 8.2 | 12.4 | 18.6 | 22.8 |
| 0.5 | 12.4 | 17.6 | 28.0 | 43.6 | 54.1 |
| 1.0 | 22.8 | 33.2 | 54.1 | 85.3 | 106.1 |
| 2.0 | 43.6 | 64.5 | 106.1 | 168.6 | 210.3 |
| 3.0 | 64.5 | 95.7 | 158.2 | 252.0 | 314.5 |

For the verified model (R = 1 ohm, i_bias = 5 A) the bus requirement is 54 V worst case and 43.6 V
symmetric, so the 200 V bus has about 3.8x / 4.6x margin and **the binding limit on the control current is
the physical `i_a >= 0` constraint, not the bus**. The rule scales with `R*i_bias`: a 2 ohm winding would
need 106 V, a 3 ohm winding 158 V. Above that, the only remedies are a higher bus or a lower bias - and the
latter costs load capacity, which is the price of circulating the bias inside the machine instead of through
the bus.
