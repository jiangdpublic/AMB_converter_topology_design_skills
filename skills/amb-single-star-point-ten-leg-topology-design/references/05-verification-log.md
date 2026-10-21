# Reference 5: verification log and acceptance checklist

> Environment: MATLAB / Simulink / Simscape / Simscape Electrical
> **R2025b (25.2) Update 6**, Linux.
> The primary topology is the single-star-point ten-leg topology; **w9x1** is
> its internal id and denotes exactly the same netlist.
> All numbers were produced by `../scripts/b10optb_build_all.m` and
> `b10optb_simcheck.m` in this skill. Commands are at the end.

---

## 1. Model structure self-check (primary)

| Check | Result |
|---|---|
| Different-net collinear line overlaps | **0** (1468 segments / 375 nets) |
| Netlist read-back (10 edges + 11 distinct nodes) | **10/10 ok** (`topology check: 1`) |
| Star-node KCL residual (design) | **0.00e+00 A** |
| Controller unconnected-input guard | 0 (checked when `Ctrl` is built) |
| Scope / `iw_log` channel convention | **channel k = winding wk** (lines named w1..w10) |

## 2. Primary current-loop tracking

```
winding currents w1..w10 [A]: 5.502 4.502 5.503 4.503 5.504 4.503 5.504 4.503 5.503 4.503
reference        w1..w10 [A]: 5.500 4.500 5.500 4.500 5.500 4.500 5.500 4.500 5.500 4.500
max error = 0.0038 A (0.08 %)
ripple p-p [A]             : 0.282 0.231 0.265 0.244 0.283 0.232 0.265 0.236 0.267 0.222
differential i_a-i_c [A]   : 1.0002 1.0007 1.0016 1.0019 1.0009
differential ripple p-p    : 0.054 0.051 0.054 0.053 0.050
```

- Currents are time-weighted averages over the last 20 % of an 8 ms run
  (23,583 solver samples).
- `i_ctrl` is a parameter, not a hard-coded duty. The reference currents are
  `5.5 / 4.5 A` for `i_ctrl = 1 A/DOF`; changing `i_ctrl` changes all ten
  references and the closed loop follows them.
- Winding ripple is 0.222-0.283 A p-p (about 5 % of the 5 A bias), 20 kHz.
- Differential ripple is 0.050-0.054 A p-p on **all five** DOFs. This is the
  structural `R*delta*Ts/(2L)` limit, not a tuned result.

## 3. Closed loop with realistic device drops

`b10optb_devtest` swaps the near-ideal devices for IGBT
`Vf = 0.8 V, Ron = 1 mOhm` and diode `Vf = 0.6 V, Ron = 0.3 ohm`:

```
closed loop + realistic devices, currents w1..w10 [A]:
5.502 4.502 5.504 4.503 5.505 4.504 5.504 4.503 5.503 4.504
max error = 0.0048 A (0.10 %)
```

The closed loop compensates the device drops. The same devices in an open-loop
model would leave the currents 20-50 % low; this is why the reference model is
closed-loop.

## 4. Reference variants (same model, different netlist)

| Build | `U_dc,min/phi` | Leg current (bias) | DOF1..5 differential ripple p-p |
|---|---|---|---|
| Primary `b10optb_build_all('w9x1')` | **2.3501** | **5.51 A** | **0.054 0.051 0.054 0.053 0.050** |
| Reference `b10optb_build_all('B')` | 2.0252 | 5.51 A | 0.055 0.053 0.055 0.052 **0.499** |
| Reference `b10optb_build_all('d569')` | 3.0252 | **10.01 A** | 0.053 0.051 0.054 0.054 0.051 |

The "leg current" column is the maximum time-averaged leg current reported by
`b10optb_fig_legs`; the design values are 5.00 A and 10.00 A. The small
excess is the current-loop ripple.

The reference builds also pass the topology and overlap checks (1469 segments
for B, 1470 for d569; 0 overlaps). They are comparisons only; rebuild the
primary afterwards.

`fig8` plots leg currents L1..L10, not winding currents. Its supplementary zoom
`fig8b_leg_currents_zoom.png` shows that the L6 notch is the finite-width sum
`w5 + w6 - w9` of the differential PWM pulses, not a solver artifact; a 10x
smaller step leaves the notch amplitude unchanged.

## 5. Acceptance checklist

| # | Check | Target | Primary result |
|---|---|---|---|
| 1 | Ten winding currents vs reference | < 0.5 % | 0.08 % |
| 2 | Differential current per DOF | 1.0000 +- 0.005 A | 1.0002..1.0019 A |
| 3 | Scope / log channel k | winding wk | verified channel by channel |
| 4 | Netlist read-back | 10/10, 11 distinct nodes | 10/10 |
| 5 | Different-net line overlaps | 0 | 0 |
| 6 | Star-node KCL residual | 0 | 0.00e+00 A |
| 7 | Criteria vs reference table | 2.3501 / 5 / 7.5 | matches |
| 8 | `i_ctrl` change to 0.5 | 5.25 / 4.75 A | 5.252 / 4.753 A (0.08 %) |
| 9 | Realistic device drops | < 0.5 % | 0.10 % |
| 10 | Winding ripple | order 0.2-0.3 A p-p @ 20 kHz | 0.222-0.283 A |

## 6. Commands

```matlab
addpath('<skill>/scripts')      % only the scripts folder is needed

b10optb_build_all               % build primary + wire + overlap + netlist + 8 ms run
b10optb_simcheck                % simulate only; prints ch k = wk
b10optb_fig                     % winding-current plot
b10optb_fig_legs                % fig8 (primary + B + d569) and fig8b (L6 zoom)
b10_fig                         % primary topology figure
b10_fig('B'); b10_fig('d569')   % reference topology figures
b10optb_devtest                 % realistic device drops
b10optb_build_all('B')          % reference B
b10optb_build_all('d569')       % reference direction_569 = -1
b10_variant_check               % criteria + ripple prediction + carrier study
```

**Notes**

- The solver is fixed at VariableStepAuto, `MaxStep = 1e-6`,
  `RelTol = 1e-4`. This is not optional for a topology-level model: a larger
  step directly corrupts the switching state and the ripple.
- Every `b10optb_build_all` deletes and rebuilds
  `../model/SINGLE_STAR_TEN_LEG_AMB.slx`; hand edits are lost.
- To change parameters (`i_ctrl`, `i_bias`, `U_dc`, `L_w`, `f_sw`), edit
  `b10optb_params.m` and rebuild.
