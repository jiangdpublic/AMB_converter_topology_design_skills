# Reference 3: differential ripple and the structural criterion

> The primary topology is the **single-star-point ten-leg topology**
> (internal id `w9x1`). This file explains why its five differential ripples
> are at the `R*delta*Ts/(2L)` limit, why reference B10-OPT-B is not, and what
> the reference `d569 = -1` variant costs.

---

## 1. The differential voltage of one DOF

For DOF j, let the a winding be `(ta, ha)` and the c winding `(tc, hc)`
(tail, head). Then

```
L * d(i_a - i_c)/dt = (V_ta - V_ha) - (V_tc - V_hc)
```

Every leg potential that appears with the **same sign in both windings**
cancels. The ripple is therefore set by the number of legs left in the
differential expression and by their potential spread.

The clean case is a pair that meets a **common node with the same orientation**:
both windings share a tail node, or both share a head node. The common potential
then cancels exactly and only the two winding drops remain:

```
differential ripple p-p = R*delta*Ts/(2L)
                        = 1 * 1 * 50e-6 / (2*500e-6)
                        = 0.05 A
```

This number is independent of the topology and of the bus; it depends only on
R, delta, the switching period and L.

### 1.1 Structural criterion

> A DOF pair is **clean** when its two windings meet a common node with the
> **same orientation**. The common node may be a driven leg **or the undriven
> star X**. If the pair meets no common node, or meets one with opposite
> orientations, the differential expression spans three or more legs and the
> ripple is roughly an order of magnitude larger.

The criterion is about **orientation**, not about "sharing a leg": two windings
that meet a leg with one tail and one head have the leg potential with
coefficient `-2` in the differential expression; that is the worst case.

---

## 2. The primary topology: all five pairs clean

| DOF | Pair | Common node | Orientation | Differential expression |
|---|---|---|---|---|
| DOF1 | w1, w2 | L1 | both heads | `V_L7 - V_L5` |
| DOF2 | w3, w4 | X | both heads | `V_L10 - V_L1` |
| DOF3 | w5, w6 | L6 | both tails | `V_L4 - V_L2` |
| DOF4 | w7, w8 | X | both tails | `V_L9 - V_L8` |
| DOF5 | w9, w10 | L1 | both tails | `V_L3 - V_L6` |

Every expression contains only two legs and no common leg, so the design
potential difference is exactly the winding drop. The predicted ripple is
0.05 A on all five DOFs. The Simscape run at `i_ctrl = 1 A/DOF`, `U_dc = 200 V`,
gives **0.054 0.051 0.054 0.053 0.050 A p-p**.

### 2.1 Why L1 does not blow up the leg current

L1 is the common node of DOF1 (two heads) and DOF5 (two tails). Each pair's
common-mode current is `i_a + i_c = 2*i_bias = 10 A`. At L1 one pair flows in
and the other flows out; the two 10 A common-mode flows cancel **exactly**, so

```
i_L1 = i_w4 = i_bias - delta2/2
```

The bias current is 5 A and the envelope is 7.5 A, despite L1 touching four
windings plus the trunk. This is the key difference from the reference
`d569 = -1` variant, where the 10 A common-mode current is not cancelled and
L3 must carry it.

### 2.2 Reading fig8 correctly: leg current vs winding current

`fig8_leg_currents.png` plots the **leg currents** L1..L10, reconstructed from
the winding currents by KCL. It is **not** a plot of w1..w10. For example:

```
I_L6 = i_w5 + i_w6 - i_w9
I_L1 = i_w4 + i_w9 + i_w10 - i_w1 - i_w2
```

A leg current is the algebraic sum of several winding currents, so it is not a
single sawtooth even when every winding current is. The narrow notches visible
on L1 (~0.73 A) and L6 (~0.49 A) are the summed fast edges of the winding
currents: each winding sees a ~1.4 us differential PWM pulse, and the edges
add in the leg sum. The feature is not a solver step artifact: reducing
`MaxStep` from 1e-6 to 1e-7 and `RelTol` from 1e-4 to 1e-5 changes the L6 notch
from 0.4947 A to 0.4951 A (0.08 %), and the winding currents themselves remain
clean sawtooths. `fig8b_leg_currents_zoom.png` zooms into L6 and shows the
finite width and the decomposition `w5 + w6 - w9`.

If the goal is a winding-current figure, use `fig4_winding_currents.png`
(`iw_log` has one column per winding, w1..w10).

---

## 3. Reference B10-OPT-B: one pair is not clean

In B, w9 is `L3 -> L6` and w10 is `L1 -> L3`. The pair meets L3 with **opposite**
orientations (w9 has its tail there, w10 its head), so

```
L * d(i_w9 - i_w10)/dt = (V_L3 - V_L6) - (V_L1 - V_L3)
                       = 2*V_L3 - V_L6 - V_L1
```

Three legs remain, with a coefficient of -2 on L3. The potential spread is
about 10 V, i.e. 0.05*U_dc, so the ripple is about 0.5 A instead of 0.05 A.
The Simscape measurement is **0.499 A p-p** on DOF5.

This is not a wiring error. It is the price of the 2.0252 phi bus: the same
L1-L3 cancellation that lowers the bus creates the opposite-orientation pair
on L3. The primary topology trades 16 % more bus for removing this ripple.

---

## 4. Reference d569 = -1: clean, but 10 A legs

Reversing the current directions of w5, w6 and w9 makes every pair meet a
common node with the same orientation:

| DOF | Pair | Common node | Orientation |
|---|---|---|---|
| DOF1 | w1, w2 | L1 | heads |
| DOF2 | w3, w4 | X | heads |
| DOF3 | w5, w6 | L6 | heads |
| DOF4 | w7, w8 | X | tails |
| DOF5 | w9, w10 | L3 | heads |

All five ripples drop to 0.05 A (measured 0.053 0.051 0.054 0.054 0.051 A), but
L3 now carries `i_a + i_c = 10 A`, the envelope rises to 10 A, and the bus
rises to 3.0252 phi. The primary topology obtains the same clean ripple with
5 A / 7.5 A legs and 2.3501 phi, so it strictly dominates this reference.

---

## 5. Variant comparison

| Variant | Command | `U_dc,min/phi` | Bias / envelope | DOF1..5 ripple p-p |
|---|---|---|---|---|
| **Primary (single-star-point, w9x1)** | `b10optb_build_all` | **2.3501** | **5 / 7.5 A** | **0.054 0.051 0.054 0.053 0.050** |
| Reference B10-OPT-B | `b10optb_build_all('B')` | 2.0252 | 5 / 7.5 A | 0.055 0.053 0.055 0.052 **0.499** |
| Reference d569 = -1 | `b10optb_build_all('d569')` | 3.0252 | 10 / 10 A | 0.053 0.051 0.054 0.054 0.051 |
| Historical w4 -> L3 (B directions) | `b10optb_build_all('w4x3')` | 2.7002 | 5 / 12.5 A | DOF5 0.503 (no benefit) |
| Historical w4 -> L3 + d569 = -1 | `b10optb_build_all('w4x3-ripple')` | 2.7002 | 5 / 7.5 A | 0.054 0.050 0.055 0.055 0.052 |
| Historical user proposal | `b10optb_build_all('user')` | 2.6751 | 5 / 7.5 A | 0.055 0.052 0.050 0.052 0.502 |

The historical variants are kept for the record. None is better than the
primary in the 5 A class; the user proposal is dominated by reference B.

### 5.1 How the primary was found

Starting from B, move the **driven end** of one winding to another leg and keep
the other end fixed. For w9 the fixed end is L6 and the moved end is L3 -> L1.
The requirement is that every DOF pair ends up meeting a common node with the
same orientation; the search is automated in `b10_reterm_sweep2.m`.

- Moving w4's L1 end to L3 (the historical `w4x3`) is effective only together
  with `d569 = -1`, and costs 2.7002 phi.
- Moving w9's L3 end to L1 costs only 2.3501 phi and needs **no direction
  flips**; it is the primary.
- Do **not** re-terminate to a leg that is already connected to the target
  node: e.g. w4 -> L8/L9/L10, or leaving w9 on L3 and moving w10 to L6, creates
  a **parallel edge**, hence a 2-edge cycle (`cyc = 1`) and a static bias
  error under R/L mismatch. The sweep filters these out.

### 5.2 Do not interleave the carriers

For reference B, the DOF5 differential expression contains L1, L3 and L6.
With one synchronous carrier their "off" intervals are nested, so the -2
coefficient on L3 is never exposed. Shifting the carriers breaks the nesting:

| Carrier phase [L1 L3 L6] | DOF5 ripple p-p |
|---|---|
| 0, 0, 0 (synchronous) | **0.503 A** |
| 0, 0, 0.5 | 9.998 A |
| 0, 0.5, 0 | 19.493 A |
| 0, 1/3, 2/3 | 13.329 A |

All builds in this skill use one common carrier and centred duties. Keep it
that way for the reference B comparison; for the primary, the clean pairs do
not rely on the nesting, but there is no reason to interleave.

---

## 6. Recompute

```matlab
addpath('<skill>/scripts')
b10_variant_check              % criteria + predicted ripple + carrier study
b10optb_build_all; b10optb_simcheck
b10optb_build_all('B'); b10optb_simcheck
b10optb_build_all('d569'); b10optb_simcheck
b10optb_build_all('w9x1')      % restore the primary model
```
