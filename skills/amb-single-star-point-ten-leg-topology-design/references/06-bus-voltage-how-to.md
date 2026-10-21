# Reference 6: estimating the dc-bus requirement from the topology

> Scope: fixed-bias (bias + differential) AMB drive topologies.
> Parameters used throughout: `R = 1 ohm`, `L = 500 uH`, `i_bias = 5 A`,
> `delta_max = 5 A`, `f_c = 1500 Hz`.
> The companion script `b10_bus_intuition.m` prints every intermediate quantity
> in this document and needs only MATLAB (no Simscape).
> The primary topology is the single-star-point ten-leg topology (internal id
> `w9x1`); reference B10-OPT-B is used for the cancellation examples.

---

## 0. One-line result

The bus does not have to cover "the voltage of one winding"; it has to cover
**the largest potential difference between any two legs over the full control
envelope**. Each leg potential is the signed sum of the winding voltages along
the tree path from the star.

The requirement therefore always has the form

```
U_dc,min = n_phi * (phi/2) + n_b * (R * i_bias)          (n_phi, n_b integers, signed)
```

With `phi/2 = 14.2810 V` and `R*i_bias = 5.0 V`, every topology's bus is an
integer combination of these two numbers. Counting terms and deciding their
signs is enough to rank topologies; no simulation is needed.

---

## 1. Inputs: how phi is obtained, and why only half of it

### 1.1 Explicit phi

The current loop must move the control current to `delta_max` at bandwidth
`f_c`, so the required current slew is

```
sigma = 2*pi*f_c*delta_max = 2*pi*1500*5 = 47123.9 A/s
```

A winding must provide the resistive and inductive drops at the same time (same
sign at the worst instant), so

```
phi = R*delta_max + L*sigma
```

Numerically:

```
R*delta_max = 1 * 5              =  5.0000 V
L*sigma     = 500e-6 * 47123.9   = 23.5619 V
phi         = 5.0000 + 23.5619   = 28.5619 V
```

A useful mental form is `phi = delta_max*(R + 2*pi*f_c*L) = 5*(1 + 4.7124) = 28.5619 V`.

> Physical meaning: **phi is the total winding-voltage swing needed to drive one
> DOF's control current from -delta_max to +delta_max as fast as the bandwidth
> allows.**

### 1.2 Why a winding sees only phi/2

The control current of a DOF is the difference of its two winding currents:
`delta = i_a - i_c`. At the bias point both are `i_bias`; a control command
splits equally:

```
i_a = i_bias + delta/2 ,      i_c = i_bias - delta/2
```

When delta swings from `-delta_max` to `+delta_max`, one winding's current
swings by only `delta_max/2`, and its voltage offset is only half of phi:

```
R*(delta_max/2) + L*(sigma/2) = 2.5000 + 11.7810 = 14.2810 V = phi/2
```

So every winding voltage has the form **bias + half-swing**:

```
u_a = R*i_bias + s * (phi/2)          % a windings: w1,w3,w5,w7,w9
u_c = R*i_bias - s * (phi/2)          % c windings: w2,w4,w6,w8,w10
```

`s = +-1` is the DOF corner sign defined below. Numerically `u` lies in
`[R*i_bias - phi/2, R*i_bias + phi/2] = [-9.2810, +19.2810] V`; the upper end
is the `u_max` used in the criteria.

---

## 2. Exact procedure (three lines)

1. **Pick a corner**: each of the five DOFs takes `+-delta_max`, so there are
   `2^5 = 32` corners. At a corner,
   ```
   u_k = R*i_bias + s_dof(k) * sigma_k * (phi/2)
   ```
2. **Solve the potentials**: root the tree at the star (`phi_X = 0`) and
   back-substitute `phi_head = phi_tail - u_k` to get the ten leg potentials.
3. **Take the span**: the bus needed at that corner is
   `max(phi_leg) - min(phi_leg)` (the window can be shifted, so only the span
   matters); `U_dc,min` is the maximum over the 32 corners.

### 2.1 s_dof and sigma_k are different things

| Symbol | Decided by | Value | Meaning |
|---|---|---|---|
| `s_dof(k)` | **the corner you are enumerating** | `+-1` | `s_j = +1` means DOF j is at `delta_j = +delta_max` (its a winding has the larger current); `-1` means the opposite |
| `sigma_k` | **the parity of the winding number** | `+1 / -1` | w1,w3,w5,w7,w9 (odd) are a windings -> `+1`; w2,w4,w6,w8,w10 (even) are c windings -> `-1` |

The **tail/head direction is not in these two signs**. It is already contained
in the `+-1` coefficient of the term in the path expression. The net sign of
a term is the product of three factors:

```
net sign = (path coefficient +-1) * sigma_k * s_dof(k)
```

Example: a path contains `+u4`; w4 is even, so `sigma = -1`; with `s_2 = -1`
the net sign is `(+1)(-1)(-1) = +1`, contributing `+(phi/2)` and `+5 V`.

### 2.2 How to choose s_dof (one sign per DOF)

**`s_dof` is one sign per DOF, not one per winding.** It represents the
direction of that DOF's control current. Since both windings of the DOF are
driven by the same `delta_j`, their `s_j` must be the same; the a/c
difference is carried entirely by `sigma_k`. This is also why the a and c
terms of one DOF can cancel.

For a given two-leg path, write the difference as "fixed part + corner part":

```
DeltaV = n_b*(R*i_bias) + (phi/2) * sum_j ( g_j * s_j )

    n_b = sum_k d_k                       (path only; corner-independent)
    g_j = sum_{k in DOF j} d_k * sigma_k  (path only; corner-independent)
```

- `g_j = 0`: the DOF's `s_j` is **arbitrary** (its a and c phi/2 terms
  already cancel).
- `g_j != 0`: the DOF contributes only `+-|g_j|`.
- Flipping all `s_j` flips the phi/2 term but leaves `n_b*(R*i_bias)`
  unchanged. A leg pair therefore has only two extremes:

```
DeltaV+- = n_b*(R*i_bias) +- ( sum_j |g_j| ) * (phi/2)
```

**Three-step choice (no trial and error):**

```
1. n_b = sum_k d_k
2. g_j = sum_{k in DOF j} d_k * sigma_k          % skip DOFs with g_j = 0
3. m = sign(n_b)   (arbitrary if n_b = 0)
   s_j = m * sign(g_j)
   => |DeltaV|max = ( sum_j |g_j| ) * (phi/2) + |n_b| * (R*i_bias)
```

**Why `m = sign(n_b)`**: the maximum magnitude is obtained when the bias term
and the phi/2 term have the same sign. A positive bias term needs a positive
phi/2 combination; a negative bias term needs a negative one. This is exactly
why the two examples below use opposite `s_dof`.

| | Example A: primary `L10 <-> L4` | Example B: reference B `L5 <-> L8` |
|---|---|---|
| `n_b` | **+2** | **+3** |
| `m = sign(n_b)` | **+1** | **+1** |
| path `g_j` | g_2 = +2, g_3 = +1, g_5 = +1 | g_1 = g_2 = g_4 = -1 |
| `s_j = m*sign(g_j)` | s_2 = s_3 = s_5 = +1 (s_1, s_4 arbitrary) | s_1 = s_2 = s_4 = -1 (others arbitrary) |
| result | `4*(phi/2) + 2*(R i_bias) = 67.1239 V` | `3*(phi/2) + 3*(R i_bias) = 57.8429 V` |

### 2.3 Feasibility check

`u` must lie in the column space of `A` (equivalently, the star-node KCL must
close). If a corner is infeasible, the script prints `corner NN infeasible`:
the wiring is physically invalid, usually because one side of the star is not
a complete DOF pair. If both sides are complete pairs, all 32 corners are
feasible.

---

## 3. Hand calculation (three steps on paper)

### Step 1: write every leg potential along the tree

Set the star to 0 and use `phi_head = phi_tail - u_k`. For the primary
topology (`b10_bus_intuition('w9x1')` prints exactly this):

```
phi_L1  = +u4                      phi_L6  = +u4 -u9
phi_L2  = +u4 -u6 -u9              phi_L7  = +u4 +u1
phi_L3  = +u4 -u10                 phi_L8  = -u8
phi_L4  = +u4 -u5 -u9              phi_L9  = -u7
phi_L5  = +u4 +u2                  phi_L10 = +u3
```

### Step 2: evaluate all 45 leg pairs (do not guess "the longest path")

Subtract any two legs; the result is the signed sum along their two-leg path.
The tool sorts all 45 pairs and prints the worst first. The top of the primary
list is:

| Leg pair | Terms | `n_phi` | `n_b` | Worst `DeltaV` | `/phi` |
|---|---|---|---|---|---|
| **L10 <-> L4** | **4** | **-4** | **-2** | **-67.1239 V** | **2.3501** |
| L2 <-> L10 | 4 | -4 | -2 | -67.1239 V | 2.3501 |
| L5 <-> L8 | 3 | +3 | +3 | +57.8429 V | 2.0252 |
| L4 <-> L5 | 3 | -3 | -3 | -57.8429 V | 2.0252 |
| L7 <-> L8 | 3 | +3 | +3 | +57.8429 V | 2.0252 |
| L2 <-> L5 | 3 | -3 | -3 | -57.8429 V | 2.0252 |
| L4 <-> L9 | 4 | -4 | 0 | -57.1239 V | 2.0000 |
| L4 <-> L8 | 4 | +4 | 0 | +57.1239 V | 2.0000 |

The signs of `n_phi` and `n_b` depend on which leg is subtracted from which;
only the magnitude matters. "Terms" is the raw count before cancellation;
`n_phi` is the signed sum of the remaining phi/2 coefficients.

**The important lesson: more terms is not necessarily worse.** In reference B,
`L5 <-> L8` has only 3 terms but ties the 5-term `L4 <-> L10` at 2.0252 phi,
because all three terms align at the same corner. Conversely, B's
`L4 <-> L8` has 5 terms but only 1.6751 phi: `-u9 - u10` come from the a and c
windings of the **same DOF5**, with the same path direction, so their phi/2
terms cancel (only `2*R*i_bias` remains). The 4-term `L4 <-> L5` (1.7002) is
therefore worse than the 5-term `L4 <-> L8` (1.6751). **Count terms, then
count the bias terms separately.**

### Step 3: sign every term of the worst pair

**Example A (primary, 4 terms)**: binding pair `L10 <-> L4`.

```
phi_L10 - phi_L4 = u3 - (u4 - u5 - u9) = -u4 + u5 + u9 + u3          (4 terms)
```

The four coefficients are `d = [-1, +1, +1, +1]`, so `n_b = +2` (positive)
and `m = +1`. The DOF sums are `g_2 = +2` (u4 and u3), `g_3 = +1` (u5),
`g_5 = +1` (u9), so `s_2 = s_3 = s_5 = +1`; `s_1` and `s_4` do not appear
and are arbitrary. The tool reports the corner `[-1 +1 +1 -1 +1]` (the two
arbitrary signs happen to be -1).

| Term | Path coefficient | `sigma_k` | `s_dof` | Net sign | Contribution |
|---|---|---|---|---|---|
| u4 (c2) | -1 | -1 | s_2 = +1 | +1 | +(phi/2) **- 5 V** |
| u5 (a3) | +1 | +1 | s_3 = +1 | +1 | +(phi/2) + 5 V |
| u9 (a5) | +1 | +1 | s_5 = +1 | +1 | +(phi/2) + 5 V |
| u3 (a2) | +1 | +1 | s_2 = +1 | +1 | +(phi/2) + 5 V |

The **net sign** column fixes the phi/2 contribution. The R*i_bias part carries
the **path coefficient** `d` itself: u4 has `d = -1`, so it contributes -5 V,
while the other three contribute +5 V each; the sum is 4*(phi/2) + 10 V.

```
U_dc,min >= 4*14.2810 + 2*5.0 = 57.124 + 10.0 = 67.1239 V = 2.3501*phi
```

The LP cross-check gives the same 2.3501 phi.

**Example B (reference B, 3 terms)**: binding pair `L5 <-> L8`.

```
phi_L5 - phi_L8 = (u4 + u2) - (-u8) = +u4 + u2 + u8          (3 terms)
```

All three are c windings with coefficient +1; `n_b = +3`, `m = +1`,
`g_1 = g_2 = g_4 = -1`, so `s_1 = s_2 = s_4 = -1`. Each term contributes
`+(phi/2) + 5 V`, hence

```
U_dc,min >= 3*14.2810 + 3*5.0 = 57.8429 V = 2.0252*phi
```

**Example C (reference B, 5 terms with cancellation)**: `L4 <-> L8`.

```
phi_L4 - phi_L8 = (u4 -u5 -u9 -u10) - (-u8) = +u4 -u5 -u9 -u10 +u8
```

`n_b = -1` (negative), so `m = -1` and `s_j = +1`. The a/c pair
`u9, u10` of DOF5 has `g_5 = 0`, so its phi/2 terms cancel and `s_5` is
arbitrary. The remaining phi/2 terms give `n_phi = -3`, and

```
|DeltaV| = | -3*14.2810 - 1*5.0 | = 47.8429 V = 1.6751*phi
```

The bias terms alone differ by 20 V (+15 V in B, -5 V in C), but the phi/2
terms are +3*14.2810 in B and -3*14.2810 in C, so they partly cancel that
difference: the two bus magnitudes differ by **10 V = 0.35 phi** (57.8429 vs
47.8429), not by 20 V. Always compare the total magnitude, not one column.

---

## 4. When do terms cancel? Two rules

> **Rule 1 (same polarity type, opposite traversal).** On a two-leg path, if
> the same polarity type (both a or both c) is traversed twice in opposite
> directions, the net signs are opposite, so the two terms cancel (both the
> phi/2 part and the R*i_bias part).
> Intuition: a is always +phi/2, c always -phi/2; traversing a winding backwards
> flips the whole term; flipping twice brings the sign back to opposite.
> Example: in B's `L4 <-> L10`, `-u4` and `+u10` (both c, opposite
> directions) cancel two terms, leaving 3 of 5.
>
> **Rule 2 (same DOF, a and c in the same direction).** If the a and c windings
> of one DOF appear with the **same path direction**, their phi/2 terms add to
> zero and only `2*R*i_bias` remains (the bias does not cancel; it adds).
> Example: `-u9 - u10` in B's `L4 <-> L8`.

Combined: **first cancel all phi/2 pairs you can, then count the remaining
phi/2 terms; count the R*i_bias terms separately, because they never cancel-
they only add.**

---

## 5. Five-case comparison (all computed by `b10_bus_intuition.m` and
cross-checked by the LP)

| Topology | Worst leg pair | Terms | Cancelled | `n_phi` | `n_b` | `U_dc,min` | `/phi` |
|---|---|---|---|---|---|---|---|
| **Primary (single-star-point, w9x1)** | **L10 <-> L4** | **4** | **0** | **4** | **2** | **67.1239 V** | **2.3501** |
| B10-OPT-B (reference) | L5 <-> L8 (also L4 <-> L10) | 3 / 5 | 0 / 2 | 3 | 3 | 57.8429 V | 2.0252 |
| w4 -> L3 (`w4x3`) | L7 <-> L9 | 4 | 0 | 4 | 4 | 77.1239 V | 2.7002 |
| w4 -> L3 + `d569 = -1` | L4 <-> L9 | 4 | 0 | 4 | 4 | 77.1239 V | 2.7002 |
| B10-OPT-B + `d569 = -1` | L2 <-> L9 | 5 | 0 | 5 | 3 | 86.4054 V | 3.0252 |

### Why B is the cheapest, and why its ripple is the worst

- B's worst path goes through L1: after `X -> L1` (w4, traversed backwards)
  it leaves L1 through w10 to L3. Both are c windings traversed in opposite
  directions, so two terms cancel and the bus is only 2.0252 phi.
- The same "w9 and w10 share L3 with opposite orientations" wiring puts
  `2*V_L3 - V_L6 - V_L1` into the DOF5 differential voltage, raising the
  ripple by an order of magnitude (0.5 A instead of 0.05 A).
- **One structural feature buys the bus and spoils the ripple.** Re-terminating
  w9 to L1 (the primary) removes the cancellation pair and raises the bus to
  2.3501 phi (+16 %), but makes all five pairs clean.

---

## 6. One-key tool

```matlab
addpath('<skill>/scripts')
b10_bus_intuition            % primary topology (default)
b10_bus_intuition('B')       % reference B10-OPT-B
b10_bus_intuition('d569')    % reference direction_569 = -1
b10_bus_intuition('w4x3')    % historical w4 -> L3
```

It prints the symbolic potential of every leg, the sorted 45-pair table with
terms / `n_phi` / `n_b`, the binding corner and legs, the term-by-term path
breakdown, `U_dc,min`, and the LP cross-check. Change one wire and re-run it
to see the bus cost immediately.

---

## 7. Relation to the criteria table

- The criterion vector is `(legs, cyc, U_dc,min/phi, Idc_bias, Idc_env)`;
  this document covers only the third entry.
- `Idc_bias` / `Idc_env` use the same `A` matrix: `I_leg = A' * i`,
  evaluated at the bias point and at the 32 corners, then maximised. They are
  pure topology + algebra, like the bus.
- Ripple is not in this algebra (it depends on the switching process); see
  `03-ripple-and-direction.md`.
