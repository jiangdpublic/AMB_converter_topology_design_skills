# Reference 2: class model, criteria, and optimality

> This file answers two questions: **what the 10-leg cycle-free class looks
> like**, and **how the criteria are computed**. The primary topology is the
> single-star-point ten-leg topology (internal id `w9x1`). Reference variants
> `B` and `d569` are included for comparison.
>
> All numbers can be recomputed with `../scripts/b10_check.m` and
> `../scripts/b10_variant_check.m`.

---

## 1. Parameters (identical to the sibling AMB topology studies)

| Quantity | Value | Note |
|---|---|---|
| `R` / `L` | 1 ohm / 500 uH | per winding |
| `i_bias` | 5 A | fixed bias per winding |
| `delta_max` | 5 A | control-current amplitude per DOF |
| `f_c` | 1500 Hz | current-loop bandwidth (= `f_sw/13`) |
| `f_sw` | 20 kHz | carrier |
| `sigma` | `2*pi*f_c*delta_max` = 47123.9 A/s | current-slew proxy |
| **`phi`** | `R*delta_max + L*sigma` = **28.5619 V** | normalisation of the bus criterion |
| `u_max` | `R*i_bias + phi/2` = 19.2810 V | maximum voltage of one winding |

**Criterion vector (smaller is better):**
`(legs, cyc, U_dc,min/phi, Idc_bias, Idc_env)`.

- `cyc` = number of independent cycles. It must be 0: with a cycle, winding
  R/L mismatch creates a static bias error (measured 0.394 A at +/-5 % R).
- `U_dc,min` = the smallest bus that lets every leg potential produce the
  required winding voltages over the full control envelope. It is computed by
  `tp_floor_leg.m` (linear program: `min U s.t. A*phi = u, 0 <= phi_leg <= U`,
  maximised over the 32 envelope corners) and cross-checked by
  `b9_floor_fast.m` (spanning-tree recursion). The two must agree.
- `Idc_bias` = maximum leg current at the bias point (`delta = 0`);
  `Idc_env` = maximum over the 32 envelope corners.

---

## 2. Class model: what a 10-leg cycle-free topology can look like

The winding graph must be a **forest** (`cyc = 0`) with
`edges = nodes - components`. Ten winding edges and eleven nodes give

```
10 = 11 - c   =>   c = 1   =>   connected
10 + k - 1 = 10   =>   k = 1   =>   exactly one undriven node
```

So every connected 10-leg tree has **exactly one undriven node**. It must obey
three hard rules:

1. its degree is `0 (mod 4)` (`#in = #out`, so the bias and the deltas cancel);
2. each side is a **complete DOF pair** (otherwise the control currents couple
   linearly);
3. degree 4, with the other nodes driven legs.

The shape is therefore a **star + a path + subtrees**: four edges at the star
(two complete pairs), then a tree of driven legs.

The primary topology satisfies this with X of degree 4 and one additional
branching leg, L1, of degree 5. L1 is **not** an undriven star: it is a driven
leg that joins DOF1's pair, DOF5's pair, and the trunk `w4 = L1 -> X`. Its two
pair common-mode currents cancel exactly (10 A in, 10 A out), leaving the
`w4` current, so the leg current stays in the 5 A class.

---

## 3. Candidates in the 10-leg class

| Topology | Legs | `U_dc,min/phi` | Bias / envelope | Split? | Verdict |
|---|---|---|---|---|---|
| **Single-star-point ten-leg (w9x1)** | **10** | **2.3501** | **5 / 7.5 A** | no | **primary: all-clean 5 A class** |
| B10-OPT-B (reference) | 10 | 2.0252 | 5 / 7.5 A | no | 5 A-class bus optimum; DOF5 0.5 A |
| B10-OPT-A | 10 | 2.0000 | 10 / 15 A | no | 10 A-class optimum |
| B10-OPT-C | 10 | 1.6751 | 15 / 22.5 A | no | 15 A-class optimum |
| B9P10 + driven star | 10 | 3.0503 | 5 / 7.5 A | **yes** | strictly dominated by B10-OPT-B |
| B9-OPT-A + driven star | 10 | 2.3501 | 10 / 15 A | **yes** | strictly dominated by B10-OPT-A |

Two warnings:

- The 2.3501 entries are **different topologies**. The primary is a native
  10-leg netlist with one undriven star and 5 A legs; the last row is a 9-leg
  shape with its star promoted to a driven leg and 10 A legs. Do not merge them.
- The primary is **not** the minimum-bus point of the class. It is 16 % higher
  than reference B (2.3501 vs 2.0252 phi). The primary is selected because its
  ripple is clean on **all five** DOFs at the same 5 A / 7.5 A leg current.

The "split" references do not improve the criteria: driving a star of the
9-leg shape leaves the star potential inside the span of the leg potentials,
so the bus and the currents are unchanged; one extra half-bridge buys nothing.
B10-OPT-B is different: its degree-4 L1 is not a passive star because its
outgoing pair is `(w4 = c2, w10 = c5)`, which is not a DOF pair. Hard-wiring
L1 as a star leaves a 5 KCL residual; `b10_undrive.m` reproduces this.

### 3.1 Reference d569 = -1

The same wiring as B with the current directions of w5, w6 and w9 reversed:

| Quantity | Value |
|---|---|
| `U_dc,min` | 86.4054 V = 3.0252 phi |
| Bias / envelope | 10.00 A / 10.00 A |
| DOF1..5 ripple | 0.053 0.051 0.054 0.054 0.051 A |
| Why it is not primary | same clean ripple as the primary, but 10 A legs and a 29 % higher bus; strictly dominated |

---

## 4. How the bus floor is computed

For each of the 32 envelope corners (`delta_j = +/-delta_max` for the five
DOFs), the winding voltages are

```
u_k = R*i_bias + s_j * sigma_k * (phi/2)
```

with `sigma_k = +1` for an odd (a) winding and `-1` for an even (c) winding.
Solving `A*phi = u` along the tree gives every leg potential; the bus needed
at that corner is `max(phi_leg) - min(phi_leg)`. `U_dc,min` is the maximum
over the 32 corners. The LP and the spanning-tree recursion are independent
implementations of the same quantity.

The hand calculation of the same number is the subject of
`06-bus-voltage-how-to.md`. The primary topology's binding path is
`L10 -> L4` (or the tied `L2 -> L10`): four aligned `phi/2` terms plus two
bias terms, `4*14.2810 + 2*5.0 = 67.1239 V = 2.3501 phi`.

---

## 5. Recompute

```matlab
addpath('<skill>/scripts')
b10_check          % class table, split references, dominance, LP cross-check
b10_variant_check  % primary + references: criteria, ripple prediction, carrier study
b10_undrive        % reference B: hard-wiring L1 as a star leaves a 5 A KCL residual
```

`tp_floor_leg.m` (LP) and `b9_floor_fast.m` (recursion) are independent; any
conclusion must agree in both. The star-node KCL residual must be exactly 0
before a topology is admitted.
