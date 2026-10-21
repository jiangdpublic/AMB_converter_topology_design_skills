# Reference 1: Single-Star-Point Ten-Leg Topology — authoritative definition

> This file is the authoritative definition of the topology itself: netlist,
> structure, criteria, and the comparison with the reference variants.
> The canonical name is **single-star-point ten-leg topology**. The internal
> variant id **w9x1** is kept for continuity with the development record and
> denotes exactly the same netlist. All statements below apply to both names.
>
> Companion figure: `../assets/fig1_topology.png`.
> Reference figures: `../assets/fig9_reference_b10optb_topology.png` and
> `../assets/fig10_reference_d569_topology.png`.

---

## 1. Result at a glance

| Item | Primary topology | Reference B10-OPT-B | Reference d569 = -1 |
|---|---|---|---|
| Legs / windings | **10 / 10** | 10 / 10 | 10 / 10 |
| Undriven star points | **1 (X, degree 4)** | 1 (X, degree 4) | 1 (X, degree 4) |
| Independent cycles | **0 (tree)** | 0 (tree) | 0 (tree) |
| `U_dc,min` | **67.1239 V = 2.3501 phi** | 57.8429 V = 2.0252 phi | 86.4054 V = 3.0252 phi |
| Bias / envelope leg current | **5.00 A / 7.50 A** | 5.00 A / 7.50 A | 10.00 A / 10.00 A |
| DOF1..5 differential ripple (p-p) | **0.054 0.051 0.054 0.053 0.050 A** | 0.055 0.053 0.055 0.052 **0.499 A** | 0.053 0.051 0.054 0.054 0.051 A |
| Role | **primary** | reference: minimum bus in the 5 A class | reference: all-clean but 10 A legs |

The primary topology is the **all-clean 5 A-class** point: every DOF pair meets
a common node with the same orientation, so all five differential ripples are
at the `R*delta*Ts/(2L)` limit. Its bus is 16 % above reference B, which is the
minimum-bus point of the same current class. Reference `d569 = -1` has the same
clean ripple but needs 10 A legs and a 29 % higher bus than the primary. It is
strictly dominated by the primary.

---

## 2. Structure

```
      L8    L9    L10                 L5         L7
       \     |     /                   \         /
        \    |    /                     \       /
         +---+---+                       [L1]--+
             |                            |  \
           [X] (star, degree 4)           |   \ w10
             |                            |    \
             +----------- w4 -------------+     [L3] (leaf leg)
                                          |
                                         w9
                                          |
                                         [L6]
                                        /    \
                                      L2      L4
```

- **Nodes**: `X` = the only undriven node (the single star point); `L1..L10` =
  the ten driven half-bridge legs.
- **Edges**: ten windings, each carrying a positive current at the bias point
  in the tail -> head direction of the netlist below.
- **Graph**: 11 nodes, 10 edges, connected, `cyc = 0` (tree).

### 2.1 Netlist (tail -> head)

| Winding | DOF | Tail | Head | Role |
|---|---|---|---|---|
| w1 = a1 | DOF1 | L7 | L1 | leaf at L1 |
| w2 = c1 | DOF1 | L5 | L1 | leaf at L1 |
| w3 = a2 | DOF2 | L10 | X | into the star |
| w4 = c2 | DOF2 | L1 | X | trunk L1-X |
| w5 = a3 | DOF3 | L6 | L4 | leaf at L6 |
| w6 = c3 | DOF3 | L6 | L2 | leaf at L6 |
| w7 = a4 | DOF4 | X | L9 | out of the star |
| w8 = c4 | DOF4 | X | L8 | out of the star |
| w9 = a5 | DOF5 | **L1** | L6 | trunk L1-L6; **re-terminated** (reference B: L3 -> L6) |
| w10 = c5 | DOF5 | L1 | L3 | trunk L1-L3; L3 is a leaf leg |

The only difference from reference B10-OPT-B is the tail of w9: **L3 -> L1**.
Rows, names, and DOF pairs are unchanged. This is why the internal id `w9x1`
is still used: it is a one-end re-termination of the B netlist, not a new
winding order.

### 2.2 Node degrees and bias-point leg currents

```
node degrees:  X 4 | L1 5 | L6 3 | L3 1 | L2,L4,L5,L7,L8,L9,L10 = 1
bias currents: X 0 | L1 +5 | L5 +5 | L6 +5 | L7 +5 | L10 +5
                    | L2 -5 | L3 -5 | L4 -5 | L8 -5 | L9 -5      [A]
```

The bias point is `delta = 0`, i.e. all ten winding currents equal `i_bias`.
The signs follow the leg-current convention of the model: positive means the
leg sources current into a winding tail; negative means it sinks current from
a winding head.

### 2.3 Bias-point potential map

With the lowest leg at 0 V (`u_k = R*i_bias = 5 V` along the tree):

| Leg | L2 | L4 | L8 | L9 | L3 | L6 | L1 | L10 | L5 | L7 |
|---|---|---|---|---|---|---|---|---|---|---|
| p [V] | 0 | 0 | 0 | 0 | 5 | 5 | 10 | 10 | 15 | 15 |

The span is 15 V = 3 * R*i_bias, so the static auxiliary criterion
`U_dc,avg = 15 V` (see `07-auxiliary-criteria.md`). At `U_dc = 200 V`, the
worst winding still has 190 V (95 %) available for control.

---

## 3. Why "single star point" and why L1 is driven

The star point X joins the **complete DOF2 pair in** (`w3 = a2` and `w4 = c2`,
both with their heads at X) against the **complete DOF4 pair out**
(`w7 = a4` and `w8 = c4`, both with their tails at X). KCL at X therefore gives

```
i_a2 + i_c2 = i_a4 + i_c4 = 2*i_bias
```

for every control current, so the bias and the deltas cancel at X exactly.
There is no split-capacitor bus and no need to control the star potential.

L1 is **not** a star: it is a driven leg of degree 5. It joins DOF1's pair
(`w1`, `w2`, both heads at L1), DOF5's pair (`w9`, `w10`, both tails at L1),
and the trunk `w4 = L1 -> X`. The two pairs' common-mode flows are each
`2*i_bias = 10 A`; they enter and leave L1 in opposite directions and cancel
exactly, leaving the `w4` current:

```
i_L1 = i_w4 = i_bias - delta2/2
```

Thus the bias-point current is 5 A and the envelope is 7.5 A, even though L1
touches four windings plus the trunk. This is the structural reason the
topology keeps the 5 A current class while removing the DOF5 ripple.

The price is visible in the bus criterion: the path that binds `U_dc,min` is
`L10 -> L4` (or the tied `L2 -> L10`), a four-term path with no cancellation,
giving 2.3501 phi. Reference B uses a two-term cancellation on `L1 -> L3` and
reaches 2.0252 phi, but pays for it with the opposite-orientation DOF5 pair on
L3 and a 0.5 A differential ripple.

---

## 4. Reference netlists (comparison only)

### 4.1 Reference B10-OPT-B

| Winding | Tail | Head |
|---|---|---|
| w1 | L7 | L1 |
| w2 | L5 | L1 |
| w3 | L10 | X |
| w4 | L1 | X |
| w5 | L6 | L4 |
| w6 | L6 | L2 |
| w7 | X | L9 |
| w8 | X | L8 |
| w9 | **L3** | L6 |
| w10 | L1 | L3 |

Degrees: `X 4 | L1 4 | L6 3 | L3 2`; bias currents:
`X 0 | L1 0 | L3 0 | L2 -5 | L4 -5 | L8 -5 | L9 -5 | L5 +5 | L6 +5 | L7 +5 | L10 +5`.
Criteria: 57.8429 V = 2.0252 phi, 5 A / 7.5 A. DOF5 is the
opposite-orientation pair on L3, so its differential ripple is 0.499 A.

### 4.2 Reference direction_569 = -1

Same wiring as B, with the current directions of **w5, w6 and w9 reversed**:

| Winding | Tail | Head |
|---|---|---|
| w5 | L4 | L6 |
| w6 | L2 | L6 |
| w9 | L6 | L3 |

The other seven rows are unchanged. Degrees remain `X 4 | L1 4 | L6 3 | L3 2`;
bias currents are `X 0 | L1 0 | L2 +5 | L4 +5 | L5 +5 | L7 +5 | L10 +5 |
L3 -10 | L6 -5 | L8 -5 | L9 -5`. Criteria: 86.4054 V = 3.0252 phi,
10 A / 10 A. All five differential ripples are at the 0.05 A limit, but L3
carries `i_a + i_c = 10 A`. The primary topology gets the same ripple at
2.3501 phi and 5 A legs, so this reference is dominated.

---

## 5. Position in the 10-leg class

| Topology | Legs | `U_dc,min/phi` | Bias / envelope | Note |
|---|---|---|---|---|
| **Single-star-point ten-leg (w9x1)** | **10** | **2.3501** | **5 / 7.5 A** | **primary: all-clean 5 A class** |
| B10-OPT-B (reference) | 10 | 2.0252 | 5 / 7.5 A | minimum bus, DOF5 0.5 A |
| B10-OPT-A | 10 | 2.0000 | 10 / 15 A | 10 A-class optimum |
| B10-OPT-C | 10 | 1.6751 | 15 / 22.5 A | 15 A-class optimum |
| B9P10 + driven star | 10 | 3.0503 | 5 / 7.5 A | dominated by B10-OPT-B |
| B9-OPT-A + driven star | 10 | 2.3501 | 10 / 15 A | dominated by B10-OPT-A |

The last row has the same 2.3501 phi number as the primary but is a **different
topology** (a 9-leg shape with its star driven). Do not confuse the two; the
primary is a native 10-leg netlist with a single undriven star and 5 A legs.

---

## 6. Reproduce

```matlab
addpath('<skill>/scripts')
b10optb_build_all            % primary model, 8 ms closed-loop run
b10optb_simcheck             % channel-by-channel currents and ripple
b10_fig                      % fig1_topology.png
b10_check                    % criteria table and dominance
b10optb_build_all('B'); b10optb_simcheck       % reference B
b10optb_build_all('d569'); b10optb_simcheck    % reference d569 = -1
b10optb_build_all('w9x1')    % restore the primary model
```

`b10optb_build_all` deletes and rebuilds `<skill>/model/SINGLE_STAR_TEN_LEG_AMB.slx`.
The packaged model is always the primary topology; the reference builds
overwrite it temporarily.
