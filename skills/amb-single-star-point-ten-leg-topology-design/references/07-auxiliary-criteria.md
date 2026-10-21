# Reference 7: auxiliary criteria that use only static quantities

> **Motivation.** The main criterion `U_dc,min(phi)` assumes that the control
> current amplitude `delta_max` and the current-loop bandwidth `f_c` are known.
> In practice the control-current range is often hard to estimate up front
> (load, controller tuning, rotor eccentricity). Auxiliary criteria that look
> only at the **bias point** provide a second, independent view: they use only
> `R`, `i_bias`, the topology and a given bus, and are completely independent
> of `delta_max` and `f_c`.
>
> The primary topology is the single-star-point ten-leg topology; **w9x1** is
> its internal id (same netlist).
>
> Companion tool: `b10_aux_criteria.m` (prints every number below in under a
> second; no Simscape required).

---

## 1. Auxiliary criterion 1: average-current bus requirement U_dc,avg

**Definition.** Let all ten windings carry only the bias current, i.e.
`u_k = R*i_bias` (the `phi = 0` special case of the main criterion). The bus
must then cover the span of the leg potentials at that bias point:

```
U_dc,avg  =  max_leg p  -  min_leg p
          =  max over all leg pairs  |n_b| * (R*i_bias)          n_b = sum_k d_k
```

`n_b` is the signed number of terms on the two-leg path (corner-independent;
see `06-bus-voltage-how-to.md` section 2.2). There is no control current to
align, so the only thing to do is find the pair of legs with the highest
stacked bias drop; take the maximum of `|n_b|`.

**Primary topology example:** `max|n_b| = 3` (the extremes are
`L5 <-> L2` / `L5 <-> L4`, three bias drops apart in potential), so

```
U_dc,avg = 3 * 5 V = 15 V
```

**Reference B10-OPT-B example:** `max|n_b| = 4` (e.g. `L4 <-> L5` or
`L2 <-> L5`), so `U_dc,avg = 20 V`.

What it answers: "what bus is needed to hold the ten bias currents with no
voltage left for control?" It is the **absolute lower bound** of the bus; no
control current, however large, can make the bus smaller than this.

---

## 2. Auxiliary criterion 2: bias-point potential map and winding headroom

### 2.1 Bias-point potential map

Set the lowest leg to 0 V and back-substitute `u_k = R*i_bias = 5 V` along the
tree. For the primary topology:

| Leg | L2 | L4 | L8 | L9 | L3 | L6 | L1 | L10 | L5 | L7 |
|---|---|---|---|---|---|---|---|---|---|---|
| p [V] | 0 | 0 | 0 | 0 | 5 | 5 | 10 | 10 | 15 | 15 |

The highest legs are 15 V above the lowest (three bias drops). This is the same
15 V as `U_dc,avg`.

For reference B the map is: L2/L4 = 0, L6/L8/L9 = 5, L3 = 10, L1/L10 = 15,
L5/L7 = 20 V; the span is 20 V.

### 2.2 Available voltage headroom per winding

With the bus fixed, to drive winding k positive the tail must be raised toward
the top rail, while its head is already `p_head - p_min` above the lowest leg.
That part of the bus is already used by the bias drop, so

```
V+_k  =  U_dc - ( p_head(k) - p_min )          % positive headroom
V-_k  =  U_dc - ( p_max - p_tail(k) )          % negative headroom (symmetric)
```

**Scalar criterion:** `V_min = min over the ten windings of min(V+_k, V-_k)`,
together with the binding winding.

**Primary topology at `U_dc = 200 V`:**

- `w2 = L5 -> L1`: `p_head = p_L1 = 10 V`, so `V+_2 = 200 - 10 = 190 V`.
- `w6 = L6 -> L2`: `p_max - p_tail = 15 - 5 = 10`, so `V-_6 = 190 V`. The
  same holds for `w5 = L6 -> L4`.
- **Worst case: 95 % of the bus is available for control; 5 % is pinned by the
  bias drops.**

**Reference B at 200 V:** 185 V (92.5 %), pinned by w2 / w6.

### 2.3 Two conventions

1. **Common-mode placement.** The map above fixes the lowest leg at 0 V, the
   placement most favourable to positive drive and always realisable by the
   duty assignment. A controller using **min-max common-mode injection**
   (centred duties) gives each side half of the remaining margin:
   `(U_dc - span)/2`. The **ranking is unchanged**, but absolute numbers
   change; compare topologies with the same convention. The `duty` column of
   the tool is `p/U_dc` for the lowest-leg-at-0 placement (the primary spans
   0-7.5 %); the Simscape model then centres the ten duties around 50 %.
2. **Static only.** This is a dc headroom, not a dynamic requirement; the
   `L*di/dt` requirement is the main `phi` criterion.

---

## 3. Five-variant comparison

| Topology | `U_dc,avg` [V] | `max abs(n_b)` | `min V+` @200 V | `min V-` @200 V | Binding winding |
|---|---|---|---|---|---|
| **Primary (single-star-point, w9x1)** | **15** | **3** | **190 (95.0 %)** | **190** | **w2 / w6** |
| B10-OPT-B (reference) | 20 | 4 | 185 (92.5 %) | 185 | w2 / w6 |
| w4 -> L3 (`w4x3`) | 20 | 4 | 185 (92.5 %) | 185 | w2 / w6 |
| w4 -> L3 + `d569 = -1` | 20 | 4 | 185 (92.5 %) | 185 | w6 / w8 |
| B10-OPT-B + `d569 = -1` | 15 | 3 | 190 (95.0 %) | 190 | w6 / w8 |

**Reading the table**

- `U_dc,avg` depends only on how the bias drops stack: B and `w4x3` need
  20 V (`max|n_b| = 4`); the primary and the pure direction flip need 15 V
  (`max|n_b| = 3`), because their bias potential maps are flatter.
- The headroom column differs by only 5 V at 200 V because the bus is loose
  relative to the bias drop (15-20 V). The difference matters more as the bus
  is tightened.
- The auxiliary ranking can be **completely different** from the main
  criterion: a large `U_dc,avg` does not imply a bad bus, and a small one does
  not imply a good bus. They measure two non-overlapping parts of the bus cost;
  do not sort them in one table. For a combined view use
  `06-bus-voltage-how-to.md`.

---

## 4. Tool

```matlab
addpath('<skill>/scripts')
b10_aux_criteria              % primary topology, U_dc = 200 V (defaults)
b10_aux_criteria('B', 200)    % reference B10-OPT-B
b10_aux_criteria('d569', 200) % reference direction_569 = -1
```

Output: `U_dc,avg` (with `max|n_b|` and the binding pair), the bias-point
potential map (with duty), `V+`/`V-` for every winding, and the most
restricted winding with its percentage of `U_dc`. Everything uses only `R`,
`i_bias`, the topology and the given `U_dc`; no `delta_max` or `f_c`
calculation is called.

---

## 5. What these criteria do **not** include

- **No dynamic requirement:** `L*di/dt` is covered by the main `phi`
  criterion (`U_dc,min(phi)`).
- **No ripple:** ripple is a switching-process quantity; see
  `03-ripple-and-direction.md`.
- **No leg current:** that is the `Idc_bias` / `Idc_env` criterion (same `A`
  matrix, pure algebra).
- **The common-mode placement changes the absolute `V+`/`V-` values**
  (ranking unchanged); state the convention when comparing.
