# Reference 4: device-level Simscape model guide

> Model: `../model/SINGLE_STAR_TEN_LEG_AMB.slx`
> Primary netlist: single-star-point ten-leg topology (internal id `w9x1`).
> Reference builds: `b10optb_build_all('B')` and `b10optb_build_all('d569')`.

---

## 1. In one sentence

A ten-leg / ten-winding AMB power stage built from Simscape devices, with all
ten winding currents closed-loop controlled: 200 V bus, 5 A bias, 1 A control
current per DOF (adjustable in the parameter file), a Scope that shows all ten
winding currents, and **no overlapping wires between different nets**.

One-key reproduction:

```matlab
addpath('<skill>/scripts')
b10optb_build_all        % rebuild + wire + overlap check + netlist check + 8 ms run
b10optb_simcheck         % simulate only; print currents / differential / ripple
b10optb_fig              % winding-current plot -> ../assets/fig4_winding_currents.png
b10optb_fig_legs         % fig8 (primary + references) and fig8b (L6 zoom)
b10_fig                  % primary topology -> ../assets/fig1_topology.png
b10_fig('B'); b10_fig('d569')   % reference topology figures
b10optb_build_all('B'); b10optb_simcheck       % reference B
b10optb_build_all('d569'); b10optb_simcheck    % reference d569 = -1
b10optb_build_all('w9x1') % restore the primary model
```

`b10optb_build_all` deletes and rebuilds the model file, so the model always
matches the scripts. Do not hand-edit the wiring; change the builder or the
netlist instead.

---

## 2. Model structure (two layers + a wireless controller, about 950 blocks)

| Location | Content |
|---|---|
| root | `Udc` (Simscape voltage source, `dc_voltage = 200`), `GND`, `Solver`, `Leg1..Leg10`, `W1..W10`, `MuxI`, `Scope_Iw`, `Iw_log` |
| root, left | PWM area: one 20 kHz triangular carrier + ten rows (`From d<k>`, two comparators, two x15 drivers, two Goto tags) |
| `Ctrl` | **ten current loops** (no ports: reads `iw<k>` tags, publishes `duty<k>` tags) |
| `Legk` | 2 x IGBT (Ideal, Switching) + 2 freewheel diodes + 2 Simulink-PS converters; ports (left edge, top to bottom) P = bus +, M = midpoint, N = bus - |
| `Wk` | current sensor + inductor + resistor in series; ports T (tail) / H (head), signal output `i` |

- A leg is a standard half bridge: upper C->P, E->M; lower C->M, E->N; upper
  diode A->M / K->P; lower diode A->N / K->M. The average midpoint potential is
  `d_k*U_dc`; `d_k = 0.5 + phi_k/U_dc` with X as the potential reference.
- Gates: `tri < d` drives the upper switch, `tri > d` the lower one
  (complementary, no dead time; this topology has no shoot-through path in the
  ideal switch model).
- The controller is a current loop, not a fixed duty: the duty is computed by
  `Ctrl`. The steady-state duty printed by `b10optb_params` is only a
  reference / starting value.

**Channel convention (important):** the Scope and `iw_log` channel k is
winding `wk`, k = 1..10, **not** netlist row order.

- Each `W<r>` subsystem (netlist row r) publishes its own winding number
  (`iw` + winding number), so `iw9` is always w9.
- `MuxI` channel k is fed by `From iw<k>` and the signal line is named
  `w1..w10`, so the Scope legend shows the winding names.
- The netlist row order only matters for placing the blocks
  (`W1..W10` are rows w4 w6 w5 w2 w9 w10 w1 w8 w7 w3). Every external
  interface is numbered by winding.
- `b10optb_simcheck` prints `ch k = wk` and its reference value, so a
  channel-order mistake is visible immediately.

---

## 3. Parameters (`b10optb_params.m`)

| Quantity | Value | Note |
|---|---|---|
| `U_dc` | 200 V | user-specified, not the minimum |
| `R_w` / `L_w` | 1 ohm / 500 uH | per winding |
| `i_bias` | 5 A | fixed bias per winding |
| `i_ctrl` | 1 A / DOF | **adjustable**; or `i_ctrl_by_dof = [1 1 0.5 0 1]` |
| `f_sw` | 20 kHz | carrier period 50 us |
| `T_stop` | 8 ms | winding time constant 0.5 ms; 8 ms is enough for steady state |
| solver | VariableStepAuto, `MaxStep = 1e-6`, `RelTol = 1e-4` | see the sibling half-H skill |

### 3.1 Current loop

The ten legs are the actuators and the ten winding currents are the controlled
outputs, but the windings share legs, so the loops are coupled. A static
decoupling matrix turns them into ten SISO loops:

```
A  = winding-node incidence matrix (rows w1..w10, columns X,L1..L10)   u = A'*phi
AL = A with the star column removed   ->   AL*phi_L = u   ->   phi_L = M*u
```

This is the same tree back-substitution used by the open-loop design, with
`u = R*i_ref` replaced by the PI output. Per-channel chain:

```
iw<k> -> 2nd-order anti-alias filter (6 kHz, zeta=0.707) -> ZOH(Ts) -> Memory
      -> err = ref - fbk -> Kp*err + conditional-integral term
      -> Saturation(+-1, per-unit) -> U_dc*M -> common-mode centring
      -> +0.5 -> Saturation[0,1] -> duty<k>
```

- `Kp = 2*pi*f_c*L/U_dc = 0.023562`,
  `Ki = 2*pi*f_c*R/U_dc = 47.1239` (per-unit; `f_c = 1500 Hz = f_sw/13`).
  The PI zero `Ki/Kp = R/L` cancels the winding pole.
- Anti-windup: conditional integration (freeze when `|x| >= 1` and x*err >= 0),
  plus an integrator limit of +-1 as a second line of defence.
- Feedback is filtered before sampling; ZOH + Memory break the algebraic loop.

---

## 4. Netlist used by the model (primary)

```
w4  L1  -> X      w6  L6  -> L2     w5  L6  -> L4     w2  L5  -> L1     w9  L1  -> L6
w10 L1  -> L3     w1  L7  -> L1     w8  X   -> L8     w7  X   -> L9     w3  L10 -> X
```

The star X is the single electrical net joining the w3/w4/w7/w8 terminals; it
is floating and is not grounded. The only difference from reference B is
`w9: L1 -> L6` instead of `L3 -> L6`.

The builder reads the netlist and wires every winding explicitly. The star node
is a vertical trunk at the right of the legs; no two different nets are drawn
on top of each other. `b10optb_line_overlap` counts only different-net
collinear overlaps; crossings and same-net coincident segments are allowed.

---

## 5. Acceptance of the primary model

Run: `b10optb_build_all('w9x1')`, 8 ms, `i_ctrl = 1 A/DOF`, `U_dc = 200 V`.

```
winding currents w1..w10 [A]: 5.502 4.502 5.503 4.503 5.504 4.503 5.504 4.503 5.503 4.503
reference        w1..w10 [A]: 5.500 4.500 5.500 4.500 5.500 4.500 5.500 4.500 5.500 4.500
max error = 0.0038 A (0.08 %)
ripple p-p [A]             : 0.282 0.231 0.265 0.244 0.283 0.232 0.265 0.236 0.267 0.222
overlap check: 1468 segments in 375 nets -> 0 overlapping different-net pairs
topology check: 1     (10/10 windings ok, 11 nodes distinct)
differential i_a-i_c [A]   : 1.0002 1.0007 1.0016 1.0019 1.0009
differential ripple p-p    : 0.054 0.051 0.054 0.053 0.050
```

- Currents are time-weighted averages over the last 20 % of the run.
- Winding ripple is 0.222-0.283 A p-p at 20 kHz; differential ripple is
  0.050-0.054 A p-p on all five DOFs (the structural limit).
- Changing `i_ctrl` to 0.5 gives 5.25 / 4.75 A; the control current is
  adjustable.
- With realistic device drops (`b10optb_devtest`: IGBT Vf = 0.8 V,
  diode Vf = 0.6 V / Ron = 0.3 ohm), the closed loop measures 0.10 % error;
  see `05-verification-log.md`.

### 5.1 Reference builds

| Build | Leg current (bias) | Differential ripple p-p |
|---|---|---|
| `b10optb_build_all('B')` | 5.51 A | 0.055 0.053 0.055 0.052 **0.499** |
| `b10optb_build_all('d569')` | 10.01 A | 0.053 0.051 0.054 0.054 0.051 |

Both use the same model file and the same controller; only the netlist changes.
Rebuild the primary afterwards.

### 5.2 Reading fig8 and fig8b: leg current is not winding current

`fig8_leg_currents.png` plots the **leg currents** L1..L10, reconstructed from
the ten winding currents by KCL. The legend says `L1..L10`, not `w1..w10`. For
example `I_L6 = i_w5 + i_w6 - i_w9`. The narrow notches on L1/L6 are the summed
fast edges of the winding currents (each winding sees a ~1.4 us differential
PWM pulse), not a solver step artifact and not a channel-order error. A 10x
smaller `MaxStep`/`RelTol` leaves the L6 notch amplitude unchanged
(0.4947 A -> 0.4951 A). `fig8b_leg_currents_zoom.png` zooms into L6 and shows
the finite-width notch together with its three winding components; the
individual winding currents are clean 20 kHz sawtooths (fig4).

---

## 6. "Different nets must not overlap"

`b10optb_line_overlap` first reconstructs the **electrical nets** from the
model (adjacent segments of one line, branches on one port, and same-name
connections across subsystem ports are all one net), then counts only segment
pairs that belong to different nets and are collinear for more than 2 px.
Crossings are fine; two segments of the same net may coincide (they are one
node and are drawn as one line). Coordinates are compared only inside the same
canvas (subsystem).

The primary build reports **1468 segments in 375 nets -> 0** overlapping
different-net pairs. The reference builds report 1469 (B) and 1470 (d569)
segments, also **0** overlaps.

---

## 7. Pitfalls (all encountered in practice)

1. **Goto/From visibility.** A root-level Goto defaults to `local` and is
   invisible inside a subsystem - without an error; the From reads a constant
   0. Cross-layer tags (gates, currents, duties) must be `global`.
2. **Channel order must not leak the row order.** If the Scope is wired in
   netlist row order, "channel 9 and 10" are w7 and w3, and the user sees
   "w9 = w10 = 5.5 A" although the controller is correct. Fix: tag by winding
   number, name the Mux input lines `w1..w10`, and print `ch k = wk` in the
   self-check.
3. **One unconnected input silently turns a PI into P.** An anti-windup
   subsystem with an unwired `Ki` input reads 0 and the steady-state error is
   about 1 A. The builder checks every input port and errors out if any is
   unconnected.
4. **Controller channels must be in winding order**, not netlist row order,
   otherwise the decoupling matrix is fed the wrong order and the simulation
   diverges to hundreds of amperes. `b10optb_params` returns `ctrlRow` and
   `wOfRow` and the builder asserts the mapping.
5. **Simscape library names contain newlines.** Resolve by
   `find_system(...,'Name',...)` and `strrep(...,char(10),' ')` instead of
   hard-coding the path.
6. **Two PS converters cannot be connected directly.** Measure physical
   quantities through a real sensor (current sensor / voltage sensor).
7. **Simulink handles are not integers** and port handles are recycled; do not
   store them across loops or use them as array indices.
8. **Port geometry changes** while a subsystem is being edited; read the port
   positions again at wiring time and do a few "snap" passes at the end.
9. **Physical connection lines have no SrcPortHandle** (-1), and a polyline is
   split at branches; net extraction must combine line handles, port
   `Port/Side`, and geometric contact.
10. **`struct()` expands cell fields** into struct arrays; assign fields one by
    one.
11. **MATLAB cannot index a function-call result directly** (`pp(h)(2)` is a
    syntax error); use a temporary variable.
12. **Do not give a model the same name as an existing .slx on the path**;
    shadowing makes a function call open the model instead.

---

## 8. File index

| File | Purpose |
|---|---|
| `../model/SINGLE_STAR_TEN_LEG_AMB.slx` | packaged primary model (generated; do not hand-wire) |
| `b10optb_params.m` | all parameters + reference currents + duty back-substitution |
| `b10optb_build_all.m` | one-key build + wire + overlap + netlist + run |
| `b10optb_build_ctrl.m` | build the ten current loops (PI + anti-windup + M) |
| `b10optb_simcheck.m` | simulate and print currents / differential / ripple |
| `b10optb_line_overlap.m` | different-net line-overlap checker |
| `b10optb_nets.m` | electrical-net extraction (shared by both checks) |
| `b10optb_check_topology.m` | read-back netlist check (10 edges, 11 nodes) |
| `b10optb_fig.m` | winding-current plot (fig4) |
| `b10optb_fig_legs.m` | leg currents of primary + references (fig8) and L6 zoom (fig8b) |
| `../assets/fig8b_leg_currents_zoom.png` | supplementary zoom of fig8: L6 = w5+w6-w9 |
| `b10optb_devtest.m` | realistic device drops, closed-loop accuracy |
| `b10optb_diag.m` | read-only model diagnostics |
| `b10optb_porttest.m` | empirical device-port orientation probes |
| `b10optb_netlist.m` | all netlist variants (primary / B / d569 / historical) |
| `sstl_def.m` | primary topology descriptor |
| `sstl_fig_model.m` | export model/subsystem screenshots |
| `b10_fig.m` | topology schematics (fig1 / fig9 / fig10) |
