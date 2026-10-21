---
name: amb-single-star-point-ten-leg-topology-design
description: Design, model, and verify a SINGLE-STAR-POINT TEN-LEG active magnetic bearing (AMB) power topology - ten half-bridge legs drive ten windings, exactly one undriven star node X (degree 4) joins a complete DOF pair in against a complete DOF pair out, and every DOF pair meets a common node with the SAME orientation so all five differential ripples sit at R*delta*Ts/(2L). The primary netlist is the internal variant w9x1 (w9 re-terminated from L3 to L1); w9x1 and "single-star-point ten-leg topology" denote exactly the same netlist. Criteria = 2.3501 phi = 67.1239 V bus, 5 A bias / 7.5 A envelope leg current, cyc = 0 (tree), all-clean ripple 0.05 A. Contents = authoritative netlist and reference netlists (B10-OPT-B and direction_569 = -1), class model and optimality, tree back-substitution and decoupling matrix, ten per-unit PI current loops with conditional-integration anti-windup, a device-level Simscape reference model and one-key rebuild/check scripts, the structural ripple criterion, the hand calculation of U_dc,min, static auxiliary criteria, acceptance log and pitfalls. Use when modeling or reviewing an AMB drive with ten legs, one shared star point, or shared-leg winding pairs; when deciding winding numbering versus leg numbering; when evaluating ripple versus leg current; or when a runnable Simscape AMB topology verification model is needed.
metadata:
  author: AMB topology workspace
  version: "1.0"
  verified-with: MATLAB/Simulink/Simscape R2025b (25.2) Update 6
---

# Single-Star-Point Ten-Leg AMB Topology

This skill is about a **ten-leg / ten-winding, one-undriven-star, cycle-free**
AMB power topology. The canonical name is **single-star-point ten-leg
topology**. The internal variant id **w9x1** is kept for continuity with the
development record: it is the same netlist, obtained from the reference
B10-OPT-B by re-terminating w9 from L3 to L1. Whenever this skill says
`w9x1`, read "single-star-point ten-leg topology".

The topology is a tree (`cyc = 0`), so with +/-5 % winding R/L mismatch the
static bias error is exactly zero. Its distinguishing feature is that **all five
DOF pairs meet a common node with the same orientation**, so all five
differential ripples are at the `R*delta*Ts/(2L)` limit, while the leg
currents stay in the 5 A class. The cost is a 16 % higher bus than the
minimum-bus reference B10-OPT-B.

---

## When to use this skill

- Model, simulate, or verify this ten-leg / one-star topology.
- Design or review an AMB power stage where **several windings share legs** and
  one node is hard-wired as a star, so KCL enforces pair currents.
- Decide the mapping between **winding numbers** and **leg numbers**, or debug
  Scope/log channel order.
- Evaluate the trade-off between **differential ripple** and **leg current**
  when a winding end is re-terminated or a current direction is reversed.
- Use the included closed-loop, device-level Simscape model as a starting
  point for a topology verification.

## When not to use it

- Detailed engineering design (loss, thermal, EMI, protection, bus capacitors,
  digital controller implementation). The model deliberately omits these.
- Position control, rotor dynamics, force model, saturation, mutual inductance.
- Other leg counts (9-leg or 11-leg classes) - this skill covers the 10-leg
  single-star-point class; B10-OPT-A/C and the 9-leg split references appear
  only as comparison points.

---

## 0. Result at a glance

1. **Topology**: 10 driven legs + 1 undriven star X (degree 4) + tree
   (`cyc = 0`). Primary criteria: **67.1239 V = 2.3501 phi**, **5 A** bias leg
   current, **7.5 A** envelope, **all five differential ripples 0.05 A**.
2. **What the star buys**: X joins the complete DOF2 pair in against the
   complete DOF4 pair out, so the bias and the deltas cancel at X exactly.
   **L1 is not a star**: it is a driven leg of degree 5 joining DOF1's pair,
   DOF5's pair and the trunk w4. The two pair common-mode currents (10 A each)
   cancel exactly, leaving the w4 current (5 A bias, 7.5 A envelope).
3. **Primary versus references**:

   | Topology | Bus | Bias / envelope | DOF1..5 differential ripple | Role |
   |---|---|---|---|---|
   | **Single-star-point ten-leg (w9x1)** | **2.3501 phi** | **5 / 7.5 A** | **0.054 0.051 0.054 0.053 0.050 A** | **primary** |
   | B10-OPT-B | 2.0252 phi | 5 / 7.5 A | 0.055 0.053 0.055 0.052 **0.499 A** | reference: minimum bus |
   | direction_569 = -1 | 3.0252 phi | 10 / 10 A | 0.053 0.051 0.054 0.054 0.051 A | reference: all clean, 10 A legs |

   The primary is the **all-clean 5 A-class** choice. It is 16 % higher on bus
   than reference B, which has a 0.5 A DOF5 ripple. Reference direction_569 = -1 is clean but
   needs 10 A legs and a 29 % higher bus; the primary strictly dominates it.

---

## 1. Topology definition (authoritative netlist)

Nodes: `X` = the single undriven star; `L1..L10` = the ten driven legs.
Current direction is **tail -> head**; all ten winding currents are positive at
the bias point.

| Winding | DOF | Tail | Head | Note |
|---|---|---|---|---|
| w1 = a1 | DOF1 | L7 | L1 | leaf at L1 |
| w2 = c1 | DOF1 | L5 | L1 | leaf at L1 |
| w3 = a2 | DOF2 | L10 | X | into the star |
| w4 = c2 | DOF2 | L1 | X | trunk L1-X |
| w5 = a3 | DOF3 | L6 | L4 | leaf at L6 |
| w6 = c3 | DOF3 | L6 | L2 | leaf at L6 |
| w7 = a4 | DOF4 | X | L9 | out of the star |
| w8 = c4 | DOF4 | X | L8 | out of the star |
| w9 = a5 | DOF5 | **L1** | L6 | **re-terminated**; reference B has L3 -> L6 |
| w10 = c5 | DOF5 | L1 | L3 | trunk L1-L3; L3 is a leaf leg |

- **DOF pairs**: `(w1,w2) (w3,w4) (w5,w6) (w7,w8) (w9,w10)`;
  `i_a - i_c` is the control current, `i_a = i_bias + delta/2`,
  `i_c = i_bias - delta/2`.
- **Node degrees**: `X 4 | L1 5 | L6 3 | L3 1`; the other seven legs are 1.
- **Bias-point leg currents** (delta = 0): `X 0 | L1 +5 | L5 +5 | L6 +5 |
  L7 +5 | L10 +5 | L2 -5 | L3 -5 | L4 -5 | L8 -5 | L9 -5` A.
- The star X's two sides must each be a **complete DOF pair**: in
  `(w3,w4) = DOF2`, out `(w7,w8) = DOF4`. Then the bias and the deltas
  cancel at X simultaneously.
- **Only one row differs from reference B**: w9's tail is L1 instead of L3.
  This is the whole `w9x1` change; it is not a new winding order.

---

## 2. Criteria and class context

- Criterion vector `(legs, cyc, U_dc,min/phi, Idc_bias, Idc_env)`, with
  `phi = R*delta_max + L*sigma = 28.5619 V`
  (`R = 1 ohm, L = 500 uH, i_bias = 5 A, delta_max = 5 A, f_c = 1500 Hz`).
- `U_dc,min` is computed twice: linear program `tp_floor_leg.m` and
  spanning-tree recursion `b9_floor_fast.m`; the two must agree. The
  star-node KCL residual must be exactly 0.
- In the 10-leg class: B10-OPT-B is the **bus optimum of the 5 A class**
  (2.0252 phi); B10-OPT-A is the 10 A-class point (2.0000); B10-OPT-C is the
  15 A-class point (1.6751). The primary is 2.3501 phi, chosen for its clean
  ripple, not for the minimum bus. The "drive one star of the 9-leg shape"
  references are dominated and are not alternatives.
- A hand calculation of `U_dc,min` is in
  `references/06-bus-voltage-how-to.md`:
  `U_dc,min = n_phi*(phi/2) + n_b*(R*i_bias)`. The primary's binding path is
  `L10 -> L4` (or the tied `L2 -> L10`): 4*14.2810 + 2*5.0 = 67.1239 V.
- Static auxiliary criteria (no control current): `U_dc,avg = 15 V` and
  `V_min = 190 V` at 200 V (95 %) for the primary; see
  `references/07-auxiliary-criteria.md`.

---

## 3. Control law: tree back-substitution -> leg potentials -> duties

The average midpoint potential of leg k is `v_k = d_k*U_dc`. With the
incidence matrix `A` (rows w1..w10, columns X,L1..L10):

```
u = A' * phi            % u_k = phi(tail) - phi(head) = winding voltage
AL = A without the star column  =>  phi_L = M*u ,  M = inv(AL)
d_k = 0.5 + phi_k/U_dc  % then centre all ten duties in [0,1]
```

- **Open-loop design**: `u_k = R*i_ref_k` (steady-state drop) gives the leg
  potentials and duties directly.
- **Closed loop**: replace `u` by the ten PI outputs; put `U_dc*M` between
  the controllers and the modulator, and the ten loops become decoupled SISO.
- The star potential is set by KCL and must not be controlled directly; as long
  as the reference currents keep both sides of the star balanced, KCL closes
  automatically.

---

## 4. Ripple: the structural criterion and the reference variants

**Criterion**: if the two windings of a DOF meet a **common node with the same
orientation** (both tails or both heads), that node's potential cancels in the
differential voltage and the ripple is `R*delta*Ts/(2L) = 0.05 A`. If they do
not share a node, or share it with opposite orientations, the differential
expression spans three or more legs and the ripple is about an order of
magnitude larger. The common node may be a **driven leg or the undriven star
X**.

In the primary topology:

| DOF | Common node | Orientation | Differential expression |
|---|---|---|---|
| DOF1 | L1 | both heads | V_L7 - V_L5 |
| DOF2 | X | both heads | V_L10 - V_L1 |
| DOF3 | L6 | both tails | V_L4 - V_L2 |
| DOF4 | X | both tails | V_L9 - V_L8 |
| DOF5 | L1 | both tails | V_L3 - V_L6 |

All five are clean. The measured ripple is 0.054 0.051 0.054 0.053 0.050 A p-p.

> **Reading fig8 correctly.** `fig8_leg_currents.png` plots the **leg currents**
> L1..L10 (KCL sums of the winding currents), not the winding currents w1..w10.
> The narrow notches on L1 and L6 are the summed differential PWM pulses of the
> windings attached to the leg; `fig8b_leg_currents_zoom.png` zooms into L6 and
> shows that they are finite-width (~1.4 us) pulses, not solver spikes. The
> individual winding currents are clean 20 kHz sawtooths (fig4).

**Reference B** has w9 = L3 -> L6 and w10 = L1 -> L3, so DOF5 meets L3 with
**opposite** orientations. Its differential voltage is
`2*V_L3 - V_L6 - V_L1`; the ripple is 0.499 A p-p. That is the price of the
2.0252 phi bus.

**Reference d569 = -1** reverses w5, w6 and w9. All five pairs become clean,
but L3 carries `i_a + i_c = 10 A` and the bus rises to 3.0252 phi. The primary
does the same job with 5 A / 7.5 A legs and 2.3501 phi, so it dominates.

**Do not interleave the carriers.** For reference B, shifting the L1/L3/L6
carriers destroys the nesting of the off intervals and raises the DOF5 ripple
from 0.5 A to 10-20 A. All builds here use one synchronous carrier and centred
duties.

---

## 5. Modeling paradigm: keep / may drop / never drop

**Keep**: (1) a stiff dc bus (ideal voltage source); (2) a two-switch half
bridge with two freewheel diodes per leg; (3) winding = series R-L + current
sensor; (4) a wiring that makes the current unidirectional (the star KCL does
this); (5) differential reference generation; (6) one PI + PWM per winding
(per-unit form).

**May drop**: device drops and switching loss, thermal network, saturation and
mutual inductance, back-EMF, motion and position loop, bus impedance and
protection. In a topology-verification model they only slow the simulation and
hide the switching behaviour being verified. **However**, if the conclusion is
device-related, use the closed loop with realistic `Vf/Ron`: the loop
compensates the drops (measured 0.10 % error), while an open-loop model would
be 20-50 % low.

**Never drop**: the current feedback sampling and the solver step. This model
uses `Ts = 1/f_sw` (ZOH + Memory) and `MaxStep = 1e-6`,
`RelTol = 1e-4`.

**Device port roles** (measured, do not guess): IGBT `LConn(1)` = gate,
`RConn(1)` = C, `RConn(2)` = E; diode `LConn(1)` = anode,
`RConn(1)` = cathode; current sensor `LConn(1)` = +,
`RConn(1)` = I (signal), `RConn(2)` = -; the bus source parameter name is
`dc_voltage`, `LConn(1)` = +, `RConn(1)` = -.

---

## 6. Closed-loop controller recipe (already built in the model)

```
iw<k> -> 2nd-order anti-alias filter (fn = 6 kHz, zeta = 0.707) -> ZOH(Ts)
      -> Memory -> err = i_ref - fbk
      -> Kp*err and conditional-integral anti-windup -> Saturation(+-1 per-unit)
      -> decoupling matrix U_dc*M -> common-mode centring (max+min)/2
      -> +0.5 -> Saturation[0,1] -> duty<k>
```

- `Kp = 2*pi*f_c*L/U_dc = 0.023562`,
  `Ki = 2*pi*f_c*R/U_dc = 47.1239` (per-unit, `f_c = 1500 Hz = f_sw/13`).
  The PI zero `Ki/Kp = R/L` cancels the winding pole.
- Anti-windup: conditional integration (freeze when `|x| >= 1` and
  `x*err >= 0`), plus an integrator limit of +-1.
- Filter the feedback before sampling; ZOH + Memory break the algebraic loop.
- The controller lives in a port-less subsystem and communicates through
  `iw<k>` (read) and `duty<k>` (write) **global** tags, so the carefully
  routed power stage does not change.

---

## 7. Acceptance checklist

| # | Check | Target | Primary result |
|---|---|---|---|
| 1 | Ten winding currents (time-weighted, last 20 %) | < 0.5 % error | 0.08 % |
| 2 | Differential current `i_a - i_c` | 1.0000 +- 0.005 A | 1.0002..1.0019 A |
| 3 | Scope/log channel k | winding wk | verified channel by channel |
| 4 | Netlist read-back (10 edges, 11 distinct nodes) | 10/10 ok | 10/10 |
| 5 | Different-net collinear overlaps | 0 | 0 (1468 segments / 375 nets) |
| 6 | Star-node KCL residual | 0 | 0.00e+00 A |
| 7 | Criteria | 2.3501 / 5 / 7.5 | matches |
| 8 | Change `i_ctrl` to 0.5 | 5.25 / 4.75 A | 5.252 / 4.753 A (0.08 %) |
| 9 | Realistic device drops | < 0.5 % | 0.10 % |
| 10 | Winding ripple | 0.2-0.3 A p-p at 20 kHz | 0.222-0.283 A |

---

## 8. Pitfalls (all actually encountered)

1. **Goto/From visibility**: a root Goto defaults to `local` and is invisible
   inside a subsystem, with no error; the From reads 0. Use
   `TagVisibility='global'` for gates, currents and duties.
2. **Channel order must not leak the row order**: `W1..W10` are placed in
   netlist row order (w4 w6 w5 w2 w9 w10 w1 w8 w7 w3). If the Scope is wired in
   that order, "channels 9 and 10" are w7 and w3 and the user sees
   "w9 = w10 = 5.5 A". Tag by winding number, name the Mux lines w1..w10, and
   print `ch k = wk`.
3. **One unconnected input silently degrades a PI to P**: an anti-windup
   subsystem with an unwired Ki input reads 0; the steady-state error is about
   1 A. The builder checks every input port and errors out.
4. **Controller channels must be in winding order**, not netlist row order,
   otherwise the decoupling matrix is fed the wrong order and the simulation
   diverges to hundreds of amperes. `b10optb_params` returns `ctrlRow` and
   `wOfRow`, and the builder asserts the mapping.
5. **Simscape library names contain newlines**: resolve with
   `find_system(...,'Name',...)` and `strrep(...,char(10),' ')`.
6. **Two PS converters cannot be connected directly**; measure through a real
   sensor.
7. **Simulink handles are not integers** and ports are recycled; do not store
   them across loops or use them as array indices.
8. **Port geometry changes** during editing; read it again at wiring time and
   do several snap passes.
9. **Physical connection lines have no SrcPortHandle** (-1), and a polyline is
   split at branches; net extraction must combine line handles, port
   `Port/Side` and geometric contact.
10. **`struct()` expands cell fields**; assign fields one by one.
11. **MATLAB cannot index a function-call result directly**; use a temporary.
12. **Do not give a model the same name as an existing .slx on the path**;
    shadowing makes a function call open the model instead.
13. **A leg current is not a winding current.** fig8 plots L1..L10, reconstructed
    by KCL; L6 = w5+w6-w9. Its narrow notches are the sum of the differential PWM
    pulses, not a step-size artifact or a channel-order error. See fig8b; the
    winding currents themselves are clean sawtooths (fig4).

---

## 9. Bundle and reproduction

```
amb-single-star-point-ten-leg-topology-design/
  SKILL.md                         this file
  model/SINGLE_STAR_TEN_LEG_AMB.slx  primary model: 10 legs + 10 windings + 10 current loops
  assets/
    fig1_topology.png              PRIMARY topology schematic (netlist, bias, criteria)
    fig2_model_layout.png          model overview
    fig3_controller.png            closed-loop controller subsystem
    fig4_winding_currents.png      ten winding currents + five differential currents
    fig5_leg_internals.png         one half-bridge leg
    fig6_winding_internals.png     one winding branch
    fig7_antiwindup.png            conditional-integration anti-windup
    fig8_leg_currents.png          primary vs reference B vs reference d569 leg currents
    fig8b_leg_currents_zoom.png    ZOOM of fig8: L6 = w5+w6-w9; the notch is a PWM pulse, not a solver spike
    fig9_reference_b10optb_topology.png   reference B topology
    fig10_reference_d569_topology.png     reference direction_569 = -1 topology
  references/
    01-topology-design.md          authoritative primary netlist, structure, references
    02-class-and-criteria.md       class model, criteria, optimality
    03-ripple-and-direction.md     structural ripple criterion, primary, B and d569
    04-model-guide.md              model structure, parameters, closed loop, acceptance
    05-verification-log.md         measured results and checklist
    06-bus-voltage-how-to.md       hand calculation of U_dc,min, with examples
    07-auxiliary-criteria.md       static auxiliary criteria
  scripts/                         add this folder to the MATLAB path
    b10optb_build_all.m            one-key build + wire + overlap + netlist + run
    b10optb_build_ctrl.m           ten PI loops + anti-windup + decoupling matrix
    b10optb_params.m               parameters, reference currents, M, Kp/Ki
    b10optb_netlist.m              primary 'w9x1' + references 'B' / 'd569' + historical
    b10optb_simcheck.m             simulate and print channel-by-channel currents
    b10optb_fig.m / b10optb_fig_legs.m   winding plot / leg-current plot + fig8b zoom
    b10_fig.m                      topology schematics (fig1 / fig9 / fig10)
    b10_check.m                    criteria table + dominance + LP cross-check
    b10_bus_intuition.m            bus-requirement breakdown
    b10_aux_criteria.m             static auxiliary criteria
    b10_variant_check.m            any netlist: criteria + ripple prediction
    b10_reterm_sweep2.m            single-end re-termination search
    b10_undrive.m                  proof that reference B's L1 cannot be a star
    tp_par.m tp_mk.m tp_floor_leg.m b9_floor_fast.m b9_isleg.m   criteria tools
    b9opta_def.m b9p10_def.m b10_def.m   archived class netlists
    sstl_def.m                     primary topology descriptor
    sstl_fig_model.m               export model/subsystem screenshots (fig2/3/5/6/7)
    b10optb_diag.m / b10optb_porttest.m   diagnostics / device-port probes
```

**Shortest path**

```matlab
addpath('<skill>/scripts')   % one line; scripts locate the model and assets themselves
b10optb_build_all            % build + self-check + 8 ms run (~20 s)
b10optb_simcheck             % ten currents and channel order
b10_variant_check            % criteria + ripple prediction (seconds, no Simscape)
b10_fig                      % primary topology figure
```

---

## 10. Deployment

1. Copy the whole `amb-single-star-point-ten-leg-topology-design/` folder to
   your skills directory (or any location).
2. Add only **`<skill>/scripts`** to the MATLAB path:

   ```matlab
   addpath('<skill>/scripts')
   ```

   The scripts use `mfilename('fullpath')` to locate themselves and write the
   model to `<skill>/model/` and figures to `<skill>/assets/`; they do not
   depend on the current folder.
3. Do not put another copy of the same-named scripts (for example a development
   workspace) earlier on the path: MATLAB will use the first one, so
   "it looks like the skill but runs another copy". Check with
   `which b10optb_params`; it should point into `<skill>/scripts/`.
4. Requirements: MATLAB/Simulink + Simscape + Simscape Electrical (verified on
   R2025b 25.2 Update 6). The pure criteria commands (`b10_check`,
   `b10_variant_check`, `b10_bus_intuition`, `b10_aux_criteria`) need only
   MATLAB.

## 11. One-line guidance

> **Default to the single-star-point ten-leg topology (w9x1)**: all five
> differential ripples are at the 0.05 A limit, the leg currents are 5 A / 7.5 A,
> and the bus is 2.3501 phi. Use reference B10-OPT-B only if the 16 % bus
> increase matters more than the 0.5 A DOF5 ripple; use direction_569 = -1 only as a
> historical comparison (10 A legs, 3.0252 phi).
