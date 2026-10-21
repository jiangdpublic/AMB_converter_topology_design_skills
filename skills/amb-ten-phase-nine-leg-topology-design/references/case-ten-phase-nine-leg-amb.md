# Case study: AMB ten-phase-nine-leg drive with an RL load

This is the audit trail behind SKILL.md. It documents how the bundled model was decoded, what was
measured, and how the numbers quoted in the skill were obtained. All measurements come from the bundled
model `model/Ten_phase_nine_leg_with_RL_load.slx` at i_ref = 1 A, U_dc = 200 V,
i_bias = 5 A, 20 ms of simulated time; the model was never modified (only copies were instrumented).

Environment: MATLAB R2025b (25.2.0.3312555 Update 6), Simulink 25.2, Simscape 25.2, Simscape Electrical
25.2, Linux glnxa64.

---

## 1. Inventory of the delivered model

The root system contains 146 blocks:

| BlockType | Count | Role |
|---|---|---|
| SubSystem | 49 | 5 reference generators, 10 PI controllers, 9 PWM generators, 9 half-bridge legs, 10 windings, 3 reference delays, 2 MATLAB Functions, 1 solver configuration |
| SimscapeBlock | 14 | ideal voltage source, electrical reference, 12 "Open Circuit" terminators |
| From / Goto | 38 / 28 | signal routing by tag |
| Constant | 7 | the five i_ref values, the min-max offset, the 9-element broadcast vector |
| Sum | 3 | cmin+cmax, -0.5, ubridge - common mode |
| Step | 3 | start-up staggering of DOF3, DOF4, DOF5 |
| Demux / Product / Gain / Scope | 1 each | 9-way split, scalar-to-vector broadcast, the 0.5 injection gain, the winding-current scope |

Only Simulink + Simscape Electrical are needed. The 12 "Open Circuit" elements are inert: they terminate
unused connection ports on the leg midpoints and on the ends of the DC bus chain, and contribute no
equations.

## 2. What the two sub-circuits are

**Half-bridge leg** (`Bridge_module`): three connection ports per side, shorted pairwise internally -
Vcci/Vcco = DC+, Vsi/Vso = midpoint, Vddi/Vddo = DC-. Devices: an IGBT
(ideal, switching, Vf = 0.8 V, Ron = 1 mohm, Vth = 6 V) with an antiparallel diode
(Vf = 0.6 V, Ron = 0.3 ohm) on each of the two positions. The gate signal is a 0/15 V
physical signal, so a Simulink-PS Converter plus a Gain of 15 sits in front of each gate.

**Winding** (RL_load): the internal chain is L port -> resistor -> inductor -> current sensor -> R port,
so the measured current is positive when it flows **from the L terminal to the R terminal**, and the winding
voltage is v_L - v_R = R*i + L*di/dt. The verified values are R = 1 ohm,
L = 500 uH.

## 3. Netlist (authoritative, read straight from the .slx XML)

Ten windings, nine legs, two hard-wired nodes. wk is the model's RL_load /
RL_load(k-1); Lk is Bridge_module / Bridge_module(k-1):

| Winding | Module | L terminal | R terminal | DOF |
|---|---|---|---|---|
| w1 | RL_load  | L1 | **X1** | DOF1-a |
| w2 | RL_load1 | L2 | **X1** | DOF1-c |
| w3 | RL_load2 | **X1** | L3 | DOF2-a |
| w4 | RL_load3 | **X1** | L4 | DOF2-c |
| w5 | RL_load4 | L4 | L5 | DOF3-a (axial) |
| w6 | RL_load5 | L5 | L6 | DOF3-c (axial) |
| w7 | RL_load6 | L6 | **X2** | DOF4-a |
| w8 | RL_load7 | L7 | **X2** | DOF4-c |
| w9 | RL_load8 | **X2** | L8 | DOF5-a |
| w10 | RL_load9 | **X2** | L9 | DOF5-c |

The nets that define the topology:

```

X1  = { w1.R, w2.R, w3.L, w4.L }              hard-wired star node
X2  = { w7.R, w8.R, w9.L, w10.L }             hard-wired star node
L4  = { w4.R, w5.L }        L5 = { w5.R, w6.L }        L6 = { w6.R, w7.L }
L1 = { w1.L }  L2 = { w2.L }  L3 = { w3.R }  L7 = { w8.L }  L8 = { w9.R }  L9 = { w10.R }
DC+ = voltage source + to leg1.L1, chained leg_k.R1 -> leg_(k+1).L1, ends on leg9.R1
DC- = electrical reference = leg1.L3 = leg2.L3, chained leg_k.R3 -> leg_(k+1).L3, ends on leg9.R3
```

The graph of 9 leg midpoints + 2 star nodes + 10 winding edges has 11 nodes, 10 edges, is connected and has
no cycle: **a tree**. Note that some leg midpoints appear twice in the XML (L2 and R2 of
the same leg block are internally shorted), which is why the raw net list has 159 line elements but only 11
electrical nodes.

## 4. Control and modulation chain as delivered

```

i_ref --+--> [ i_a_ref = i_bias + 0.5*i_ref ] --+
        |                                       +--> PI_ctrl(winding a) --+
        +--> [ i_c_ref = i_bias - 0.5*i_ref ] --+                        |
                                            +--> PI_ctrl(winding c) --+ |
                                                                      | |
   10 winding voltage commands u_w1..u_w10 ---------------------------+ |
                                                                        v
                              winding_voltage_to_bridge  ->  9 leg commands ub1..ub9
                                                                        |
              minmax -> 0.5*(cmax+cmin) - 0.5  ->  duty = ub - that      v
                                                                        |
                         9 x PWM_gen (triangle 0..1, 20 kHz, Td = 1 us) v
                                                                        |
                                   9 x Bridge_module  ->  10 x RL_load
```

Important details verified in the model:

- The PI output is per-unit (+-1 = +-U_dc) and clamped; the integrator is clamped with a
  freeze-on-saturation anti-windup.
- The feedback is sampled with a Rate Transition at SW_Ts = 50 us, followed by a Memory block
  (which breaks the algebraic loop) and a second-order low-pass at fn = 6 kHz,
  zeta = 0.707.
- Gains are K_p = 2*pi*1500*L/U_dc = 0.0235619 and
  K_i = 2*pi*1500*R/U_dc = 47.1239, i.e. a 1500 Hz design bandwidth whose zero
  K_i/K_p = 2000 rad/s cancels the winding pole R/L exactly.
- The carrier is one symmetric triangle t = [0, 25us, 50us], y = [0 1 0], i.e. 20 kHz;
  the comparison is U >= carrier.
- DOF1 and DOF2 receive their reference at t = 0; DOF4/DOF5 at 2 ms and DOF3 at 4 ms through the
  ref_delay blocks (start-up staggering only).

## 5. The leg-voltage back-substitution, line by line

```

ubridge(5) = 0;                          % common-mode reference, leg 5
ubridge(4) = ubridge(5) + uw5;           % w5 runs L4 -> L5
ubridge(3) = ubridge(4) + uw4 - uw3;     % vX1 = V4 + u_w4 = V3 + u_w3
ubridge(2) = ubridge(4) + uw4 + uw2;     % w2 runs L2 -> X1
ubridge(1) = ubridge(4) + uw4 + uw1;     % w1 runs L1 -> X1
ubridge(6) = ubridge(5) - uw6;           % w6 runs L5 -> L6
ubridge(7) = ubridge(6) - uw7 + uw8;     % vX2 = V6 - u_w7 = V7 - u_w8
ubridge(8) = ubridge(6) - uw7 - uw9;     % w9 runs X2 -> L8
ubridge(9) = ubridge(6) - uw7 - uw10;    % w10 runs X2 -> L9
```

Every line is the physical node relation of one edge, with the sign set by the winding orientation: for w1
(L1 -> X1) the winding voltage is V1 - vX1, so V1 = vX1 + u_w1; for w3
(X1 -> L3) it is vX1 - V3, so V3 = vX1 - u_w3. The whole function is one
depth-first walk of the tree. It was checked against the netlist and found **correct**.

Because the map is linear, ub = M*u with M a 9x10 matrix whose rows are the expressions above.
That matrix is what the boundary study (see the companion reference) uses.

## 6. Leg currents implied by KCL

I_leg is positive when the leg pushes current into the winding network:

| Leg | Expression | Measured mean | Bias present |
|---|---|---|---|
| L1 | i_a1 | +5.495 A | yes |
| L2 | i_c1 | +4.496 A | yes |
| L3 | -i_a2 | -5.494 A | yes |
| L4 | i_a3 - i_c2 | +0.997 A | **no** |
| L5 | i_c3 - i_a3 | -0.997 A | **no** |
| L6 | i_a4 - i_c3 | +0.996 A | **no** |
| L7 | i_c4 | +4.494 A | yes |
| L8 | -i_a5 | -5.493 A | yes |
| L9 | -i_c5 | -4.495 A | yes |

Two identities follow, and both were confirmed numerically:

```

star X1 : i_a1 + i_c1 = i_a2 + i_c2          residual 5.3e-15 A
star X2 : i_a4 + i_c4 = i_a5 + i_c5          residual 7.1e-15 A
sum(I_leg, 1..9) = 0                         residual 9.8e-15 A
```

The second identity uses only the two star-node equations and does not depend on the bias value. Its physical
meaning is that the bias current circulates entirely inside the winding/leg network: the DC source supplies
only the losses.

## 7. Measured steady state (18-20 ms window)

| Quantity | Value |
|---|---|
| winding currents [A] | 5.495, 4.496, 5.494, 4.497, 5.494, 4.497, 5.494, 4.494, 5.493, 4.495 |
| control currents [A] | 0.99916, 0.99701, 0.99653, 0.99930, 0.99880 |
| pair sums [A] | 9.9905, 9.9905, 9.9911, 9.9878, 9.9878 |
| switching ripple (pp, one 50 us period) [A] | 0.313, 0.263, 0.389, 0.204, 0.331, 0.140, 0.304, 0.307, 0.330, 0.280 |
| modulation spread S | 0.2089 |
| duty range, whole run | 0.2903 .. 0.7097 |
| duty range, steady state | 0.3956 .. 0.6044 |
| copper loss sum(R*i^2) | 252.0 W |
| DC bus current (dead-time corrected) | 1.376 A |
| Tellegen residual | 1.1e-13 W |

The control-current error is at most 0.35 % of the 1 A reference, and the pair sums are held to within
0.012 A of 2*i_bias by the two star nodes.

## 8. Dead time and device drops

Reconstructing the actual leg voltages from the measured winding currents ( u_act = R*i + L*di/dt )
and comparing them with duty*U_dc gives a sign-dependent leg voltage error:

```

leg           1      2      3      4      5      6      7      8      9
I_leg [A]   +5.5   +4.5   -5.5   +1.0   -1.0   +1.0   +4.5   -5.5   -4.5
error [V]   -4.88  -4.76  +6.07  -4.32  +5.37  -4.33  -4.88  +5.93  +5.81
```

The pattern is exactly -sign(I_leg) * (2*Td/Ts*U_dc + V_device): 4 V from the two dead-time
edges plus 0.8..2.3 V of IGBT/diode forward drop. The current loop absorbs it - the integrators simply
command more voltage - so the *currents* stay accurate, but the winding voltage commands are about 2.5-3 %
of U_dc away from the ideal R*i. Consequences:

- Reading the model's own voltage commands as "the applied voltage" is misleading at this level of detail.
- When the modulation boundary is predicted quantitatively, the empirical overhead sigma0
  must be included (see the boundary reference).

A direct experiment isolates the dead time: with i_ref = 0 the modulation spread is 0.2033 with the
delivered 1 us dead time and 0.1635 with all 18 dead-time parameters set to zero - a difference of 0.0399
against the theoretical 2*Td/Ts = 0.0400. The residual 0.1635 is the device forward voltage
acting on the near-zero leg currents.

## 9. Power balance

- Copper loss sum(R*i^2) = 252.0 W (the bias dominates: 5 pairs at 5.5 A / 4.5 A).
- Estimated device conduction loss from the Vf/Ron parameters: about 44 W.
- Dead-time-corrected bus current 1.376 A = 275 W, consistent with copper plus device loss.
- The commanded volt-ampere product sum(u_wk*i_k) is 428 W; the difference from the copper loss is
  the price of the dead-time and drop compensation, not a physical loss.

## 10. Modelling-quality assessment

**Kept, correctly**: ideal bus, nine half-bridge legs with antiparallel diodes and 1 us dead time, R-L windings
with current sensors, bias-plus-differential references, per-winding per-unit PI with anti-windup, one
carrier plus min-max common-mode injection, switching-frequency feedback sampling with one sample of delay.

**Dropped, correctly**: motion/force/position loop, saturation, mutual coupling, iron and eddy loss, device
thermal networks, bus impedance, protection, quantisation, computation delay, gate-drive level design.

**Should be added before any engineering conclusion**: dead-time compensation if the applied voltage matters,
and a bus capacitor if the bus current ripple matters.

**Numerical caveat**: ideal switches plus dead time are ill-conditioned at exactly zero leg current. Two
operating points of this topology hit it - i_ref = 0 (legs 4-6 at zero) and the worst-case control
current distribution delta = [+10 -10 +10 +10 +10] (legs 2, 3, 4, 7, 9 at zero, where the solver
aborts). These are solver limitations, not topology limitations; see the boundary reference for the
near-extreme verification.
