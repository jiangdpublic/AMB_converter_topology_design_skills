# Block, port and parameter map

Every path, parameter name and netlist edge that scripts/verify_model_facts.m asserts against the
bundled models. Read it together with case-ten-phase-nine-leg-amb.md.

Models: model/Ten_phase_nine_leg_with_RL_load.slx (md5 0e5bbb66a6f61da9e0210f63aa7ccffd),
model/Ten_phase_nine_leg_relocated_L3_L7.slx (md5 1f051832b3c2760930ffcd561aadd9c4),
model/Ten_phase_nine_leg_relocated_L2_L8.slx (md5 36e3503b18cf1c1abf8b819d801ccf7f),
model/parameters_for_AMB_topology.m (md5 a90c6e390a9c1e14bf015396b2d6054d).

> **Naming note.** `Bridge_module` and the MATLAB variables/functions `ubridge` and
> `winding_voltage_to_bridge` are historical model identifiers. They denote one **half-bridge
> leg** and its per-leg voltage commands, not complete H-bridges - a complete H-bridge has two
> legs.

Environment: MATLAB R2025b (25.2.0.3312555 Update 6), Simulink 25.2, Simscape 25.2, Simscape Electrical
25.2, Linux glnxa64.

---

## 1. Root-level inventory

146 blocks in the root system (all three models):

| BlockType | Count |
|---|---|
| SubSystem | 49 |
| From | 38 |
| Goto | 28 |
| SimscapeBlock | 14 |
| Constant | 7 |
| Step | 3 |
| Sum | 3 |
| Demux | 1 |
| Gain | 1 |
| Product | 1 |
| Scope | 1 |

Named blocks, by group:

| Group | Names |
|---|---|
| Reference generators | AMB_ref_gen, AMB_ref_gen1 ... AMB_ref_gen4 |
| Half-bridge legs | Bridge_module, Bridge_module1 ... Bridge_module8 (legs L1..L9) |
| Windings | RL_load, RL_load1 ... RL_load9 (w1..w10) |
| Current controllers | PI_ctrl, PI_ctrl1 ... PI_ctrl9 (w1..w10) |
| Modulators | PWM_gen, PWM_gen1 ... PWM_gen8 (legs L1..L9) |
| Reference delays | ref_delay, ref_delay1, ref_delay2 |
| Modulator glue | MATLAB Function (min-max), winding_voltage_to_bridge (back-substitution), Add1, Add2, Add3, Gain1, Product, Demux, Constant5, Constant7 |
| Reference constants | Constant6 (DOF1), Constant1 (DOF2), Constant2 (DOF3), Constant3 (DOF4), Constant4 (DOF5) |
| Start-up steps | Step (DOF5), Step1 (DOF4), Step2 (DOF3) |
| Plant | Voltage Source, Electrical Reference, Solver Configuration |
| Terminators | Open Circuit, Open Circuit1 ... Open Circuit11 (inert) |
| Logging in the delivered model | winding_currents (Scope, 10 inputs) |

## 2. Parameters (name = value)

Base-workspace variables produced by model/parameters_for_AMB_topology.m:

| Variable | Value | Note |
|---|---|---|
| R_value | 1 | winding resistance [ohm] |
| L_value | 500e-6 | winding inductance [H] |
| U_dc | 200 | DC bus [V] |
| f_c | 1500 | current-loop design bandwidth [Hz] |
| K_p | 2*pi*f_c*L_value/U_dc | = 0.0235619 |
| K_i | 2*pi*f_c*R_value/U_dc | = 47.1239 |
| SW_frequency | 20e3 | switching frequency [Hz] |
| SW_Ts | 1/SW_frequency | 50e-6 |
| half_SW_Ts | SW_Ts*0.5 | 25e-6 |
| bias_current | 5 | bias current [A] |

The script begins with clearvars, so it must be run before touching anything else.

Block parameters asserted by the verification script:

| Block | Parameter | Value |
|---|---|---|
| Voltage Source | dc_voltage | U_dc |
| PWM_gen/Repeating Sequence | rep_seq_t / rep_seq_y | [0 half_SW_Ts SW_Ts] / [0 1 0] |
| PWM_gen/On-Off Delay and On-Off Delay1 (all 9 PWM_gens) | Td_on / Td_off | 1e-6 / 0 |
| Bridge_module/IGBT (Ideal, Switching)1 | Vf / Ron / Vth | 0.8 / 0.001 / 6 |
| Bridge_module/Diode1 | Vf / Ron / Goff | 0.6 / 0.3 / 1e-8 |
| PI_ctrl/Saturation | UpperLimit / LowerLimit | 1 / -1 |
| PI_ctrl/Second-Order Filter | fn / zeta | 6000 / 0.707 |
| PI_ctrl/Rate Transition | OutPortSampleTime | SW_Ts |
| PI_ctrl/Gain4 | Gain | K_p |
| AMB_ref_gen/Gain / Constant | Gain / Value | 0.5 / bias_current |
| PWM_gen/Gain, Gain1 | Gain | 15 (gate level) |
| PWM_gen/Relational Operator2 | Operator | >= |
| Gain1 | Gain | 0.5 (min-max injection) |
| Constant5 | Value | 0.5 |
| Constant6 | Value | 1 (DOF1 i_ref) |
| Constant7 | Value | [1 1 1 1 1 1 1 1 1] |
| Demux / Product | Outputs / Inputs | 9 / 2 |

Model configuration:

| Setting | Value |
|---|---|
| Solver | VariableStepAuto |
| StopTime | 0.05 |
| MaxStep / RelTol | 1e-6 / 1e-4 |
| SimulationMode | accelerator |
| ZeroCrossControl / IgnoredZcDiagnostic | UseLocalSettings / warning |
| Solver Configuration | UseLocalSolver = off, ResidualTolerance = 1e-9 |

## 3. Half-bridge leg ports

Each Bridge_module has six electrical connection ports, shorted pairwise inside:

| Port (both sides) | Label | Internal node |
|---|---|---|
| LConn1 / RConn1 | Vcci / Vcco | DC+ (upper IGBT collector, upper diode cathode) |
| LConn2 / RConn2 | Vsi / Vso | leg midpoint (upper emitter = lower collector, upper diode anode, lower diode cathode) |
| LConn3 / RConn3 | Vddi / Vddo | DC- (lower emitter, lower diode anode) |

Mirrored instances (legs 2, 4, 5, 6, 8, 9 in the drawing) simply place the same ports on the other side;
the electrical meaning is unchanged. That is why the net list sometimes refers to a LConn2 and
sometimes to an RConn2 for the same electrical node.

RL_load has one connection port per side: L (LConn1) and R (RConn1).

## 4. Netlist (root system, from the SLX XML)

Port references use the XML form Block#lconn:n / Block#rconn:n, numbered per side
(1 = DC+, 2 = midpoint, 3 = DC- for a leg).

```

w1.L  RL_load#lconn:1  <-> Bridge_module#rconn:2
w1.R  RL_load#rconn:1  <-> X1 { RL_load1#rconn:1, RL_load2#lconn:1, RL_load3#lconn:1 }
w2.L  RL_load1#lconn:1 <-> Bridge_module1#rconn:2
w3.R  RL_load2#rconn:1 <-> Bridge_module2#lconn:2
w4.R  RL_load3#rconn:1 <-> Bridge_module3#lconn:2
w4.R  (same net also holds Bridge_module3#lconn:2 only)
w5.L  RL_load4#lconn:1 <-> Bridge_module3#rconn:2
w5.R  RL_load4#rconn:1 <-> Bridge_module4#lconn:2
w6.L  RL_load5#lconn:1 <-> Bridge_module4#rconn:2
w6.R  RL_load5#rconn:1 <-> Bridge_module5#lconn:2
w7.L  RL_load6#lconn:1 <-> Bridge_module5#rconn:2
w7.R  RL_load6#rconn:1 <-> X2 { RL_load7#rconn:1, RL_load8#lconn:1, RL_load9#lconn:1 }
w8.L  RL_load7#lconn:1 <-> Bridge_module6#rconn:2
w9.R  RL_load8#rconn:1 <-> Bridge_module7#lconn:2
w10.R RL_load9#rconn:1 <-> Bridge_module8#lconn:2
DC+ : Voltage Source#lconn:1 = Bridge_module#lconn:1 = Bridge_module1#lconn:1
      Bridge_module#rconn:1 = Bridge_module2#lconn:1   (and so on to Bridge_module8#rconn:1 -> Open Circuit2)
DC- : Electrical Reference#lconn:1 = Bridge_module#lconn:3 = Bridge_module1#lconn:3
      = Voltage Source#rconn:1 = Solver Configuration#rconn:1
      Bridge_module#rconn:3 = Bridge_module2#lconn:3   (and so on to Bridge_module8#rconn:3 -> Open Circuit1)
inert terminators: Open Circuit3 on Bridge_module#lconn:2, Open Circuit11 on Bridge_module1#lconn:2,
      Open Circuit10 on Bridge_module2#rconn:2, Open Circuit 9/8 on Bridge_module3#rconn:1 / :3,
      Open Circuit7 on Bridge_module6#lconn:2, Open Circuit 6/5/4 on Bridge_module7#rconn:1 / :2 / :3,
      Open Circuit 2/1 on Bridge_module8#rconn:1 / :3, Open Circuit on Bridge_module8#rconn:2
```

The root system has 159 line elements; the three models differ only in the two nets that carry w5.L and
w6.R (see case-ten-phase-nine-leg-relocating-the-extra-leg.md).

## 5. MATLAB Function sources

winding_voltage_to_bridge (the topology-specific back-substitution, verified character by
character):

```

function ubridge = u_w2b(uw1, uw2, uw3, uw4, uw5, uw6, uw7, uw8, uw9, uw10)
ubias = 0;
ubridge = [0 0 0 0 0 0 0 0 0];
ubridge(5) = ubias;
ubridge(4) = ubridge(5) + uw5;
ubridge(3) = ubridge(4) + uw4 - uw3;
ubridge(2) = ubridge(4) + uw4 + uw2;
ubridge(1) = ubridge(4) + uw4 + uw1;
ubridge(6) = ubridge(5) - uw6;
ubridge(7) = ubridge(6) - uw7 + uw8;
ubridge(8) = ubridge(6) - uw7 - uw9;
ubridge(9) = ubridge(6) - uw7 - uw10;
end
```

MATLAB Function (the min-max helper; its only non-English text is the three comment lines,
kept because the engineer model is bundled byte-identical):

```

function [cmin, cmax] = minmax(u)
c_sorted = sort(u);
cmin = c_sorted(1);
cmax = c_sorted(end);
end
```

## 6. Signal routing

Goto tags at root (28): i_a1..i_a5, i_c1..i_c5 (winding current feedback, one per
controller) and pwmh1..pwmh9, pwml1..pwml9 (the two gate signals of each leg).

Wiring of the controller chain:

| Signal | Source | Destination |
|---|---|---|
| i_ref | Constant6/1/2/3/4 | AMB_ref_gen..4#in:1 |
| ia_ref / ic_ref | AMB_ref_gen out 1 / 2 | PI_ctrl (a) / PI_ctrl1 (c) of the same DOF |
| i_a1..i_c5 | RL_load current sensor | PI_ctrl#in:2 and the winding_currents scope |
| u_w1..u_w10 | PI_ctrl..PI_ctrl9#out:1 | winding_voltage_to_bridge#in:1..10 |
| ub1..ub9 | winding_voltage_to_bridge#out:1 | MATLAB Function#in:1 and Add3#in:1 |
| common mode | minmax -> Add1 -> Gain1(0.5) -> Add2(-0.5) -> Product(x ones(1,9)) | Add3#in:2 |
| duty | Add3 -> Demux(9) | PWM_gen..PWM_gen8#in:1 |
| gates | PWM_gen..8 out 1 / 2 | pwmh1..9 / pwml1..9 -> Bridge_module..8#in:1/2 |

## 7. Relocation variants

Only two terminal nets and the winding_voltage_to_bridge source differ:

| | variant A (L3_L7) | variant B (L2_L8) |
|---|---|---|
| RL_load4#lconn:1 (w5.L) | Bridge_module2#rconn:2 (+ Open Circuit10) | Bridge_module1#rconn:2 (+ RL_load1#lconn:1) |
| RL_load5#rconn:1 (w6.R) | Bridge_module6#rconn:2 (+ RL_load7#lconn:1) | Bridge_module7#lconn:2 (+ RL_load8#rconn:1) |
| new back-substitution | see the relocation reference | see the relocation reference |
| block count | 146 | 146 |
| logging blocks | none | none |

The variants were regenerated from the pristine engineer model in a scratch folder, so they carry no logging
blocks and their .slx files are the bundled artefacts.
