# Exact block, parameter, and signal map

Reference model: `Half_3_Legs_with_RL_load.slx` (5 cells, 10 windings, 5 DOF).
Inventory: 1390 blocks, 177 subsystems, 15 `IGBT (Ideal, Switching)`, 15 `Diode`, one
`Voltage Source`, one `Solver Configuration`, one `Scope`.

All block names below are short names inside their parent; prefix with the model name or the
subsystem path as appropriate.

> **Naming note.** The bundled model predates the current naming standard. Block and subsystem
> names such as `tbt_3bridge_module`, `btb_3bridge_module` and `PWM_tbt_3bridge_gen` are
> historical identifiers: the `3bridge` part denotes the **three-leg cell**, not a complete
> bridge. Keep the identifiers unchanged when reading or editing the model; use "half 3-legs
> cell" in prose.

## 1. Model configuration

| Parameter | Value |
|---|---|
| Solver | `VariableStepAuto`, resolves to **ode45** (variable-step) |
| MaxStep | `1e-6` (= `SW_Ts/50`) |
| RelTol / AbsTol | `1e-4` / auto |
| StartTime / StopTime | 0 / `0.1` |
| SimulationMode | `accelerator` |
| SignalLogging | `on`, name `logsout`; only **one** signal pre-ticked (`From30`, i.e. `i_a1`) |
| SimscapeLogType | `none` |
| ZeroCrossControl | `UseLocalSettings` |
| Algebraic loop | none (compile and 100 ms run produce no warning) |

Runtime: about 16 s per 100 ms of simulated time; 350 775 steps, 35.3 % of them shorter than
1 ns, smallest 5.67e-30 s.

## 2. Base-workspace dependencies

The model needs exactly **eight** base-workspace variables, all produced by
`parameters_for_AMB_topology.m`:

| Variable | Value | Used by |
|---|---|---|
| `R_value` | 1 | `Resistor` in every topology module (10 instances) |
| `L_value` | `500e-6` | `Inductor` in every topology module (10 instances) |
| `U_dc` | 200 | `Voltage Source` (`dc_voltage`) |
| `f_c` | 1500 | only used to compute `K_p` / `K_i`; not referenced by the model |
| `K_p` | `2*pi*f_c*L_value/U_dc = 0.023562` | `PI_ctrl*/Gain4` (10 instances) |
| `K_i` | `2*pi*f_c*R_value/U_dc = 47.1239` | `PI_ctrl*/Ictrl_anti_windup/Gain1` (10 instances) |
| `SW_Ts` | `5e-5` | `Repeating Sequence` mask input, `Rate Transition` `OutPortSampleTime` |
| `half_SW_Ts` | `2.5e-5` | `Repeating Sequence` mask input |
| `bias_current` | 5 | `AMB_ref_gen*/Constant` (5 instances) |

`SW_frequency` is defined by the script but is only used to derive `SW_Ts` /
`half_SW_Ts`.

Every other name that `Simulink.findVars` reports (`fn`, `Zeta`, `Ts`, `FilterType`,
`x0`, `xdot0`, `const`, `period`, `rep_seq_t`, `rep_seq_y`,
`antiwindupUpperLimit`, `antiwindupLowerLimit`, `windupUpperLimit`,
`windupLowerLimit`, `UseLocalSolver`, `LocalSolverSampleTime`) comes from the **mask
workspace** of `Second-Order Filter`, `Repeating Sequence`, `Solver Configuration` and
`Compare To Constant` blocks. They must not be defined in the base workspace.

> `parameters_for_AMB_topology.m` starts with `clearvars;`. Run it **before** touching any
> other variable, or your variables will be wiped.

## 3. Root-level structure: five identical channels

| Ch | Topology module | Type | Current tags | PWM generator | Gate generator | Current controllers | Reference generator |
|---|---|---|---|---|---|---|---|
| 1 | `tbt_3bridge_module` | TBT | `i_a1` / `i_c1` | `PWM_tbt_3bridge_gen` | `SW_tbt_3bridge_gen` | `PI_ctrl`, `PI_ctrl1` | `AMB_ref_gen` |
| 2 | `btb_3bridge_module` | BTB | `i_a2` / `i_c2` | `PWM_btb_3bridge_gen` | `SW_btb_3bridge_gen` | `PI_ctrl2`, `PI_ctrl3` | `AMB_ref_gen1` |
| 3 | `tbt_3bridge_module1` | TBT | `i_a3` / `i_c3` | `PWM_tbt_3bridge_gen1` | `SW_tbt_3bridge_gen1` | `PI_ctrl4`, `PI_ctrl5` | `AMB_ref_gen2` |
| 4 | `btb_3bridge_module1` | BTB | `i_a4` / `i_c4` | `PWM_btb_3bridge_gen1` | `SW_btb_3bridge_gen1` | `PI_ctrl6`, `PI_ctrl7` | `AMB_ref_gen3` |
| 5 | `tbt_3bridge_module2` | TBT | `i_a5` / `i_c5` | `PWM_tbt_3bridge_gen2` | `SW_tbt_3bridge_gen2` | `PI_ctrl8`, `PI_ctrl9` | `AMB_ref_gen4` |

Signal chain per channel:
`i_ref -> AMB_ref_gen -> (ia_ref, ic_ref) -> PI_ctrl / PI_ctrl' -> (u_a, u_c) ->
PWM_*_3bridge_gen -> (b1, b2, b3) -> SW_*_3bridge_gen -> (sw1, sw2, sw3) ->
*_3bridge_module -> (i_a, i_c) -> back to the controllers, and to the scope`

Electrical skeleton:

- one `Voltage Source` (200 V) plus one `Electrical Reference` on the bus negative rail;
- the five modules are daisy-chained on the **same** bus through their Simscape connection ports
  `Vcci/Vddi` (in) and `Vcco/Vddo` (out);
- the far end of the chain is terminated by two `Open Circuit` blocks;
- one `Solver Configuration` (`UseLocalSolver = off`, `LocalSolverSampleTime = 1e-6`);
- one `Scope` named `winding_currents` with 10 inputs (i_a1, i_c1, ... i_a5, i_c5).

Control gain: all five `i_ref` are constants equal to 1 A
(`Constant6`, `Constant1`, `Constant2`, `Constant3`, `Constant4`).

## 4. `tbt_3bridge_module` - TBT cell

Ports: in `sw1`, `sw2`, `sw3`; out `i_a`, `i_c`; connection ports `Vcci`, `Vddi`
(left) and `Vcco`, `Vddo` (right).

Netlist (nodes P = bus +, N = bus -, M1 = leg 1 midpoint, A1 = leg 2 midpoint,
C1 = leg 3 midpoint):

| Block | Reference block | Terminals | Net |
|---|---|---|---|
| `IGBT (Ideal, Switching)` | `ee_lib/Semiconductors & Converters` | G <- `Simulink-PS Converter` <- `sw1` | gate |
| | | C - E | P - M1 |
| `Diode1` | `fl_lib/Electrical/Electrical Elements/Diode` | p - n | N - M1 |
| `IGBT (Ideal, Switching)1` | | G <- `sw2` | gate |
| | | C - E | A1 - N |
| `Diode` | | p - n | A1 - P |
| `IGBT (Ideal, Switching)2` | | G <- `sw3` | gate |
| | | C - E | P - C1 |
| `Diode2` | | p - n | N - C1 |
| `Resistor` + `Inductor` + `Current Sensor` | | series, p to n | M1 -> A1 (winding a) |
| `Resistor1` + `Inductor1` + `Current Sensor1` | | series, p to n | A1 -> C1 (winding c) |

Sensor orientation: `Current Sensor` measures p -> n = M1 -> A1, `Current Sensor1` measures
p -> n = C1 -> A1. Both are therefore "T leg to B leg" positive, i.e. both flow **into the
shared node A1**.

Device parameters (base Simscape defaults, not driven by the parameter script):
`Diode` `Ron = 0.3`, `Vf = 0.6`, `Goff = 1e-8`;
`IGBT (Ideal, Switching)` `Ron = 0.001`, `Vf = 0.8`, `Goff = 1e-5`.

## 5. `btb_3bridge_module` - BTB cell

Ports identical to the TBT cell.

| Block | Terminals | Net |
|---|---|---|
| `IGBT (Ideal, Switching)1` | G <- `sw2`; C - E | P - M1 |
| `Diode1` | p - n | N - M1 |
| `IGBT (Ideal, Switching)3` | G <- `sw1`; C - E | A1 - N |
| `Diode3` | p - n | A1 - P |
| `IGBT (Ideal, Switching)2` | G <- `sw3`; C - E | C1 - N |
| `Diode4` | p - n | C1 - P |
| `Resistor` + `Inductor` + `Current Sensor` | series | A1 -> M1 (winding a) |
| `Resistor1` + `Inductor1` + `Current Sensor1` | series | M1 -> C1 (winding c) |

Sensor orientation: `Current Sensor` measures p -> n with p at M1, i.e. M1 -> A1;
`Current Sensor1` measures p -> n = M1 -> C1. Again "T leg to B leg" positive, but both flow
**out of the shared node M1**.

Note the index swap relative to TBT: `sw1` drives the **lower** device of leg 2 (B), `sw2`
drives the **upper** device of leg 1 (T), `sw3` drives the **lower** device of leg 3 (B).

The cell is drawn as T-B-B. It is the conventional B-T-B seen from a different leg ordering:
relabel leg 1 <-> leg 2 and the chain becomes B-T-B. The circuit is correct as built.

## 6. `AMB_ref_gen` - differential reference generation

Ports: in `i_ref`; out `ia_ref`, `ic_ref`.

| Block | Type | Setting | Connections |
|---|---|---|---|
| `Gain` | Gain | `0.5` | `i_ref -> u1` |
| `Constant` | Constant | `bias_current` | feeds both sums |
| `Add` | Sum | Inputs `'++'` | `Constant.y1 -> u1`, `Gain.y1 -> u2`, out -> `ia_ref` |
| `Add1` | Sum | Inputs `'-+'` | `Gain.y1 -> u1`, `Constant.y1 -> u2`, out -> `ic_ref` |

## 7. `PI_ctrl` - per-winding current controller

Ports: in `ref`, `fbk`; out `U_cmd`. Not masked.

| Block | Type | Setting |
|---|---|---|
| `Add2` | Sum `'+-'` | `ref - fbk_filtered` |
| `Gain4` | Gain | `K_p` |
| `Ictrl_anti_windup` | SubSystem | see section 8 |
| `Add3` | Sum `'++'` | `K_p*err + Int_out` |
| `Saturation` | Saturate | `LowerLimit = -1`, `UpperLimit = +1` |
| `Rate Transition` | RateTransition | `Integrity = on`, `Deterministic = on`, `InitialCondition = 0`, `OutPortSampleTimeOpt = 'Specify'`, `OutPortSampleTime = SW_Ts` |
| `Memory` | Memory | `InitialCondition = 0`, `InheritSampleTime = off` |
| `Second-Order Filter` | masked SubSystem | `FilterType = Low-pass`; mask value string `Low-pass` | `6000` | `0.707` | `0` | `0` | `0` | `0` | `0` |

Signal flow: `ref -> Add2.u1`; `fbk -> Rate Transition -> Memory -> Second-Order Filter ->
`Add2.u2`; `Add2 -> Gain4 -> Add3.u1`; `Add2 -> Ictrl_anti_windup -> Add3.u2`;
`Add3 -> Saturation -> U_cmd`.

The `Rate Transition` has direct feedthrough, so it is the `Memory` block that breaks the
algebraic loop; removing the `Memory` raises exactly one algebraic-loop diagnostic.

## 8. `PI_ctrl/Ictrl_anti_windup` - conditional-integration anti-windup

Ports: in `err`; out `Int_out`. Not masked.

| Block | Type | Setting |
|---|---|---|
| `AND` | Logic | **`Operator = OR`** (despite the name), 2 inputs |
| `AND1` | Logic | `Operator = AND`, 2 inputs |
| `Compare To Constant` | CompareToConstant | `relop = '<='`, `const = -1` |
| `Compare To Constant1` | CompareToConstant | `relop = '>='`, `const = 1` |
| `Compare To Constant2` | CompareToConstant | `relop = '>='`, `const = 0` |
| `Product` | Product | `Integrator.y1 * err` |
| `Constant` | Constant | `0` |
| `Switch` | Switch | `Criteria = 'u2 ~= 0'`, `Threshold = 0` |
| `Gain1` | Gain | `K_i` |
| `Integrator` | Integrator | `InitialCondition = 0`, `LimitOutput = on`, `UpperSaturationLimit = 1`, `LowerSaturationLimit = -1` |

Signal flow: `AND` takes (`x <= -1`) and (`x >= 1`) - an OR - giving `abs(x) >= 1`;
`AND1` combines that with (`x*err >= 0`); the `Switch` outputs `0` when the condition is
true and `err` otherwise; `Gain1` scales by `K_i` into the integrator. The integrator state
therefore freezes when it is at a limit and the error would push it further, exactly the
intended conditional integration.

## 9. `PWM_tbt_3bridge_gen` - TBT modulator

Ports: in `u_a`, `u_c`; out `b1`, `b2`, `b3`.

| Block | Type | Setting | Role |
|---|---|---|---|
| `Add2` | Sum `'-+'` | | `e2 = -u_c` |
| `Add3` | Sum `'++'` | | `e1 = u_a - u_c` |
| `Max` / `Min` | MinMax, 3 inputs | max / min | over `(e1, e2, 0)` |
| `Add` | Sum | | `max + min` |
| `Gain` | Gain | `0.5` | `m` |
| `Constant` | Constant | `0.5` | offset |
| `Constant7` | Constant | `0` | third input |
| `Add1` | Sum `'+-'` | | `m - 0.5` |
| `Add4` | Sum `'-+'` | | threshold for `b1` |
| `Add5` | Sum `'-+'` | | threshold for `b2` |
| `Add6` | Sum `'-+'` | | threshold for `b3` |
| `Relational Operator1` | RelationalOperator | `Operator = '>='` | `Add4 >= carrier -> b1` |
| `Relational Operator` | RelationalOperator | `Operator = '>='` | `Add5 >= carrier -> b2` |
| `Relational Operator2` | RelationalOperator | `Operator = '>='` | `Add6 >= carrier -> b3` |
| `Repeating Sequence` | masked SubSystem | `rep_seq_t = [0 half_SW_Ts SW_Ts]`, `rep_seq_y = [0 1 0]` | symmetric triangle, 0..1 |

Thresholds: `b1 <- 0.5 - m + e1`, `b2 <- 0.5 - m + e2`, `b3 <- 0.5 - m`.

## 10. `PWM_btb_3bridge_gen` - BTB modulator

Ports: in `u_a`, `u_c`; out `b1`, `b2`, `b3`.

Same block inventory as the TBT modulator, but:

- `Add2` computes `e2' = u_c` (not `-u_c`);
- `Add3` computes `e1' = u_c - u_a`;
- the min/max operate on `(e1', e2', 0)`, giving `m' = -m`;
- `Relational Operator` (threshold `b2`) drives outport `b2`;
- `Relational Operator1` (threshold `b1`) drives outport `b1`;
- `Relational Operator2` (threshold `b3`) drives outport `b3`.

Because `m' = -m`, the resulting duty triple equals the TBT one exactly.

## 11. Gate generators

| Block | `sw1` | `sw2` | `sw3` |
|---|---|---|---|
| `SW_tbt_3bridge_gen` | `15 * b1` | `15 * NOT(b2)` | `15 * b3` |
| `SW_btb_3bridge_gen` | `15 * NOT(b1)` | `15 * b2` | `15 * NOT(b3)` |

Each gate generator contains three `Gain` blocks with `Gain = 15` and (only in the TBT
version) one `Logic` block configured as NOT.

## 12. Goto/From tag map

| Goto tag | Produced by | Consumed by |
|---|---|---|
| `sw1_m1` / `sw2_m1` / `sw3_m1` | `SW_tbt_3bridge_gen` | `From42` / `From1` / `From2` -> `tbt_3bridge_module` |
| `sw1_m2` / `sw2_m2` / `sw3_m2` | `SW_btb_3bridge_gen` | `From5` / `From3` / `From4` -> `btb_3bridge_module` |
| `sw1_m3` / `sw2_m3` / `sw3_m3` | `SW_tbt_3bridge_gen1` | `From10` / `From6` / `From7` -> `tbt_3bridge_module1` |
| `sw1_m4` / `sw2_m4` / `sw3_m4` | `SW_btb_3bridge_gen1` | `From11` / `From8` / `From9` -> `btb_3bridge_module1` |
| `sw1_m5` / `sw2_m5` / `sw3_m5` | `SW_tbt_3bridge_gen2` | `From12` / `From13` / `From14` -> `tbt_3bridge_module2` |
| `i_a1` / `i_c1` | `tbt_3bridge_module` | `From40` -> `PI_ctrl.fbk`, `From30` -> scope |
| `i_a2` / `i_c2` | `btb_3bridge_module` | `From15` -> `PI_ctrl2.fbk`, `From32` -> scope |
| `i_a3` / `i_c3` | `tbt_3bridge_module1` | `From17` -> `PI_ctrl4.fbk`, `From34` -> scope |
| `i_a4` / `i_c4` | `btb_3bridge_module1` | `From19` -> `PI_ctrl6.fbk`, `From36` -> scope |
| `i_a5` / `i_c5` | `tbt_3bridge_module2` | `From21` -> `PI_ctrl8.fbk`, `From38` -> scope |

(The i_c tags also feed the second controller of each pair through `From41`, `From16`,
`From18`, `From20`, `From22`, and the scope through `From31`, `From33`,
`From35`, `From37`, `From39`.)

## 13. Reproduction notes

The model and the parameter script ship with this skill in `model/`, so the snippet below
runs with nothing else on the MATLAB path. Built and verified with **MATLAB R2025b (25.2),
Simulink 25.2, Simscape 25.2** on Linux (GLNXA64); the `.slx` was saved by that release.

```matlab
cd('<this skill folder>/model')                 % bundled .slx + .m live here
run('parameters_for_AMB_topology.m');           % must run first: it starts with clearvars
m = 'Half_3_Legs_with_RL_load';
load_system(m);
out = sim(m);                                   % 0.1 s, about 16 s wall clock
```

To log the winding currents for analysis without editing the saved model, mark the **source port**
of each signal (a port handle must be an output port for `DataLogging` to be settable):

```matlab
ph = get_param('Half_3_Legs_with_RL_load/Goto30','PortHandles');
sp = get_param(get_param(ph.Inport(1),'Line'),'SrcPortHandle');
set_param(sp,'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','z_i_a1');
```

Reduce with time-weighted averages - the solver is variable-step:

```matlab
idx = find(T >= 0.03);  dt = diff(T(idx));
avg = @(x) sum(x(2:end).*dt)/sum(dt);
```

Resample onto a uniform grid before any FFT of the ripple.
