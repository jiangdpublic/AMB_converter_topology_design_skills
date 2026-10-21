# Exact block, parameter, and signal map

Reference model: `Reversed_shared_Bridge_with_RL_load.slx` (10 windings / 5 pairs / 5 T modules + 5 B modules
+ 1 shared leg, a bidirectional half-bridge), bundled with the skill at `model/Reversed_shared_Bridge_with_RL_load.slx`
(md5 `80211c88ec9d55c4f5af97db1d07eff0`), with its parameters at
`model/parameters_for_AMB_topology.m` (md5 `a90c6e390a9c1e14bf015396b2d6054d`).

Verified with **MATLAB 25.2 (R2025b), Simulink 25.2, Simscape 25.2, Simscape Electrical 25.2
(Linux `glnxa64`)**; the model's stored `Version` property is 25.2. Every name, path, parameter and
value below was read from the model with `get_param`/`find_system`; nothing here is paraphrased.
Re-check it at any time with `scripts/verify_model_facts.m`, which asserts every entry of this file
against the bundled model.

**Naming note.** Several blocks were created by pasting a library block whose display name wraps, so
their **stored** names contain embedded newline characters - for example the stored name of each module
switch is `IGBT\n(Ideal,\nSwitching)5`. Simulink resolves a block path when those newlines are written
as spaces (e.g. `Bridge_normal_module/IGBT (Ideal, Switching)5`), which is the form used below.

---

## 1. Model configuration

| Parameter (root model) | Value |
|---|---|
| `Version` | `25.2` (saved by R2025b) |
| `Solver` / `SolverType` / `SolverName` | `VariableStepAuto` / `Variable-step` / `VariableStepAuto` |
| `StopTime` | `0.1` |
| `StartTime` | `0.0` |
| `MaxStep` | `1e-6` |
| `MinStep` / `InitialStep` / `AbsTol` | `auto` |
| `RelTol` | `1e-4` |
| `SimulationMode` | `accelerator` |
| `ZeroCrossAlgorithm` | `Nonadaptive` |
| `MaxConsecutiveZCs` | `1000` |
| `SignalLogging` / `SignalLoggingName` | `on` / `logsout` |
| `SaveOutput` / `SaveFormat` | `on` / `Dataset` |
| `SimscapeLogType` | `none` (the case-study script switches it to `all` for its short device-level run) |
| `SimscapeLogLimitData` | `on` - only the **last 5000 solver steps** of each physical variable are kept |

`Solver Configuration` block (library `nesl_utility/Solver Configuration`):

| Parameter | Value |
|---|---|
| `UseLocalSolver` | `off` |
| `LocalSolverChoice` / `LocalSolverSampleTime` | `NE_TRAPEZOIDAL_ADVANCER` / `1e-6` (unused while `UseLocalSolver = off`) |
| `IndexReductionMethod` | `DerivativeReplacement` |
| `ResidualTolerance` | `1e-9` |
| `DoFixedCost` / `DoDC` / `ComputeImpulses` | `off` |
| `MaxNonlinIter` / `MaxModeIter` | `3` / `2` |
| `AutomaticFiltering` / `FilteringTimeConstant` | `on` / `0.001` |
| `SimulateNoise` | `off` |

---

## 2. Root-level blocks

| Block name | BlockType | `ReferenceBlock` (library path) | Key parameters |
|---|---|---|---|
| `Voltage Source` | SimscapeBlock | `ee_lib/Sources/Voltage Source` | `dc_voltage = U_dc`, `sample_time = 1e-3` |
| `Solver Configuration` | SubSystem (masked) | `nesl_utility/Solver Configuration` | see section 1 |
| `Electrical Reference` | SimscapeBlock | `fl_lib/Electrical/Electrical Elements/Electrical Reference` | - |
| `Open Circuit`, `Open Circuit1`, `Open Circuit2` | SimscapeBlock | `fl_lib/Electrical/Electrical Elements/Open Circuit` | terminate the far end of the `Vcc` / `Shared` / `Vdd` chains |
| `IGBT (Ideal, Switching)3` | SimscapeBlock | `ee_lib/Semiconductors & Converters/IGBT (Ideal, Switching)` | shared leg, **upper** switch |
| `IGBT (Ideal, Switching)4` | SimscapeBlock | idem | shared leg, **lower** switch |
| `Diode3`, `Diode4` | SimscapeBlock | `fl_lib/Electrical/Electrical Elements/Diode` | shared-leg freewheel diodes |
| `Current Sensor`, `Current Sensor1` | SimscapeBlock | `fl_lib/Electrical/Electrical Sensors/Current Sensor` | shared-leg current measurement (see section 5) |
| `On-Off Delay`, `On-Off Delay1` | SubSystem (masked) | `eeGeneralControl/On-Off Delay` | dead-time blocks (section 5) |
| `Bridge_normal_module` ... `Bridge_normal_module4` | SubSystem | - | 5 T modules (section 3) |
| `Bridge_reversed_module` ... `Bridge_reversed_module4` | SubSystem | - | 5 B modules (section 4) |
| `PI_ctrl1` ... `PI_ctrl10` | SubSystem | - | 10 identical current controllers (section 7) |
| `PWM_gen`, `PWM_gen1` ... `PWM_gen10` | SubSystem | - | 11 identical modulators (section 8) |
| `AMB_ref_gen` ... `AMB_ref_gen4` | SubSystem | - | 5 differential reference generators (section 9) |
| `Mux` / `Demux` | Mux / Demux | - | `Inputs = 10` / `Outputs = 10` |
| `MATLAB Function` | SubSystem (EM chart) | - | sort-based min/max (section 10) |
| `Add1` `Add2` `Add3` `Add4`, `Gain1`, `Product` | Sum / Gain / Product | - | modulation chain (section 10) |
| `Constant1` ... `Constant4`, `Constant6` | Constant | - | `Value = 1` - the five control-current references `i_ref` |
| `Constant5` / `Constant7` / `Constant8` | Constant | - | `0.5` / `[1 1 1 1 1 1 1 1 1 1]` / `0` |
| `Gain`, `Gain2` ... `Gain12` | Gain | - | gate amplifiers, `Gain = 15` |
| `Gain13` ... `Gain17` | Gain | - | `Gain = -1` - command negation for the five B-module windings |
| `NOT` ... `NOT5` | Logic | - | `Operator = NOT` |
| `From`/`Goto` pairs | From / Goto | - | `sw0`...`sw10`, `i_a1`...`i_c5`, `i_shared_h`, `i_shared_l` (section 11) |
| `winding_currents` | Scope | - | `NumInputPorts = 10` |
| `shared_bridge_current` | Scope | - | `NumInputPorts = 2` |

There are no root Outports and no `To Workspace` blocks: to record signals, mark them for logging (see
`scripts/run_case_study.m`).

---

## 3. `Bridge_normal_module` - one per T-type winding

### 3.1 Interface

| Port | Direction | Name in the model |
|---|---|---|
| 1 | input (Simulink) | `sw` - gate command, fed through a Simulink-PS Converter |
| 2 | output (Simulink) | `i_fbk` - winding current, from a PS-Simulink Converter |
| 3 / 4 / 5 | physical, left | `Vcci` (LConn1) / `Shared_i` (LConn2) / `Vddi` (LConn3) |
| 6 / 7 / 8 | physical, right | `Vcco` (RConn1) / `Shared_o` (RConn2) / `Vddo` (RConn3) |

### 3.2 Blocks

| Block | Type / library | Parameters |
|---|---|---|
| `IGBT (Ideal, Switching)5` | `ee_lib/.../IGBT (Ideal, Switching)` | `Vf = 0.8`, `Ron = 0.001`, `Goff = 1e-5`, `Vth = 6`, `port_option = ee.enum.controlport.ps`, `diode_param = ee.enum.semiconductors.protectionDiode.external` |
| `Diode5` | `fl_lib/.../Diode` | `Vf = 0.6`, `Ron = 0.3`, `Goff = 1e-8` |
| `Resistor2` | `fl_lib/.../Resistor` | `R = R_value` (= 1 ohm) |
| `Inductor2` | `fl_lib/.../Inductor` | `l = L_value` (= 500e-6 H), `r = 0`, `g = 1e-9` |
| `Current Sensor2` | `fl_lib/.../Current Sensor` | ideal, output `I` |
| `Simulink-PS Converter` / `PS-Simulink Converter2` | `nesl_utility` | - |

### 3.3 Netlist (node names as in section 1.1 of the skill)

| Device terminal | Node |
|---|---|
| IGBT `C` | `Vcci` = `Vcco` = bus **P** |
| IGBT `E` | node **X** = diode cathode = current-sensor `p` |
| IGBT `G` | `sw` through the Simulink-PS Converter |
| Diode `p` (anode) | `Vddi` = `Vddo` = bus **N** |
| Diode `n` (cathode) | node **X** |
| Current sensor `p` -> `n` | **X** -> `Inductor2.n`; measured current is positive flowing X -> winding |
| Inductor `p` / `n` | `Resistor2.n` / sensor `n` |
| Resistor `p` / `n` | `Shared_i` = `Shared_o` / `Inductor2.p` |

So the branch is `X -> sensor -> L -> R -> Shared`: **positive current flows into the shared node**,
and the leg is "upper switch + lower freewheel diode" (a **T leg**), which cannot short the bus.

---

## 4. `Bridge_reversed_module` - one per B-type winding

Interface: identical to section 3.1 (`sw`, `i_fbk`, `Vcci`/`Shared_i`/`Vddi` left,
`Vcco`/`Shared_o`/`Vddo` right). Blocks: the same set and the same parameters as section 3.2
(`Current Sensor2`, `Diode5`, `IGBT (Ideal, Switching)5`, `Inductor2`, `Resistor2`,
`Simulink-PS Converter`, `PS-Simulink Converter2`).

Netlist:

| Device terminal | Node |
|---|---|
| Diode `p` (anode) | node **Y** = IGBT `C` = current-sensor `n` |
| Diode `n` (cathode) | `Vcci` = `Vcco` = bus **P** |
| IGBT `C` / `E` | node **Y** / `Vddi` = `Vddo` = bus **N** |
| IGBT `G` | `sw` through the Simulink-PS Converter |
| Current sensor `p` -> `n` | `Inductor2.n` -> **Y**; measured current is positive flowing winding -> Y |
| Inductor `p` / `n` | `Resistor2.n` / sensor `p` |
| Resistor `p` / `n` | `Shared_i` = `Shared_o` / `Inductor2.p` |

So the branch is `Shared -> R -> L -> sensor -> Y`: **positive current flows out of the shared node**,
and the leg is "upper freewheel diode + lower switch" (a **B leg**). The current-sensor orientation is
mirrored with respect to section 3 precisely so that both module types report a positive bias current.

---

## 5. Shared leg (root-level devices)

| Device | Wiring | Notes |
|---|---|---|
| `IGBT (Ideal, Switching)3` | `C` = bus P, `E` = the `Shared` node, gate from `Gain2` <- `On-Off Delay` | upper switch |
| `Diode3` | anode = `Shared`, cathode = bus P | upper freewheel diode |
| `IGBT (Ideal, Switching)4` | `C` = `Shared`, `E` = bus N, gate from `Gain12` <- `On-Off Delay1` | lower switch |
| `Diode4` | anode = bus N, cathode = `Shared` | lower freewheel diode |
| `Current Sensor` | `p` = the `IGBT3.E`/`Diode3.p` node, `n` = `Shared` | output `i_shared_h` = current of the **upper pair into** the shared node |
| `Current Sensor1` | `p` = `Shared`, `n` = the `IGBT4.C`/`Diode4.n` node | output `i_shared_l` = current of the **lower pair out of** the shared node |

Dead-time blocks (masked subsystems `eeGeneralControl/On-Off Delay`):

| Block | Mask variables | Mask values | Chain |
|---|---|---|---|
| `On-Off Delay` | `Td_on=@1;Td_off=@2;IC=@3;Ts=@4;` | `1e-6 | 0 | 0 | 0` | `From[sw0]` -> `On-Off Delay` -> `Gain2` (15) -> `Simulink-PS Converter` -> `IGBT3.G` |
| `On-Off Delay1` | idem | idem | `From[sw0]` -> `NOT5` -> `On-Off Delay1` -> `Gain12` (15) -> `Simulink-PS Converter1` -> `IGBT4.G` |

Measured consequence: 2.01 us of "both switches off" per 50 us carrier period, and the effective
"node at P" duty (0.50081) exceeds the commanded threshold (0.47919) because the node freewheels through
`Diode3` or `Diode4` during the dead time.

---

## 6. Signal routing: gates and currents

### 6.1 Gate chain (modulator -> gate)

| Comparator | Inversion | Amplifier | Goto tag | Drives |
|---|---|---|---|---|
| `PWM_gen` | - | `Gain` (15) | `sw1` | `Bridge_normal_module.sw` (upper switch) |
| `PWM_gen1` | - | `Gain3` (15) | `sw2` | `Bridge_normal_module1.sw` |
| `PWM_gen2` | - | `Gain4` (15) | `sw3` | `Bridge_normal_module2.sw` |
| `PWM_gen3` | - | `Gain5` (15) | `sw4` | `Bridge_normal_module3.sw` |
| `PWM_gen4` | - | `Gain6` (15) | `sw5` | `Bridge_normal_module4.sw` |
| `PWM_gen5` | `NOT4` | `Gain7` (15) | `sw6` | `Bridge_reversed_module.sw` (lower switch) |
| `PWM_gen6` | `NOT` | `Gain8` (15) | `sw7` | `Bridge_reversed_module1.sw` |
| `PWM_gen7` | `NOT1` | `Gain9` (15) | `sw8` | `Bridge_reversed_module2.sw` |
| `PWM_gen8` | `NOT2` | `Gain10` (15) | `sw9` | `Bridge_reversed_module3.sw` |
| `PWM_gen9` | `NOT3` | `Gain11` (15) | `sw10` | `Bridge_reversed_module4.sw` |
| `PWM_gen10` | - / `NOT5` | `Gain2` / `Gain12` (15) | `sw0` | shared leg, upper / lower switch (through the delays) |

### 6.2 Winding current feedback (module -> tag)

| Module | `i_fbk` -> Goto | Tag | Fed to |
|---|---|---|---|
| `Bridge_normal_module` | `Goto` | `i_a1` | `PI_ctrl1.fbk` |
| `Bridge_normal_module1` | `Goto12` | `i_c1` | `PI_ctrl2.fbk` |
| `Bridge_normal_module2` | `Goto13` | `i_a2` | `PI_ctrl3.fbk` |
| `Bridge_normal_module3` | `Goto14` | `i_c2` | `PI_ctrl4.fbk` |
| `Bridge_normal_module4` | `Goto15` | `i_a3` | `PI_ctrl5.fbk` |
| `Bridge_reversed_module` | `Goto16` | `i_c3` | `PI_ctrl6.fbk` |
| `Bridge_reversed_module1` | `Goto17` | `i_a4` | `PI_ctrl7.fbk` |
| `Bridge_reversed_module2` | `Goto18` | `i_c4` | `PI_ctrl8.fbk` |
| `Bridge_reversed_module3` | `Goto19` | `i_a5` | `PI_ctrl9.fbk` |
| `Bridge_reversed_module4` | `Goto20` | `i_c5` | `PI_ctrl10.fbk` |

### 6.3 Controller reference (`AMB_ref_gen` -> tag)

| Controller | `ref` source | `fbk` tag | Winding | Module type |
|---|---|---|---|---|
| `PI_ctrl1` | `AMB_ref_gen` out 1 | `i_a1` | pair 1, a | T |
| `PI_ctrl2` | `AMB_ref_gen` out 2 | `i_c1` | pair 1, c | T |
| `PI_ctrl3` | `AMB_ref_gen1` out 1 | `i_a2` | pair 2, a | T |
| `PI_ctrl4` | `AMB_ref_gen1` out 2 | `i_c2` | pair 2, c | T |
| `PI_ctrl5` | `AMB_ref_gen2` out 1 | `i_a3` | pair 3, a | T |
| `PI_ctrl6` | `AMB_ref_gen2` out 2 | `i_c3` | pair 3, c | **B** (split pair) |
| `PI_ctrl7` | `AMB_ref_gen3` out 1 | `i_a4` | pair 4, a | B |
| `PI_ctrl8` | `AMB_ref_gen3` out 2 | `i_c4` | pair 4, c | B |
| `PI_ctrl9` | `AMB_ref_gen4` out 1 | `i_a5` | pair 5, a | B |
| `PI_ctrl10` | `AMB_ref_gen4` out 2 | `i_c5` | pair 5, c | B |

### 6.4 Command vector order (`Mux` -> `MATLAB Function` / `Add3`)

| Mux input | Source | Sign seen by the modulator |
|---|---|---|
| u1 ... u5 | `PI_ctrl1` ... `PI_ctrl5` (T-module windings) | `+u` |
| u6 ... u10 | `Gain13` ... `Gain17` = `-1 *` (`PI_ctrl6` ... `PI_ctrl10`) | `-u` |

`Demux` output `y1`...`y10` feeds `PWM_gen`...`PWM_gen9` in the same order.

---

## 7. `PI_ctrl*` - per-winding current controller

Interface: `ref` (in 1), `fbk` (in 2), `U_cmd` (out 1).

| Block | Type | Key parameters |
|---|---|---|
| `Add2` | Sum | `Inputs = '+-'` -> `err = ref - fbk` |
| `Gain4` | Gain | `Gain = K_p` |
| `Ictrl_anti_windup` | SubSystem | integrates `K_i*err` with conditional freeze (below) |
| `Add3` | Sum | `Inputs = '++'` -> `U_raw = K_p*err + I` |
| `Saturation` | Saturate | `LowerLimit = -1`, `UpperLimit = +1` |
| `Rate Transition1` | RateTransition | `OutPortSampleTimeOpt = 'Specify'`, `OutPortSampleTime = SW_Ts/4`, `Integrity = on`, `Deterministic = on` |
| `Discrete FIR Filter` | DiscreteFir | `Coefficients = [0.25 0.25 0.25 0.25]`, `FilterStructure = 'Direct form'`; compiled sample time `[1.25e-05 0]` |
| `Rate Transition` | RateTransition | `OutPortSampleTimeOpt = 'Multiple of input port sample time'`, `OutPortSampleTime = SW_Ts` |
| `Memory` | Memory | `InitialCondition = 0`; compiled sample time `[0 1]` (breaks the algebraic loop) |

Connections (identical in all ten controllers):

```
ref(1)                   -> Add2(1)
fbk(1)                   -> Rate Transition1(1) -> Discrete FIR Filter(1) -> Rate Transition(1)
Rate Transition(1)       -> Memory(1) -> Add2(2)
Add2(1)                  -> Gain4(1), Ictrl_anti_windup(1)
Gain4(1)                 -> Add3(1)
Ictrl_anti_windup(1)     -> Add3(2)
Add3(1)                  -> Saturation(1) -> U_cmd(1)
```

There is **no second-order filter** in the feedback path (it was removed after it was shown to be
redundant; see section 4 of the case study).

### 7.1 `PI_ctrl*/Ictrl_anti_windup`

| Block | Type | Parameters |
|---|---|---|
| `Compare To Constant` | SubSystem (`RelationalOperator`) | `relop = '<='`, `const = -1` -> true when the integrator state `x <= -1` |
| `Compare To Constant1` | idem | `relop = '>='`, `const = 1` -> `x >= 1` |
| `Compare To Constant2` | idem | `relop = '>='`, `const = 0` -> `x*err >= 0` |
| `AND` | Logic | **`Operator = OR`**, `Inputs = 2` (the name is misleading) |
| `AND1` | Logic | `Operator = AND`, `Inputs = 2` |
| `Product` | Product | `Inputs = 2` (`x*err`) |
| `Switch` | Switch | `Criteria = 'u2 ~= 0'`, `Threshold = 0` |
| `Constant` | Constant | `Value = 0` |
| `Gain1` | Gain | `Gain = K_i` |
| `Integrator` | Integrator | `LimitOutput = on`, `UpperSaturationLimit = 1`, `LowerSaturationLimit = -1`, `InitialCondition = 0` |

Logic: integrator input = `0` when `((|x| >= 1) OR-checked) AND (x*err >= 0)`, otherwise `err`; the
integrator state is the PI integral term and is also the block output.

---

## 8. `PWM_gen*` - modulator

| Block | Type | Parameters |
|---|---|---|
| `Repeating Sequence` | RepeatingSequence | `rep_seq_t = [0 half_SW_Ts SW_Ts]`, `rep_seq_y = [0 1 0]` - symmetric triangle 0 -> 1 -> 0 |
| `Relational Operator2` | RelationalOperator | `Operator = '>='` - input `U` (threshold) vs the carrier; output `PWM` |

All eleven carriers are in phase (they all start at t = 0), so the winding ripple is coherent across the
model and the differential ripples of same-type pairs cancel.

---

## 9. `AMB_ref_gen*` - differential reference generator

| Block | Type | Parameters | Role |
|---|---|---|---|
| `Constant` | Constant | `Value = bias_current` | the bias term |
| `Gain` | Gain | `Gain = 0.5` | half the control-current reference |
| `Add` | Sum | `Inputs = '++'` | `ia_ref = bias + 0.5*i_ref` (out 1) |
| `Add1` | Sum | `Inputs = '-+'` | `ic_ref = bias - 0.5*i_ref` (out 2) |

---

## 10. Modulation chain and the sort-based min/max

| Block | Type | Parameters | Signals |
|---|---|---|---|
| `Mux` | Mux | `Inputs = 10` | the effective command vector `u_eff` (section 6.4) |
| `MATLAB Function` | EM chart | see below | `cmin` (out 1), `cmax` (out 2) |
| `Add1` | Sum | `Inputs = '++'` | `cmax + cmin` |
| `Gain1` | Gain | `Gain = 0.5` | `m = 0.5*(max + min)` |
| `Add2` | Sum | `Inputs = '+-'` with `Constant5 = 0.5` | `m - 0.5` |
| `Product` | Product | `Inputs = 2` with `Constant7 = [1 ... 1]` | `(m - 0.5)` broadcast to 10 elements |
| `Add3` | Sum | `Inputs = '+-'` | `u_eff + 0.5 - m` -> `Demux` -> 10 winding thresholds |
| `Add4` | Sum | `Inputs = '-+'` with `Constant8 = 0` | `0.5 - m` -> `PWM_gen10` (shared-leg threshold) |

`MATLAB Function` script (verbatim):

```matlab
function [cmin, cmax] = minmax(u)

c_sorted = sort(u);      % ascending
cmin = c_sorted(1);      % first = minimum
cmax = c_sorted(end);    % last  = maximum

end
```

(Sorting instead of a `MinMax` block chain avoids the zero-crossing chattering that aborts the run when
several commands are equal - e.g. at `i_ref = 0`.)

---

## 11. Workspace variables (`parameters_for_AMB_topology.m`)

| Variable | Value | Used by |
|---|---|---|
| `R_value` | `1` | `Resistor2.R` in all ten modules |
| `L_value` | `500e-6` | `Inductor2.l` in all ten modules |
| `U_dc` | `200` | `Voltage Source.dc_voltage` |
| `f_c` | `1500` | gain design only |
| `K_p` | `2*pi*f_c*L_value/U_dc` = 0.0235619 | `PI_ctrl*/Gain4.Gain` |
| `K_i` | `2*pi*f_c*R_value/U_dc` = 47.1239 | `PI_ctrl*/Ictrl_anti_windup/Gain1.Gain` |
| `SW_frequency` | `20e3` | (not referenced by the model; carrier is built from the two below) |
| `SW_Ts` | `1/SW_frequency` = 50e-6 | carrier period, `Rate Transition.OutPortSampleTime` |
| `half_SW_Ts` | `SW_Ts*0.5` = 25e-6 | `Repeating Sequence.rep_seq_t` |
| `bias_current` | `5` | `AMB_ref_gen*/Constant.Value` |

The script's first statement is `clearvars`; run it before touching any other variable.

---

## 12. Channel map summary

| Physical quantity | Simulink signal | Goto tag |
|---|---|---|
| Winding currents (T modules) | `Bridge_normal_module*.i_fbk` | `i_a1`, `i_c1`, `i_a2`, `i_c2`, `i_a3` |
| Winding currents (B modules) | `Bridge_reversed_module*.i_fbk` | `i_c3`, `i_a4`, `i_c4`, `i_a5`, `i_c5` |
| Module gate commands | `PWM_gen`...`PWM_gen9` (x15) | `sw1`...`sw10` |
| Shared-leg command | `PWM_gen10` | `sw0` |
| Shared-leg current, upper pair | `Current Sensor` -> PS-Simulink | `i_shared_h` |
| Shared-leg current, lower pair | `Current Sensor1` -> PS-Simulink | `i_shared_l` |
| Controller outputs | `PI_ctrl*.U_cmd` | (not tagged; mark the lines for logging if needed) |

To record any of these, mark the `From` output line (or the block output port) for logging with
`Simulink.sdi.markSignalForStreaming` and read `out.logsout` - see `scripts/run_case_study.m`.
