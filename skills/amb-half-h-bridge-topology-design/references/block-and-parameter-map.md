# Exact block, parameter, and signal map

Reference model: `Half_H_Bridge_with_RL_load.slx` (10 half-H bridges / 20 legs / 5 DOF), bundled with the skill at
`assets/Half_H_Bridge_with_RL_load.slx` (md5 `c14d7d1984439a5fa3de7977664b5482`), with its
parameters at `assets/parameters_for_AMB_topology.m` (md5 `a90c6e390a9c1e14bf015396b2d6054d`).

Verified with **MATLAB 25.2.0.3312555 (R2025b) Update 6, Simulink 25.2, Simscape 25.2,
Simscape Electrical 25.2 (Linux/WSL2)**. Every name, path, parameter and value below was read from
the model with `get_param`/`find_system`; nothing here is paraphrased. Re-check it at any time with
`scripts/verify_model_facts.m`, which asserts every entry of this file against the bundled model.

**Naming note.** Several blocks were created by pasting a library block whose display name wraps,
so their **stored** names contain embedded newline characters - for example the stored name of the
upper switch is `IGBT\n(Ideal,\nSwitching)`. Simulink resolves a block path when those newlines are
written as spaces (e.g. `Half_H_module/IGBT (Ideal, Switching)`), which is the form used below.

---

## 1. Model configuration

| Parameter (root model) | Value |
|---|---|
| `Solver` | `VariableStepAuto` |
| `SolverType` | `Variable-step` |
| `SolverName` | `VariableStepAuto` |
| `StopTime` | `0.1` |
| `MaxStep` | `1e-6` |
| `MinStep` / `InitialStep` / `AbsTol` | `auto` |
| `RelTol` | `1e-4` |
| `FixedStep` | `1e-6` (unused while the solver is variable-step) |
| `StartTime` | `0.0` |
| `SignalLogging` / `SignalLoggingName` | `on` / `logsout` |

`Solver Configuration` block (library `nesl_utility/Solver Configuration`):

| Parameter | Value |
|---|---|
| `UseLocalSolver` | `off` |
| `LocalSolverChoice` | `NE_TRAPEZOIDAL_ADVANCER` |
| `LocalSolverSampleTime` | `1e-6` |
| `DoFixedCost` | `off` |
| `AutomaticFiltering` | `on` |
| `FilteringTimeConstant` | `0.001` |
| `MaxNonlinIter` | `3` |

---

## 2. Root-level blocks

| Block name | BlockType | `ReferenceBlock` (library path) | Key parameters |
|---|---|---|---|
| `Voltage Source` | SimscapeBlock | `ee_lib/Sources/Voltage Source` | `dc_voltage = U_dc`, `sample_time = 1e-3` |
| `Solver Configuration` | SubSystem (masked) | `nesl_utility/Solver Configuration` | see section 1 |
| `Electrical Reference` | SimscapeBlock | `fl_lib/Electrical/Electrical Elements/Electrical Reference` | - |
| `Open Circuit`, `Open Circuit1` | SimscapeBlock | `fl_lib/Electrical/Electrical Elements/Open Circuit` | terminate the far end of the bus chain |
| `Half_H_module` ... `Half_H_module9` | SubSystem | - | 10 identical half-H bridges (2 legs each) |
| `PI_ctrl` ... `PI_ctrl9` | SubSystem | - | 10 identical controllers |
| `PWM_gen` ... `PWM_gen9` | SubSystem | - | 10 identical modulators |
| `AMB_ref_gen` ... `AMB_ref_gen4` | SubSystem | - | 5 reference generators |
| `Constant1` ... `Constant5` | Constant | - | `Value = 1` (control-current reference of DOF 1..5) |
| `winding_currents` | Scope | - | `NumInputPorts = 10`, `SampleTime = -1`, `DataLogging = off`, `DataLoggingVariableName = ScopeData`, `DataLoggingSaveFormat = Dataset` |
| `Goto`/`From` (30 pairs) | Goto / From | - | see section 8 |

Bus wiring (physical connections): `Voltage Source.p` -> `Half_H_module.Vcci`;
bridge *k* `.Vcco` -> bridge *k+1* `.Vcci` and `.Vddo` -> `.Vddi`; the last bridge's `.Vcco`/`.Vddo`
go to `Open Circuit`/`Open Circuit1`; `Half_H_module.Vddi`, `Voltage Source.n`,
`Electrical Reference.V` and `Solver Configuration` share the negative rail node.

---

## 3. `Half_H_module` - one per-winding half-H bridge

### 3.1 Ports

| Kind | Name | Port |
|---|---|---|
| Inport | `sw1` | 1 |
| Inport | `sw2` | 2 |
| Outport | `i` | 1 |
| Connection Port | `Vcci` | `LConn1` |
| Connection Port | `Vddi` | `LConn2` |
| Connection Port | `Vcco` | `RConn1` |
| Connection Port | `Vddo` | `RConn2` |

### 3.2 Blocks

| Block name | BlockType | `ReferenceBlock` | Key parameters |
|---|---|---|---|
| `IGBT (Ideal, Switching)` (upper) | SimscapeBlock | `ee_lib/Semiconductors & Converters/IGBT (Ideal, Switching)` | `Ron = 0.001`, `Vf = 0.8`, `Goff = 1e-5`, `Vth = 6`, `diode_param = ee.enum.semiconductors.protectionDiode.external` |
| `IGBT (Ideal, Switching)1` (lower) | SimscapeBlock | same | same |
| `Diode` (upper freewheel) | SimscapeBlock | `fl_lib/Electrical/Electrical Elements/Diode` | `Vf = 0.6`, `Ron = 0.3`, `Goff = 1e-8` |
| `Diode1` (lower freewheel) | SimscapeBlock | same | same |
| `Resistor` | SimscapeBlock | `fl_lib/Electrical/Electrical Elements/Resistor` | `R = R_value` |
| `Inductor` | SimscapeBlock | `fl_lib/Electrical/Electrical Elements/Inductor` | `L = L_value` |
| `Current Sensor` | SimscapeBlock | `fl_lib/Electrical/Electrical Sensors/Current Sensor` | - |
| `Simulink-PS Converter` | SubSystem (masked) | `nesl_utility/Simulink-PS Converter` | drives the upper gate |
| `Simulink-PS Converter1` | SubSystem (masked) | `nesl_utility/Simulink-PS Converter` | drives the lower gate |
| `PS-Simulink Converter` | SubSystem (masked) | `nesl_utility/PS-Simulink Converter` | current measurement to Simulink |

### 3.3 Netlist (node names as in section 1.1 of the skill)

| Device terminal | Node |
|---|---|
| upper IGBT. C | P (`Vcci` = `Vcco`) |
| upper IGBT. E | M |
| lower IGBT. C | A |
| lower IGBT. E | N (`Vddi` = `Vddo`) |
| `Diode` (anode / cathode) | A / P |
| `Diode1` (anode / cathode) | N / M |
| `Resistor`.p, `.n` | M, `Inductor`.p |
| `Inductor`.n | `Current Sensor`.p |
| `Current Sensor`.n | A |
| `Current Sensor`.I -> `PS-Simulink Converter` -> outport `i` | - |

---

## 4. `PI_ctrl` - per-winding current controller

### 4.1 Ports
`ref` (in 1), `fbk` (in 2), `U_cmd` (out 1).

### 4.2 Blocks and connections

| Block name | BlockType | Key parameters | Role |
|---|---|---|---|
| `Rate Transition` | RateTransition | `OutPortSampleTimeOpt = 'Specify'`, `OutPortSampleTime = SW_Ts`, `Integrity = 'on'`, `Deterministic = 'on'`, `InitialCondition = 0` | samples `fbk` once per switching period |
| `Memory` | Memory | `InheritSampleTime = 'off'`, `InitialCondition = 0` | breaks the algebraic loop (no direct feedthrough) |
| `Second-Order Filter` | SubSystem (masked) | lib `eeGeneralControl/Second-Order Filter`; `FilterType = 'Low-pass'`, `fn = 6000`, `Zeta = 0.707`, `Ts = 0` (mask string `Low-pass|6000|0.707|0|0|0|0|0`) | feedback anti-aliasing / measurement filter |
| `Add2` | Sum | `Inputs = '+-'` | `err = ref - fbk_filtered` |
| `Gain4` | Gain | `Gain = K_p` | proportional term |
| `Ictrl_anti_windup` | SubSystem | see section 5 | integral term |
| `Add3` | Sum | `Inputs = '++'` | `U_raw = K_p*err + Int_out` |
| `Saturation` | Saturate | `LowerLimit = -1`, `UpperLimit = 1` | per-unit voltage clamp |

Signal chain: `fbk -> Rate Transition -> Memory -> Second-Order Filter -> Add2(-)`;
`err -> Gain4 -> Add3(+)`; `err -> Ictrl_anti_windup -> Add3(+)`; `Add3 -> Saturation -> U_cmd`.

---

## 5. `PI_ctrl/Ictrl_anti_windup` - conditional-integration anti-windup

Ports: `err` (in 1), `Int_out` (out 1).

| Block name | BlockType | Key parameters | Role |
|---|---|---|---|
| `Switch` | Switch | `Criteria = 'u2 ~= 0'`, `Threshold = 0` | selects `0` (u1) when the block condition is true, otherwise `err` (u3) |
| `Constant` | Constant | `Value = 0` | the "freeze integration" input |
| `Gain1` | Gain | `Gain = K_i` | integral gain |
| `Integrator` | Integrator | `LimitOutput = 'on'`, `UpperSaturationLimit = 1`, `LowerSaturationLimit = -1`, `InitialCondition = 0` | integral state |
| `Compare To Constant` | SubSystem (masked) | lib `simulink/Logic and Bit Operations/Compare To Constant`; `relop = '<='`, `const = -1` | `x <= -1` |
| `Compare To Constant1` | SubSystem (masked) | same lib; `relop = '>='`, `const = 1` | `x >= 1` |
| `AND` | Logic | **`Operator = 'OR'`** (block name is `AND` but the operator is OR) | `abs(x) >= 1` |
| `Product` | Product | `Inputs = 2` | `x * err` |
| `Compare To Constant2` | SubSystem (masked) | same lib; `relop = '>='`, `const = 0` | `x*err >= 0` |
| `AND1` | Logic | `Operator = 'AND'` | "saturated AND error pushing further" |

Logic: `AND(OR(x<=-1, x>=1), (x*err >= 0))` drives `Switch`; when it is true the integrator input
is forced to `0`, otherwise the raw `err` is integrated. The `Integrator` output clamp
(`+/-1`) remains as a second line of defence.

---

## 6. `PWM_gen` - three-level modulator

Ports: `U_RL` (in 1), `sw1` (out 1), `sw2` (out 2).

| Block name | BlockType | Key parameters | Expression |
|---|---|---|---|
| `Repeating Sequence` | SubSystem (masked) | lib `simulink/Sources/Repeating Sequence`; `rep_seq_t = [0 half_SW_Ts SW_Ts]`, `rep_seq_y = [0 1 0]` | symmetric triangle carrier, period `SW_Ts` |
| `Constant` | Constant | `Value = 0.5` | feeds `Add` |
| `Gain1` | Gain | `Gain = 0.5` | `0.5*U_RL` into `Add` |
| `Add` | Sum | `Inputs = '++'` | `d1 = 0.5 + 0.5*U_RL` |
| `Constant1` | Constant | `Value = 0.5` | feeds `Add1` |
| `Gain` | Gain | `Gain = 0.5` | `0.5*U_RL` into `Add1` |
| `Add1` | Sum | `Inputs = '+-'` | `d2 = 0.5 - 0.5*U_RL` |
| `Relational Operator` | RelationalOperator | `Operator = '<='` | `d1 <= tri` |
| `Relational Operator1` | RelationalOperator | `Operator = '<='` | `d2 <= tri` |
| `Logical Operator` | Logic | `Operator = 'NOT'` | `NOT(d1 <= tri)` |
| `Gain3` | Gain | `Gain = 15` | `sw1 = 15*(d2 <= tri)` |
| `Gain2` | Gain | `Gain = 15` | `sw2 = 15*NOT(d1 <= tri)` |

---

## 7. `AMB_ref_gen` - differential reference generation

Ports: `i_ref` (in 1), `ia_ref` (out 1), `ic_ref` (out 2).

| Block name | BlockType | Key parameters | Expression |
|---|---|---|---|
| `Constant` | Constant | `Value = bias_current` | bias term |
| `Gain` | Gain | `Gain = 0.5` | `0.5*i_ref` |
| `Add` | Sum | `Inputs = '++'` | `ia_ref = bias_current + 0.5*i_ref` |
| `Add1` | Sum | `Inputs = '-+'` | `ic_ref = bias_current - 0.5*i_ref` |

Port order: `ia_ref` is outport 1 and `ic_ref` is outport 2.

---

## 8. Signal routing and channel map

30 `Goto` blocks at root publish the 20 gate signals and the 10 winding currents; matching
`From` blocks consume them inside `PI_ctrl` and at the `winding_currents` scope.

Goto tags: `sw1_a1`, `sw2_a1`, `sw1_c1`, `sw2_c1`, ... `sw1_a5`, `sw2_a5`, `sw1_c5`, `sw2_c5`
(20 tags) and `i_a1` ... `i_a5`, `i_c1` ... `i_c5` (10 tags).

| DOF | a-bridge | c-bridge | a-controller | c-controller | a-modulator | c-modulator | reference gen | ref constant | current tags | scope inputs |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | `Half_H_module` | `Half_H_module1` | `PI_ctrl` | `PI_ctrl1` | `PWM_gen` | `PWM_gen1` | `AMB_ref_gen` | `Constant1` | `i_a1` / `i_c1` | u1 / u2 |
| 2 | `Half_H_module2` | `Half_H_module3` | `PI_ctrl2` | `PI_ctrl3` | `PWM_gen2` | `PWM_gen3` | `AMB_ref_gen1` | `Constant2` | `i_a2` / `i_c2` | u3 / u4 |
| 3 | `Half_H_module4` | `Half_H_module5` | `PI_ctrl4` | `PI_ctrl5` | `PWM_gen4` | `PWM_gen5` | `AMB_ref_gen2` | `Constant3` | `i_a3` / `i_c3` | u5 / u6 |
| 4 | `Half_H_module6` | `Half_H_module7` | `PI_ctrl6` | `PI_ctrl7` | `PWM_gen6` | `PWM_gen7` | `AMB_ref_gen3` | `Constant4` | `i_a4` / `i_c4` | u7 / u8 |
| 5 | `Half_H_module8` | `Half_H_module9` | `PI_ctrl8` | `PI_ctrl9` | `PWM_gen8` | `PWM_gen9` | `AMB_ref_gen4` | `Constant5` | `i_a5` / `i_c5` | u9 / u10 |

---

## 9. Workspace variables (`parameters_for_AMB_topology.m`)

| Variable | Expression / value |
|---|---|
| `R_value` | `1` ohm |
| `L_value` | `500e-6` H |
| `U_dc` | `200` V |
| `f_c` | `1500` Hz (current-loop design bandwidth) |
| `K_p` | `2*pi*f_c*L_value/U_dc` = `0.023562` |
| `K_i` | `2*pi*f_c*R_value/U_dc` = `47.1239` |
| `SW_frequency` | `20e3` Hz |
| `SW_Ts` | `1/SW_frequency` = `50e-6` s |
| `half_SW_Ts` | `SW_Ts*0.5` = `25e-6` s |
| `bias_current` | `5` A |

Derived: `Ki/Kp = R_value/L_value = 2000 rad/s`; open-loop crossover
`K_p*U_dc/L_value = 9425 rad/s = 1500 Hz`.

**Load order matters.** The parameter script starts with `clearvars`, and the model resolves its
block parameters (`R_value`, `L_value`, `U_dc`, `K_p`, `K_i`, `SW_Ts`, `half_SW_Ts`,
`bias_current`) from the base workspace: run it before loading or simulating the model. It does not
reference `f_c` itself - only `K_p`/`K_i` are computed from it.
