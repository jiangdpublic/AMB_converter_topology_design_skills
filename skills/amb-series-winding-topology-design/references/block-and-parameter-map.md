# Block, Parameter, Netlist and Signal Map

Reference model `model/Series_winding_with_RL_load.slx` with parameters in
`model/parameters_for_AMB_topology.m`. Every entry below was read out of the bundled model;
`scripts/verify_model_facts.m` re-checks them against the bundle and prints PASS/FAIL.

Environment in which the model was built and verified:

| | |
|---|---|
| MATLAB | **R2025b (25.2)**, 25.2.0.3312555 Update 6 |
| Simulink | **25.2** |
| Simscape / Simscape Electrical | **25.2** |
| Platform | Linux (GLNXA64) |

---

## 1. Root-level blocks

| Block | Type | Notes |
|---|---|---|
| `Bridge_module`, `Bridge_module1` ... `Bridge_module10` | SubSystem | **11 legs**; 15 blocks each |
| `RL_load`, `RL_load1` ... `RL_load9` | SubSystem | **10 windings**; 8 blocks each |
| `AMB_ref_gen`, `AMB_ref_gen1` ... `AMB_ref_gen4` | SubSystem | bias + differential reference, one per DOF; 8 blocks each |
| `PI_ctrl`, `PI_ctrl1` ... `PI_ctrl9` | SubSystem | one per winding; 12 blocks each |
| `PWM_gen`, `PWM_gen1` ... `PWM_gen10` | SubSystem | one per leg; 11 blocks each |
| `ref_delay`, `ref_delay1` ... `ref_delay3` | SubSystem | staggered enable; 9 blocks each |
| `Step`, `Step1`, `Step2`, `Step3` | Step | `Time` = `0.001*4`, `0.001*3`, `0.001*2`, `0.001*1`; Before 0, After 1 |
| `MATLAB Function` | MATLAB Function | `function [cmin, cmax] = minmax(u)` |
| `Mux1` | Mux | 11 inputs |
| `Demux` | Demux | 11 outputs |
| `Add`, `Add4` ... `Add12`, `Add1`, `Add2`, `Add3` | Sum | see section 5 |
| `Product` | Product | 2 inputs |
| `Gain1` | Gain | `Gain = 0.5` |
| `Constant` | Constant | `Value = 0` (feeds `Add` and `Mux1` input 11) |
| `Constant1` ... `Constant4`, `Constant6` | Constant | `Value = 1` - the `i_ref` of each DOF |
| `Constant5` | Constant | `Value = 0.5` |
| `Constant7` | Constant | `Value = [1 1 1 1 1 1 1 1 1 1 1]` |
| `Voltage Source` | SimscapeBlock | `ee_lib/Sources/Voltage Source`, `dc_voltage = U_dc` |
| `Electrical Reference` | SimscapeBlock | `fl_lib/Electrical/Electrical Elements/Electrical Reference` |
| `Solver Configuration` | SimscapeBlock | `nesl_utility/Solver Configuration` |
| `Open Circuit`, `Open Circuit1` ... `Open Circuit3` | SimscapeBlock | `fl_lib/.../Open Circuit`; terminate unused feed-through ports |
| `winding_currents` | Scope | 10 inputs, `i_a1` ... `i_c5`; **`DataLogging = off`** |

---

## 2. Inside `Bridge_module` (one standard half-bridge leg)

| Block | Library reference | Key parameters |
|---|---|---|
| `IGBT (Ideal, Switching)1`, `...2` | `ee_lib/Semiconductors & Converters/IGBT (Ideal, Switching)` | `Vf = 0.8`, `Ron = 0.001`, `Goff = 1e-5`, `Vth = 6`, `diode_param = external`, `port_option = ee.enum.controlport.ps` |
| `Diode1`, `Diode2` | `fl_lib/Electrical/Electrical Elements/Diode` | `Vf = 0.6`, `Ron = 0.3`, `Goff = 1e-8` |
| `Simulink-PS Converter1`, `...2` | `nesl_utility/Simulink-PS Converter` | gate drive inputs |
| `swh`, `swl` | Inport | 1, 2 |

Connection ports (all six are `PMIOPort` blocks with `Side = Left`/`Right`):

| Port | Side | Internal node |
|---|---|---|
| `Vcci` | Left | IGBT1 collector = Diode1 cathode = `Vcco` |
| `Vsi` | Left | IGBT1 emitter = IGBT2 collector = Diode1 anode = Diode2 cathode = `Vso` |
| `Vddi` | Left | IGBT2 emitter = Diode2 anode = `Vddo` |
| `Vcco` | Right | same node as `Vcci` (DC bus positive) |
| `Vso` | Right | same node as `Vsi` (leg midpoint) |
| `Vddo` | Right | same node as `Vddi` (DC bus negative) |

Gate wiring: `swh` -> `Simulink-PS Converter2` -> `IGBT (Ideal, Switching)1.G`;
`swl` -> `Simulink-PS Converter1` -> `IGBT (Ideal, Switching)2.G`.

Resulting topology:

```
+U_dc --+-- IGBT1(C->E) --+-- leg midpoint --+-- IGBT2(C->E) --+-- 0
        |                 |                  |                 |
      Diode1(up) --------+               Diode2(up) ----------+
```

---

## 3. Inside `RL_load` (one winding)

| Block | Library reference | Key parameters |
|---|---|---|
| `Resistor2` | `fl_lib/Electrical/Electrical Elements/Resistor` | `R = R_value` |
| `Inductor2` | `fl_lib/Electrical/Electrical Elements/Inductor` | `l = L_value`, `r = 0`, `g = 1e-9` |
| `Current Sensor2` | `fl_lib/Electrical/Electrical Sensors/Current Sensor` | outputs `I` and `i1` |
| `PS-Simulink Converter` | `nesl_utility/PS-Simulink Converter` | drives `i_fbk` |
| `L`, `R` | PMIOPort | `Side = Left`, `Side = Right` |
| `i_fbk` | Outport | 1 |

Series order and sign convention:

```
L(port) -- Resistor2 -- Inductor2 -- Current Sensor2 -- R(port)
```

The sensor is positive for current flowing **from the L port to the R port**, i.e. from module k to module k+1.

---

## 4. Inside `PI_ctrl`, `PWM_gen`, `AMB_ref_gen`, `ref_delay`

### 4.1 `PI_ctrl`

| Block | Type | Key parameters |
|---|---|---|
| `ref`, `fbk` | Inport | 1, 2 |
| `U_cmd` | Outport | 1 |
| `Add2` | Sum | `Inputs = '+-'` |
| `Add3` | Sum | `Inputs = '++'` |
| `Gain4` | Gain | `Gain = K_p` |
| `Rate Transition` | RateTransition | `OutPortSampleTimeOpt = Specify`, `OutPortSampleTime = SW_Ts`, `Integrity = on`, `Deterministic = on` |
| `Memory` | Memory | `InitialCondition = 0` |
| `Second-Order Filter` | `eeGeneralControl/Second-Order Filter` | `FilterType = Low-pass`, `fn = 6000`, `Zeta = 0.707`, `Ts = 0` |
| `Saturation` | Saturate | `UpperLimit = 1`, `LowerLimit = -1` |
| `Ictrl_anti_windup` | SubSystem | see 4.2 |

Signal path: `fbk` -> `Rate Transition` -> `Memory` -> `Second-Order Filter` -> `Add2(-)`;
`ref` -> `Add2(+)`; `Add2` -> `Gain4` and `Ictrl_anti_windup`; `Gain4` and the integrator output -> `Add3` -> `Saturation` -> `U_cmd`.

### 4.2 `PI_ctrl/Ictrl_anti_windup`

| Block | Type | Key parameters |
|---|---|---|
| `err`, `Int_out` | Inport / Outport | |
| `Gain1` | Gain | `Gain = K_i` |
| `Integrator` | Integrator | `InitialCondition = 0`, `LimitOutput = on`, `UpperSaturationLimit = 1`, `LowerSaturationLimit = -1` |
| `Product` | Product | 2 inputs, element-wise |
| `OR` | Logic | `Operator = OR`, `Inputs = 2` - combines the two limit tests into `abs(x) >= 1` |
| `AND` | Logic | `Operator = AND`, `Inputs = 2` - combines `abs(x) >= 1` with `x*err >= 0` |
| `Compare To Constant` | `simulink/Logic and Bit Operations/Compare To Constant` | `relop = '<='`, `const = -1` |
| `Compare To Constant1` | same | `relop = '>='`, `const = 1` |
| `Compare To Constant2` | same | `relop = '>='`, `const = 0` |
| `Constant` | Constant | `Value = 0` |
| `Switch` | Switch | `Criteria = 'u2 ~= 0'`, `Threshold = 0` |

> **Naming caution.** In an earlier revision of this model the block computing `abs(x) >= 1` was
> *named* `AND` although its `Operator` was `OR`, which is a classic trap for anyone reading the
> diagram instead of the parameters. It is now named `OR`. The check in `verify_model_facts.m` asserts
> the **operator**, not the name.

### 4.3 `PWM_gen`

| Block | Type | Key parameters |
|---|---|---|
| `U` | Inport | modulation threshold |
| `Repeating Sequence` | `simulink/Sources/Repeating Sequence` | `rep_seq_t = [0 half_SW_Ts SW_Ts]`, `rep_seq_y = [0 1 0]` |
| `Relational Operator2` | RelationalOperator | `Operator = '>='` |
| `NOT` | Logic | `Operator = NOT` |
| `On-Off Delay`, `On-Off Delay1` | `eeGeneralControl/On-Off Delay` | `Td_on = 1e-6`, `Td_off = 0`, `IC = 0`, `Ts = 0` |
| `Gain`, `Gain1` | Gain | `Gain = 15` |
| `PWMH`, `PWML` | Outport | 1, 2 |

Signal path: `Relational Operator2` -> `On-Off Delay1` -> `Gain` -> `PWMH`;
`Relational Operator2` -> `NOT` -> `On-Off Delay` -> `Gain1` -> `PWML`.
Because the carrier rises 0->1->0, the comparator output is one contiguous interval across the period
boundary, so each gate loses exactly one `Td_on` per period.

### 4.4 `AMB_ref_gen`

| Block | Type | Key parameters |
|---|---|---|
| `i_ref` | Inport | 1 |
| `ia_ref`, `ic_ref` | Outport | 1, 2 |
| `Gain` | Gain | `Gain = 0.5` |
| `Constant` | Constant | `Value = bias_current` |
| `Add` | Sum | `Inputs = '++'` |
| `Add1` | Sum | `Inputs = '-+'` |

`ia_ref = bias_current + 0.5*i_ref`, `ic_ref = bias_current - 0.5*i_ref`.

### 4.5 `ref_delay`

| Block | Type | Key parameters |
|---|---|---|
| `ref1`, `ref2`, `ctrl` | Inport | 1, 2, 3 |
| `delay1`, `delay2` | Outport | 1, 2 |
| `Constant8` | Constant | `Value = 0` |
| `Switch`, `Switch1` | Switch | `Criteria = 'u2 > Threshold'`, `Threshold = 0` |

Before enable (`ctrl = 0`) both outputs are forced to 0; after enable they pass `ref1` and `ref2`.

---

## 5. Modulator wiring at the root

Cumulative sums (each `Add` has `Inputs = '++'`):

| Block | Output |
|---|---|
| `Add` | `u9` (PI_ctrl9 plus Constant 0) |
| `Add4` | `u8 + Add` |
| `Add5` ... `Add11` | successively adding `u7` ... `u1` |
| `Add12` | `u0 + Add11` = sum of all ten `U_cmd` |

`Mux1` collects, in order: `Add12`, `Add11`, `Add10`, `Add9`, `Add8`, `Add7`, `Add6`, `Add5`, `Add4`, `Add`, `Constant` (0).
So `Mux1 = [e1 .. e11]` with `e_j = sum_(k=j..10) u_(k-1)` and `e11 = 0`.

Min-max injection:

| Block | Function |
|---|---|
| `MATLAB Function` | `cmin = min(e)`, `cmax = max(e)` |
| `Add1` | `cmin + cmax` |
| `Gain1` | `0.5*(cmin + cmax)` |
| `Add2` | `0.5*(cmin+cmax) - 0.5` (`Inputs = '+-'` against `Constant5 = 0.5`) |
| `Product` | `Add2 * Constant7` (broadcast to 11) |
| `Add3` | `Mux1 - Product` = `b`, the 11 modulation thresholds (`Inputs = '+-'`) |

`Demux` sends `b(1..11)` to `PWM_gen`, `PWM_gen1` ... `PWM_gen10` in that order.

---

## 6. Electrical netlist

Winding w (1-based) is `RL_load` for w = 1 and `RL_load<w-1>` for w = 2..10.
Leg m (0-based) is `Bridge_module` for m = 0 and `Bridge_module<m>` for m = 1..10.

| Edge | Meaning |
|---|---|
| `Voltage Source.p` - `Bridge_module.Vcci` | DC bus positive |
| `Voltage Source.n` - `Electrical Reference.V` - `Solver Configuration.RConn1` - `Bridge_module.Vddi` | DC bus negative and ground |
| `Bridge_module<m>.Vcco` - `Bridge_module<m+1>.Vcci` | bus positive chained through the legs |
| `Bridge_module<m>.Vddo` - `Bridge_module<m+1>.Vddi` | bus negative chained through the legs |
| `Bridge_module<m>.Vso` - `RL_load<w=m+1>.L` | winding w starts at the midpoint of leg m |
| `Bridge_module<m+1>.Vso` - `RL_load<w=m+1>.R` | winding w ends at the midpoint of leg m+1 |
| `Open Circuit3` - `Bridge_module.Vsi` | unused midpoint feed-through of leg 0 |
| `Open Circuit1` - `Bridge_module10.Vso`, `Open Circuit2` - `.Vcco`, `Open Circuit` - `.Vddo` | unused ports of leg 10 |

Chain order and PWM pairing:

| Leg m | PWM block | Winding in front (w) | Winding behind (w) |
|---|---|---|---|
| 0 | `PWM_gen` | - | w1 = a1 |
| 1 | `PWM_gen1` | w1 = a1 | w2 = c1 |
| 2 | `PWM_gen2` | w2 = c1 | w3 = a2 |
| ... | ... | ... | ... |
| 9 | `PWM_gen9` | w9 = a5 | w10 = c5 |
| 10 | `PWM_gen10` | w10 = c5 | - |

---

## 7. Signals, Goto/From tags and logging

| Signal | Source | Destination |
|---|---|---|
| `i_fbk` of `RL_load` ... `RL_load9` | current sensor | `Goto21` ... `Goto30` |
| `i_a1`, `i_c1` ... `i_a5`, `i_c5` | `From30` ... `From39` | `winding_currents` inputs 1..10 |
| `ia_ref`, `ic_ref` of each `AMB_ref_gen` | reference generator | `PI_ctrl` (a) and `PI_ctrl` (c) of that DOF |
| `delay1`, `delay2` of each `ref_delay` | enable switch | `PI_ctrl` reference inputs of that DOF |
| `PWMH`, `PWML` of each `PWM_gen` | modulator | `Goto`/`From` pair into the matching `Bridge_module.swh` / `.swl` |
| `U_cmd` of each `PI_ctrl` | controller | the cumulative-sum chain, then `Mux1` |

The model was delivered with `SignalLogging = on` but the only recording block is the Scope
`winding_currents`, whose `DataLogging` is `off`. `scripts/run_case_study.m` adds the To Workspace taps
it needs on a **temporary copy**, so the bundled asset stays byte-identical to the engineer's model.

---

## 8. Base-workspace variables required by the model

`parameters_for_AMB_topology.m` defines exactly these eight variables:

| Variable | Value | Used by |
|---|---|---|
| `R_value` | 1 | `RL_load/Resistor2.R` |
| `L_value` | 500e-6 | `RL_load/Inductor2.l` |
| `U_dc` | 200 | `Voltage Source.dc_voltage`, `K_p`, `K_i` |
| `f_c` | 1500 | `K_p`, `K_i` |
| `K_p` | `2*pi*f_c*L_value/U_dc` = 0.023562 | `PI_ctrl/Gain4` |
| `K_i` | `2*pi*f_c*R_value/U_dc` = 47.1239 | `Ictrl_anti_windup/Gain1` |
| `SW_frequency`, `SW_Ts`, `half_SW_Ts` | 20e3, 50e-6, 25e-6 | `PWM_gen/Repeating Sequence`, `PI_ctrl/Rate Transition` |
| `bias_current` | 5 | `AMB_ref_gen/Constant` |

The script begins with `clearvars`, so it must be run before any other variable is defined for the model.