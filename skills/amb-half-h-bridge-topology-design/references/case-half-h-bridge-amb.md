# Case study: 5-DOF AMB half-H bridge topology and current loop

Model: `Half_H_Bridge_with_RL_load.slx`, bundled with the skill at
`assets/Half_H_Bridge_with_RL_load.slx` (md5 `c14d7d1984439a5fa3de7977664b5482`), together with its
parameter script `assets/parameters_for_AMB_topology.m` (md5 `a90c6e390a9c1e14bf015396b2d6054d`).
This file records the measured evidence behind the skill: the model inventory, the
feedback-sampling defect found in the first version (v1), the fixes applied by the model owner
(v2), the before/after measurements, and the snippets used to reproduce them.

All numbers are **time-weighted** steady-state values unless stated otherwise, produced by
running instrumented copies of the model and post-processing the logged data on a uniform time
base. The original model file was never modified.

---

## 1. Model inventory

| Item | Value |
|---|---|
| Windings | 10 (8 radial + 2 axial = 5 DOF x 2) |
| Bridges (legs) | 10 x `Half_H_module` (2 IGBT + 2 diode each = 10 bridges / 20 legs; 20 switches + 20 diodes) |
| Controllers | 10 x `PI_ctrl` (one per winding) |
| Modulators | 10 x `PWM_gen`, all driven by the same 20 kHz triangle |
| Reference generators | 5 x `AMB_ref_gen`, one per DOF, fed by `Constant1`..`Constant5` = 1 A |
| Bus | one ideal 200 V `Voltage Source`, bridges daisy-chained |
| Load per winding | `R_value = 1` ohm, `L_value = 500e-6` H |
| Controller gains | `K_p = 0.023562`, `K_i = 47.1239`, clamp +/-1 |
| Feedback filter | second-order low-pass `fn = 6000` Hz, `Zeta = 0.707` |
| Toolchain | **MATLAB 25.2.0.3312555 (R2025b) Update 6**, Simulink 25.2, Simscape 25.2, Simscape Electrical 25.2, Linux (WSL2) - every number in this file was reproduced on that release; both scripts print the toolchain they run on |

Block names, library paths, parameter names and the channel map are in
`block-and-parameter-map.md`.

---

## 2. v1: the feedback-sampling defect

### 2.1 Symptom

With constant references (`i_ref = 1` A, `bias_current = 5` A), the steady state did not match the
references:

| Quantity | Measured | Reference | Error |
|---|---|---|---|
| `i_a` | 5.431 A | 5.5 A | -1.25 % |
| `i_c` | 4.441 A | 4.5 A | -1.31 % |
| `i_a - i_c` | 0.989 A | 1.0 A | -1.1 % |
| bias, `(i_a + i_c)/2` | 4.936 A | 5.0 A | -1.28 % |

The PI integrator was holding a constant output while a steady-state error remained - which is
impossible for a correctly measured integral loop, so the feedback itself had to be wrong.

### 2.2 Isolation chain

1. **Filter DC gain?** The `Second-Order Filter` block was exercised standalone with a 20 kHz
   square wave: input mean 5.20025 A, output mean 5.19995 A. The filter is correct.
2. **Sensor / wiring?** The `Memory` block input and the logged winding current had identical
   means (5.42276 vs 5.42278 A). The measurement path is correct.
3. **The `Memory` block?** The **time-weighted** mean of the `Memory` output was **5.50001 A**,
   while the true time-weighted winding current was **5.43065 A**.

So the controller was regulating a feedback signal that read **+0.069 A (+1.27 %)** high, and the
true current settled 1.25 % low. The filter output matched the Memory output (5.4996 A), confirming
the Memory was the source.

### 2.3 Root cause

The `Memory` block performs a zero-order hold of the **previous major time step**. With
`MaxStep = 'auto'` the variable-step solver took steps of 10-12 us - comparable to the 25 us
ripple period - during the slow freewheel decay, and the held value over-estimated the decaying
current. Integrated over a ripple period this produced a stable +1.3 % measurement bias.

Supporting evidence: constraining `MaxStep` to `2e-6` shrank the bias from 0.069 A to 0.027 A;
forcing an explicit `SW_Ts` sample (v2) removed it almost completely.

---

## 3. v2: changes applied and re-verification

### 3.1 Changes

| Change | Implementation |
|---|---|
| Feedback sampled at the switching frequency | `Rate Transition` inserted before `Memory` in every `PI_ctrl`: `OutPortSampleTimeOpt = 'Specify'`, `OutPortSampleTime = SW_Ts`, `Integrity = 'on'`, `Deterministic = 'on'`, `InitialCondition = 0`; all ten samplers share phase (offset 0) |
| Integral anti-windup | New subsystem `Ictrl_anti_windup` implementing conditional integration: `OR(x<=-1, x>=1)` AND `(x*err >= 0)` forces the integrator input to 0; the integrator's own clamp stays at +/-1 |
| Solver step design | `MaxStep = 1e-6` (was `auto`), `RelTol = 1e-4` (was `1e-3`) |

### 3.2 Before / after

| Metric | v1 | v2 |
|---|---|---|
| `i_a` / `i_c` / `i_a - i_c` | 5.431 / 4.441 / 0.989 A | **5.507 / 4.506 / 1.0012 A** |
| Steady-state error | -1.1 % | **+0.12 %** |
| Controller feedback vs true mean | 5.4996 vs 5.4306 A (+1.27 %) | 5.50000 vs 5.50709 A (-0.13 %) |
| Sampler output mean vs true mean | (no sampler) | **5.50000 vs 5.50709 A (-0.007 A, -0.13 %)** |
| PI steady-state error `err` | about -0.07 A | **about 0** |
| Winding ripple `i_a` | 0.4025 A p-p | 0.4096 A p-p |
| Differential ripple `i_a - i_c` | 0.0612 A p-p | 0.0623 A p-p |
| Spectrum peak | 39 996 Hz (2.48 %) | 39 973 Hz (2.47 %) |
| Duty check | `duty = 0.5209` | `duty = 0.5212`, both-on `= 0.0425 = U` |
| Current slopes | +0.386 / -0.0168 A/us | +0.3858 / -0.0171 A/us |
| Start-up overshoot | about +1.2 % | **+8.8 %** (peak 1.088 A) |
| Small step 1.0 -> 0.5 A | t63 75 us, t90 150 us, no overshoot | t63 70 us, t90 108 us, 8.86 % overshoot, +/-2 % in 0.56 ms |
| Anti-windup activity | not present | **never triggered** (max `|x| = 0.0425` of 1.0) |
| Runtime | 6.9 s per 100 ms | 8.0 s per 20 ms (about 40 s per 100 ms) |

Sampling/PWM delay versus phase margin (analytic, loop = PI x plant x 6 kHz feedback filter,
plus a pure delay):

| Feedback delay | Phase margin |
|---|---|
| 0 us | 69.4 deg |
| 50 us (1 sample) | 42.4 deg |
| 75 us (1.5 samples) | 29.0 deg |
| 100 us (2 samples) | 15.5 deg |

The `Rate Transition` plus `Memory` contributes 1-1.5 samples of delay, which is what turns the
previously overshoot-free response into an 8.8 % overshoot. For topology verification this is
acceptable; for an engineering-grade current loop, either lower `f_c` or remove the extra
`Memory` sample by using a `Zero-Order Hold` as the sampler.

### 3.3 Memory block is still required

Deleting the `Memory` block (keeping the `Rate Transition`) produces exactly one algebraic loop:
`Rate Transition -> Second-Order Filter -> Add2 -> Gain4 -> Add3 -> Saturation -> PWM_gen ->
Solver Configuration/EVAL_KEY -> ... -> Rate Transition`. A `Rate Transition` has direct
feedthrough; use a `Zero-Order Hold` instead if a single-delay sampler is wanted.

### 3.4 Verification outputs (v2, steady state)

```
time-weighted duty(sw1) = 0.5212    both-on = 0.0425 (= U)    both-off = 0.0000
  (the duty estimate carries a +/-1 sample (+/-0.0005) window uncertainty; the bundled
   script prints 0.5215 for its window - both satisfy duty = 0.5 + 0.5*U = 0.5212)
V_avg = 0.0425*198.4 + (1-0.0425)*(-3.0) = 5.55 V   vs   i_a*R = 5.51 V
state sequence (50 us): 24 us freewheel -> 1.04 us energise -> 24 us freewheel -> 1.04 us energise
slopes: energise +0.3858 A/us ; freewheel -0.0171 A/us
i_a spectrum: 39 973 Hz (2.47 %); positive-frequency harmonics at 79 947 Hz (1.23 %) and 119 987 Hz (0.82 %)
ripple: i_a 0.4096 A p-p ; i_a - i_c 0.0623 A p-p
```

---

## 4. Reproduction

Everything below is reproduced by the two scripts bundled with the skill:

| Bundled file | Content |
|---|---|
| `assets/Half_H_Bridge_with_RL_load.slx` | the model as analysed here (md5 `c14d7d1984439a5fa3de7977664b5482`) |
| `assets/parameters_for_AMB_topology.m` | its base-workspace parameters (md5 `a90c6e390a9c1e14bf015396b2d6054d`) |
| `scripts/run_case_study.m` | the two simulations below, ending with a pass/fail regression table against the documented values |
| `scripts/verify_model_facts.m` | the 238 block/parameter checks against the bundled model |

```matlab
skillRoot = '<folder containing SKILL.md>';
run(fullfile(skillRoot,'scripts','run_case_study.m'));      % ~20 s -> "10/10 checks passed"
run(fullfile(skillRoot,'scripts','verify_model_facts.m'));  % ~10 s -> "238 passed, 0 failed"
```

Both scripts resolve their own location, so they run from any working folder; both load
`assets/parameters_for_AMB_topology.m` first (the model resolves its block parameters from the base
workspace), and both simulate an instrumented **copy** in the system temp folder
(`tempdir`), which is deleted afterwards - the bundled model is never modified.

The excerpts below are the parts of those scripts worth reusing.

### 4.1 Locate the assets and instrument a copy

```matlab
here = fileparts(mfilename('fullpath'));                    % relocatable: no absolute paths
modelPath  = fullfile(here,'..','assets','Half_H_Bridge_with_RL_load.slx');
paramsPath = fullfile(fileparts(modelPath),'parameters_for_AMB_topology.m');
loadAmbParams(paramsPath);   % runs the parameter script, copies its vars into the base workspace
tmp = fullfile(tempdir,'amb_half_h_case_study'); dst = 'Half_H_case';
if ~exist(tmp,'dir'), mkdir(tmp); end
copyfile(modelPath, fullfile(tmp,[dst '.slx']), 'f');       % never touch the asset
cd(tmp); load_system(fullfile(tmp,[dst '.slx']));           % slprj / .slxc stay in the temp folder

% log the 10 winding currents through the existing Scope block
set_param([dst '/winding_currents'], 'DataLogging','on', 'DataLoggingVariableName','iw', ...
    'DataLoggingSaveFormat','Structure With Time', ...
    'DataLoggingLimitDataPoints','off','DataLoggingDecimateData','off');

% tap internal signals by branching existing output ports (no rewiring of the original)
add_block('simulink/Sinks/To Workspace',[dst '/TW_Ua1'],'VariableName','Ua1', ...
    'SaveFormat','Structure With Time','MaxDataPoints','inf');
add_line(dst,'PI_ctrl/1','TW_Ua1/1','autorouting','on');
sub = [dst '/PI_ctrl'];
add_block('simulink/Sinks/To Workspace',[sub '/TW_err'],'VariableName','err', ...
    'SaveFormat','Structure With Time','MaxDataPoints','inf');
add_line(sub,'Add2/1','TW_err/1','autorouting','on');
add_block('simulink/Sinks/To Workspace',[sub '/TW_samp'],'VariableName','samp', ...
    'SaveFormat','Structure With Time','MaxDataPoints','inf');
add_line(sub,'Rate Transition/1','TW_samp/1','autorouting','on');

sIn = Simulink.SimulationInput(dst);
sIn = sIn.setModelParameter('StopTime','0.02');
out = sim(sIn);
```

### 4.2 Time-weighted statistics, duty law, and event/slope extraction

```matlab
t = out.iw.time; ia = out.iw.signals(1).values; ic = out.iw.signals(2).values;
s1 = interp1(out.sw1_1.time, double(out.sw1_1.signals.values)/15, t, 'previous');
s2 = interp1(out.sw2_1.time, double(out.sw2_1.signals.values)/15, t, 'previous');
dt = diff(t); w = t(1:end-1) > 0.018; T = sum(dt(w));
tw = @(x) sum(x(w).*dt(w))/sum(dt(w));            % time-weighted mean (never use mean(x))

fprintf('i_a=%.5f i_c=%.5f i_ctrl=%.5f\n', tw(ia(1:end-1)), tw(ic(1:end-1)), tw(ia(1:end-1))-tw(ic(1:end-1)));
d1 = sum(s1(1:end-1).*dt.*w)/T;                   % must equal 0.5 + 0.5*U
bo = sum((s1(1:end-1)>0 & s2(1:end-1)>0).*dt.*w)/T;   % must equal U
fprintf('duty=%.4f both-on=%.4f U_cmd=%.5f\n', d1, bo, tw(out.Ua1.signals.values(1:end-1)));

st = s1*2 + s2; seg = find(t>0.019 & t<0.01915); d = find(diff(st(seg))~=0); prev = seg(1);
for k = 1:numel(d)
  idx = seg(d(k));
  fprintf('t=%.7f state->%d  dur=%.2f us  slope=%+.4f A/us\n', t(idx), st(idx), ...
      (t(idx)-t(prev))*1e6, (ia(idx)-ia(prev))/(t(idx)-t(prev))/1e6);
  prev = idx;
end
```

### 4.3 Feedback bias check (the v1 defect)

```matlab
mem = out.T_memout;                                  % To Workspace tap on the Memory block output
dm = diff(mem.time); wm = mem.time(1:end-1) > 0.018;
fprintf('true winding current = %.5f A\n', sum(ia(find(t>0.018,1):end-1).*diff(t(t>0.018)))/sum(diff(t(t>0.018))));
fprintf('sampled/held value   = %.5f A\n', sum(mem.signals.values(wm).*dm(wm))/sum(dm(wm)));
% v1 (no sampler): the two differ by about +0.07 A (+1.3 %)
% v2 (Rate Transition at SW_Ts): they agree to within about 0.01 A (0.1 %)
```

### 4.4 Algebraic-loop check after removing a block

```matlab
delete_line(sub,'Memory/1','Second-Order Filter/1');
delete_line(sub,'Rate Transition/1','Memory/1');
delete_block([sub '/Memory']);
add_line(sub,'Rate Transition/1','Second-Order Filter/1','autorouting','on');
sim(dst);            % -> warning: the model contains 1 algebraic loop
```

---

## 5. Conclusions

1. **The topology conclusions are independent of the non-ideal effects that were omitted.**
   With ideal switches, no dead time, no magnetics and no motion, the model still reproduces the
   full switching-state behaviour: the duty law `U = 2*duty - 1`, the two `+U_dc` pulses per
   carrier period, the `2*f_sw` ripple, the unidirectional winding current, and the average
   voltage balance against `i*R`.
2. **What actually corrupted the result was simulation mechanics, not physics.** An unsampled
   zero-order-hold feedback under a variable-step solver produced a 1.3 % steady-state error -
   larger than every non-ideal effect that had been dropped. Sampling the feedback at `SW_Ts`
   removed it, leaving a 0.13 % residual.
3. **The cost of the fix is phase margin.** One to 1.5 samples of feedback delay at 20 kHz costs
   roughly 27-40 deg of phase margin for a 1500 Hz loop, turning a 1 % start-up overshoot into
   about 9 %. Acceptable for topology verification; the first thing to re-tune for an
   engineering-grade loop.
4. **Anti-windup is a safety net.** With per-unit gains and a fast R-L plant, the integrator stays
   at 4 % of its clamp in the nominal step-up; the conditional-integration logic never engages.
   Keep it, but do not expect it to shape the normal response.
