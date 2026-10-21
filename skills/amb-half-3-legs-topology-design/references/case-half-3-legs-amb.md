# Case study: 5-DOF AMB half 3-legs topology and current loop

Reference model: `Half_3_Legs_with_RL_load.slx` with `parameters_for_AMB_topology.m`.
Purpose of the model: **topology verification only** - prove that a half 3-legs cell
(3 legs driving 2 windings) can regulate the differential (control) current to a set reference
while keeping every winding current unidirectional. All numbers below were measured on this
model; none are estimates.

> **Naming note.** The model's block names (`tbt_3bridge_module`, `btb_3bridge_module`,
> `PWM_tbt_3bridge_gen`, ...) are historical identifiers. The `3bridge` part denotes the
> three-leg cell, not a complete bridge.

## 1. Model inventory

| Item | Value |
|---|---|
| Blocks / subsystems | 1390 / 177 |
| Cells | 5 (3 TBT + 2 BTB) |
| Windings | 10 (8 radial + 2 axial = 5 DOF) |
| Controlled switches | 15 x `IGBT (Ideal, Switching)` |
| Freewheel diodes | 15 x `Diode` |
| Bus | one ideal 200 V `Voltage Source`, all cells daisy-chained |
| Controllers | 10 x `PI_ctrl` (one per winding) |
| Modulators | 5 (2 x `PWM_btb_3bridge_gen`, 3 x `PWM_tbt_3bridge_gen`) |
| Observation | one 10-input `Scope` (`winding_currents`) |
| Solver | ode45, `MaxStep = 1e-6`, `RelTol = 1e-4`, accelerator mode |

Parameters: `U_dc = 200 V`, `R = 1 ohm`, `L = 500 uH`, `f_sw = 20 kHz`
(`Ts = 50 us`), `bias_current = 5 A`, `f_c = 1500 Hz`, `K_p = 0.023562`,
`K_i = 47.1239`, constant `i_ref = 1 A` for all five channels.

## 2. The two cell types

Reading the Simscape netlist out of the model gives the device placement directly. Writing
P / N for the bus rails and M1 / A1 / C1 for the three leg midpoints:

- **`tbt_3bridge_module` (TBT)** - leg 1 upper switch `sw1` (P to M1), leg 2 lower switch
  `sw2` (A1 to N), leg 3 upper switch `sw3` (P to C1); freewheel diodes N->M1, A1->P,
  N->C1; winding a from M1 to A1, winding c from A1 to C1. **Shared node A1**; both winding
  currents flow into it.
- **`btb_3bridge_module` (BTB)** - leg 1 upper switch `sw2` (P to M1), leg 2 lower switch
  `sw1` (A1 to N), leg 3 lower switch `sw3` (C1 to N); freewheel diodes N->M1, A1->P,
  C1->P; winding a from A1 to M1, winding c from M1 to C1. **Shared node M1**; both winding
  currents flow out of it.

The two are exact duals. The BTB is drawn with its shared T leg first (so the diagram reads
T-B-B); relabelling leg 1 and leg 2 makes it the conventional B-T-B. The current sensors are
oriented so that both cells report positive current from the T leg to the B leg, so `i_a` and
`i_c` mean the same physical direction in both.

Consequences verified on the model:

- winding current direction is M1 -> A1 (winding a) and T-leg -> B-leg (winding c) in **all four
  switch states**, so the current is unidirectional by construction;
- no leg can connect P to N (both devices in a leg point the same way), so **no dead time is
  needed**;
- the shared leg carries `i_a + i_c` continuously.

## 3. Modulation: derivation and measurement

Carrier: `Repeating Sequence` with `rep_seq_t = [0 half_SW_Ts SW_Ts]` and
`rep_seq_y = [0 1 0]` - a symmetric 0..1 triangle of period 50 us. Comparators use `>=`, so
for thresholds inside [0, 1] the comparator duty **equals the threshold**.

**TBT generator.** With `e1 = u_a - u_c`, `e2 = -u_c`, `e3 = 0` and
`m = 0.5*(max(e1,e2,e3) + min(e1,e2,e3))`, the thresholds are `0.5 - m + e1`,
`0.5 - m + e2`, `0.5 - m`. The gate generator sends `sw1 = b1`, `sw2 = NOT b2`,
`sw3 = b3`, giving `d1 = 0.5 - m + u_a - u_c`, `d2 = 0.5 + m + u_c`,
`d3 = 0.5 - m`.

**BTB generator.** With `e1' = u_c - u_a`, `e2' = u_c`, `m' = -m`, the gate generator sends
`sw1 = NOT b1`, `sw2 = b2`, `sw3 = NOT b3`, and produces exactly the same duty triple.

Both satisfy `d1 + d2 - 1 = u_a` and `d3 + d2 - 1 = u_c`, consistent with the identical
winding voltage law `V_a = U_dc(s1+s2-1)`, `V_c = U_dc(s3+s2-1)`.

Measured (time-weighted, 30-50 ms), all five cells:

```
m1 TBT  u_a = +0.04569 -> d1+d2-1 = +0.04569 | u_c = +0.04000 -> d3+d2-1 = +0.04000
m2 BTB  u_a = +0.04568 -> d1+d2-1 = +0.04568 | u_c = +0.03999 -> d3+d2-1 = +0.03999
m3 TBT  ... identical to m1
m4 BTB  ... identical to m2
m5 TBT  ... identical to m1
d = (0.5228, 0.5228, 0.5172) in every cell
```

This is the empirical confirmation that the two arrangements need different gate polarity but
produce the same duties.

For `u_a >= u_c >= 0` the min-max injection collapses to
`d(shared leg) = d(larger winding's leg) = 0.5 + u_a/2` and
`d(other leg) = 0.5 + u_c - u_a/2`. Substituting the measured commands gives
`d1 = d2 = 0.522845` and `d3 = 0.517155` - matching the measured 0.5228 / 0.5228 / 0.5172
to 1e-4.

## 4. Steady state and switching states

Time-weighted averages over 30-50 ms:

| Signal | Reference | TBT cells (m1/m3/m5) | BTB cells (m2/m4) |
|---|---|---|---|
| `i_a` | 5.5000 A | 5.4960 A (-0.073 %) | 5.4947 A (-0.097 %) |
| `i_c` | 4.5000 A | 4.4968 A (-0.071 %) | 4.4954 A (-0.103 %) |
| `i_a - i_c` | 1.0000 A | 0.9992 A | 0.9993 A |
| winding ripple | - | 0.4727 A p-p (8.6 %) | 0.4726 A p-p |
| control-current ripple | - | 0.0620 A p-p | 0.0620 A p-p |

The residual difference between TBT and BTB (about 0.03 percentage points) comes from the
different freewheel device combinations, which place the feedback sample at a slightly different
ripple phase. It is two orders of magnitude below the 0.5 % acceptance threshold.

State sequence of cell 1 (TBT) over two carrier periods starting at 45 ms, from the logged gate
signals:

```
45000.00 us  (s1,s2,s3)=(1,0,1)   11.93 us     (start of window)
45011.93 us  (s1,s2,s3)=(1,1,1)    1.00 us
45012.93 us  (s1,s2,s3)=(1,1,0)    0.14 us
45013.07 us  (s1,s2,s3)=(0,1,0)   23.86 us
45036.93 us  (s1,s2,s3)=(1,1,0)    0.14 us
45037.07 us  (s1,s2,s3)=(1,1,1)    1.00 us
45038.07 us  (s1,s2,s3)=(1,0,1)   23.86 us
... repeating ...
```

Per 50 us carrier period: two 23.86 us "both freewheel" intervals, two 1.00 us "both energised"
pulses and two 0.14 us "winding a only" pulses. Checks:

- winding a "both switches on" fraction = `2*(1.00+0.14)/50 = 4.569 %` = `u_a` exactly;
- winding c "both switches on" fraction = `2*1.00/50 = 4.000 %` = `u_c` exactly;
- two energise pulses per carrier period, hence the ripple fundamental at `2*f_sw`.

Measured slopes: energise `+0.3860 A/us`, freewheel through the lower switches
`-0.0172 A/us`, freewheel through the shared upper diode `-0.0200 A/us`.

## 5. Device-drop evidence chain for the shared-leg current

The model uses non-ideal devices (`Diode: Ron = 0.3, Vf = 0.6`;
`IGBT: Ron = 0.001, Vf = 0.8`), which turns the freewheel states into small negative voltages
and makes the shared-leg current observable through the current slopes.

| State | Measured `di/dt` | Implied winding voltage (`L*di/dt + i*R`) | Predicted from the drops |
|---|---|---|---|
| energise `(1,1,*)` | +0.3860 A/us | 198.5 V | `200 - 2*(0.8 + 5.5*0.001) = 198.39 V` |
| freewheel `(0,1,0)` | -0.0172 A/us | -3.10 V | `-[0.6 + 5.5*0.3] - [0.8 + 10*0.001] = -3.06 V` |
| freewheel `(1,0,1)` | -0.0200 A/us | -4.50 V | `-[0.8 + 5.5*0.001] - [0.6 + 10*0.3] = -4.41 V` |

The third row is the key: the shared upper diode carries `i_a + i_c = 9.99 A`, so its drop is
`0.6 + 10*0.3 = 3.6 V`, versus 2.25 V for an outer diode carrying only 5.5 A. If the shared leg
carried a single winding current, both freewheel states would have the same drop (about 3.06 V)
and both slopes would be about `-0.017 A/us`; the measured -0.0200 A/us can only be explained
by the summed current.

Because of these drops the required duty is 0.5228 / 0.5228 / 0.5172 instead of the
ideal-device 0.51375 / 0.51375 / 0.50875 - about +0.9 percentage points. That is a device
property, not a topology property; the minimal model of the skill uses ideal switches and reports
the ideal duties.

## 6. Device current stress (per cell, from measured duties and KCL)

Using `i_a = 5.496 A`, `i_c = 4.497 A`, `i_a + i_c = 9.993 A`:

| Device | Position | Duty | Current carried | Average |
|---|---|---|---|---|
| `sw2` | shared leg switch (TBT) | 0.5228 | `i_a + i_c` | **5.22 A** |
| shared diode | shared leg diode (TBT) | 0.4772 | `i_a + i_c` | **4.77 A** |
| `sw1` | outer T leg switch | 0.5228 | `i_a` | 2.87 A |
| outer diode 1 | outer T leg diode | 0.4772 | `i_a` | 2.62 A |
| `sw3` | outer T leg switch | 0.5172 | `i_c` | 2.33 A |
| outer diode 2 | outer T leg diode | 0.4828 | `i_c` | 2.17 A |

The shared leg carries about twice the current of an outer leg, so its conduction loss is about
four times higher. This is the design price of driving two windings with three legs instead of
four (25 % fewer devices), and it is the number to carry into device rating and thermal work.

## 7. Feedback sampling: A/B comparison

Two variants of the same model, identical except for the feedback path into each `PI_ctrl`:

| Feedback path | `i_a1` error | `i_c1` error | `i_a - i_c` |
|---|---|---|---|
| `Rate Transition (Ts = 50 us)` -> `Memory` -> 2nd-order LPF (the model as built) | -0.073 % | -0.071 % | 0.9992 A |
| `Memory` -> 2nd-order LPF only (sampler removed) | -0.425 % | -0.444 % | 0.9966 A (-0.34 %) |

Removing the synchronous sampler makes the steady-state error about six times worse, because the
`Memory` block zero-order-holds the value from the previous major step and the variable-step
solver legitimately takes large steps where the current ramps slowly. For an AMB, whose force
gain is proportional to current, a 0.4 % bias-current error is a 0.4 % force-gain error - worth
avoiding even in a verification model.

## 8. Solver behaviour

| Metric | Value (100 ms run) |
|---|---|
| Steps | 350 775 |
| Smallest step | 5.67e-30 s |
| Fraction of steps < 1 ns | 35.3 % |
| Wall clock | 15.8 s (accelerator mode) |
| Warnings | none; no algebraic loop |

`MaxStep = 1e-6` is only 1 us while the narrowest energise pulse is 0.14 us. The pulse width is
still correct because the **zero-crossing detection** of the variable-step solver brackets the
comparator crossings; the sub-nanosecond steps are that mechanism at work. Switching to a 1 us
Simscape local solver would quantise the 0.14 us pulse to 0 or 1 us, i.e. about +/-1 % duty error
(roughly +/-50 mA) instead of the 4 mA achieved here.

## 9. Reproduction snippets

The reference model and its parameter script ship with this skill in `model/`; `cd` there
first. Built and verified with **MATLAB R2025b (25.2), Simulink 25.2, Simscape 25.2** on
Linux (GLNXA64).

Simulate and log the ten winding currents without editing the saved model (mark the **source
port**; a port handle must be an output port for `DataLogging` to be accepted):

```matlab
cd('<this skill folder>/model')                 % bundled .slx + .m live here
run('parameters_for_AMB_topology.m');           % first: it starts with clearvars
m = 'Half_3_Legs_with_RL_load'; load_system(m);
gotoBlk = {'Goto30','Goto31','Goto2','Goto3','Goto6','Goto7','Goto4','Goto5','Goto8','Goto9'};
names   = {'i_a1','i_c1','i_a2','i_c2','i_a3','i_c3','i_a4','i_c4','i_a5','i_c5'};
for i = 1:10
    ph = get_param([m '/' gotoBlk{i}],'PortHandles');
    sp = get_param(get_param(ph.Inport(1),'Line'),'SrcPortHandle');
    set_param(sp,'DataLogging','on','DataLoggingNameMode','Custom', ...
                'DataLoggingName',['z_' names{i}]);
end
set_param(m,'SignalLogging','on','SignalLoggingName','logsout');
out = sim(m,'StopTime','0.05');
```

Time-weighted steady state and control current:

```matlab
T = out.tout; ls = out.logsout;
gv = @(n) squeeze(ls.getElement(['z_' n]).Values.Data);
idx = find(T >= 0.03); t = T(idx); dt = diff(t);
mw = @(x) sum(x(2:end).*dt)/sum(dt);
ia = gv('i_a1'); ic = gv('i_c1');
fprintf('i_a = %.4f  i_c = %.4f  i_a-i_c = %.4f\n', ...
        mw(ia(idx)), mw(ic(idx)), mw(ia(idx)-ic(idx)));
```

State sequence and slopes:

```matlab
s1 = gv('sw1_m1')/15; s2 = gv('sw2_m1')/15; s3 = gv('sw3_m1')/15;
st = round(s1(idx)*4 + s2(idx)*2 + s3(idx));
for k = 0:7
    ii = find(st(2:end)==k & st(1:end-1)==k);
    if numel(ii) > 50
        d = (ia(idx(ii+1))-ia(idx(ii)))./(t(ii+1)-t(ii));
        fprintf('state (%d,%d,%d): %+8.4f A/us\n', ...
            bitand(k,4)>0, bitand(k,2)>0, bitand(k,1)>0, mean(d)/1e6);
    end
end
```

Ripple spectrum (uniform resample first - the solver is variable-step):

```matlab
ts = (0.030:0.5e-6:0.050)';
xu = interp1(T, ia, ts, 'linear'); xu = xu - mean(xu);
N = numel(xu); fs = 1/0.5e-6;
X = abs(fft(xu.*hann(N)))*2/N; f = (0:N-1)*fs/N;
[pv, ix] = maxk(X(2:floor(N/2)), 3);
fprintf('%.0f Hz (%.2f x fsw): %.4f A\n', [f(ix+1) f(ix+1)/20e3 pv]');
```

## 10. What was deliberately not required

Recorded here because it is the point of the skill, not because it is missing:

- **A dynamic `i_ref` profile.** The constant 1 A reference is the intended stimulus: it proves
  the topology can hold the control current at a commanded value. Step/chirp/sweep testing is a
  controller-bandwidth exercise and belongs to a later, engineering-grade model.
- **A separate `i_a - i_c` signal in the model.** The control current is a two-signal
  difference; forming it in post-processing (as above) recovers it exactly. Adding a subtraction
  block is a readability choice, not a verification requirement.
- **Centralised engineering parameters.** The feedback-filter corner and damping (6000 Hz /
  0.707), the gate level (15 V), the controller and integrator clamps (+/-1), the reference gain
  (0.5) and the modulator constants (0.5 / 0) are hard-coded inside the block dialogs. They set
  loop bandwidth, drive level and protection boundaries; they do not change the switching states,
  the current direction or the modulation law, which are what this model verifies.
- **Motion, force, magnetics, bus dynamics, thermal, protection.** Out of scope by design.

## 11. Conclusions

1. Both cell types implement "3 legs driving 2 windings" correctly. Every winding current stays
   unidirectional (minimum 0 at t = 0, no zero crossing) in all four switch states.
2. TBT and BTB are exact duals with identical winding voltage laws; their modulators produce
   **bit-identical duty triples** (0.5228, 0.5228, 0.5172 in all five cells).
3. The modulation law holds to better than 1e-5, the "both-on" fraction equals `u_a` / `u_c`
   exactly, and the ripple fundamental sits exactly at `2*f_sw`.
4. The control current tracks the constant reference to within 0.1 % (0.9992 / 0.9993 A against
   1.0 A), which is the acceptance criterion for a topology-verification model.
5. The shared leg carries `i_a + i_c`, confirmed quantitatively by the measured current slopes;
   its devices see roughly twice the current and four times the conduction loss of an outer leg.
6. Synchronous feedback sampling is the one simulation mechanic that must not be dropped: it
   improves the steady-state error about six-fold (0.073 % vs 0.425 %).
