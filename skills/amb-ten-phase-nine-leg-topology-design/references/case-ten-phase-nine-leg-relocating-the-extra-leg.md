# Case study: relocating the extra leg of the ten-phase-nine-leg topology

Companion to `case-ten-phase-nine-leg-amb.md` and `case-ten-phase-nine-leg-modulation-boundary.md`.

The engineer's note that motivated this study: *the leg introduced for the remaining two windings is connected
at one particular position, but it could equally be connected at other leg midpoints; only the leg
voltage calculation has to change, and it is still the same topology.* This reference verifies that claim and
quantifies what the choice does change.

Models: `model/Ten_phase_nine_leg_with_RL_load.slx` (baseline), `model/Ten_phase_nine_leg_relocated_L3_L7.slx`
(variant A) and `model/Ten_phase_nine_leg_relocated_L2_L8.slx` (variant B). Script: `scripts/run_case_study.m`,
mode `relocation`.

Environment: MATLAB R2025b (25.2.0.3312555 Update 6), Simulink 25.2, Simscape 25.2, Simscape Electrical
25.2, Linux glnxa64.

---

## 1. Why any midpoint works

The axial pair plus its dedicated leg form a series string `[w_a5 - L_new - w_c5]`. Attaching its two ends to
two leg midpoints adds two edges and one node to the winding/leg graph. The graph stays a **connected tree**
as long as the two midpoints lie in different star groups (or, more generally, as long as no cycle is
created). A tree of 11 nodes always has exactly 10 edges, i.e. the nine legs still determine the ten winding
voltages up to one common mode - so the current loop keeps the same number of degrees of freedom.

What changes is only the **node-voltage back-substitution**: the nine lines of
`winding_voltage_to_bridge` are re-derived from the new edge list. Nothing else in the model - references,
PI controllers, min-max injection, PWM, legs, windings - is touched.

## 2. The two verified relocations

**Variant A - axial pair between legs 3 and 7.** Rewire `w5.L: leg4 -> leg3` and `w6.R: leg6 -> leg7`:

```

ubridge(5) = 0;
ubridge(3) = ubridge(5) + uw5;          ubridge(7) = ubridge(5) - uw6;
ubridge(1) = ubridge(3) + uw3 + uw1;    ubridge(2) = ubridge(3) + uw3 + uw2;
ubridge(4) = ubridge(3) + uw3 - uw4;
ubridge(6) = ubridge(7) - uw8 + uw7;    ubridge(8) = ubridge(7) - uw8 - uw9;
ubridge(9) = ubridge(7) - uw8 - uw10;
```

**Variant B - axial pair between legs 2 and 8.** Rewire `w5.L: leg4 -> leg2` and `w6.R: leg6 -> leg8`:

```

ubridge(5) = 0;
ubridge(2) = ubridge(5) + uw5;          ubridge(8) = ubridge(5) - uw6;
ubridge(1) = ubridge(2) - uw2 + uw1;    ubridge(3) = ubridge(2) - uw2 - uw3;
ubridge(4) = ubridge(2) - uw2 - uw4;
ubridge(6) = ubridge(8) + uw9 + uw7;    ubridge(7) = ubridge(8) + uw9 + uw8;
ubridge(9) = ubridge(8) + uw9 - uw10;
```

Both are exact "walk the tree" derivations, exactly like the baseline.

## 3. Leg-current expressions for each placement

The leg currents follow from KCL at each node. With `I` the ten winding currents in the model's order
(`i_a1 i_c1 i_a2 i_c2 i_a3 i_c3 i_a4 i_c4 i_a5 i_c5`):

| Leg | baseline | variant A | variant B |
|---|---|---|---|
| L1 | `i_a1` | `i_a1` | `i_a1` |
| L2 | `i_c1` | `i_c1` | `i_c1 + i_a3` |
| L3 | `-i_a2` | `-i_a2 + i_a3` | `-i_a2` |
| L4 | `i_a3 - i_c2` | `-i_c2` | `-i_c2` |
| L5 | `i_c3 - i_a3` | `i_c3 - i_a3` | `i_c3 - i_a3` |
| L6 | `i_a4 - i_c3` | `i_a4` | `i_a4` |
| L7 | `i_c4` | `i_c4 - i_c3` | `i_c4` |
| L8 | `-i_a5` | `-i_a5` | `-i_a5 - i_c3` |
| L9 | `-i_c5` | `-i_c5` | `-i_c5` |

In all three cases `sum(I_leg) = 0` identically, using only the two star-node identities: the bias never
reaches the bus, whatever the placement. The scripts verify this by checking Tellegen's identity as well.

## 4. Measured comparison

Same references, same parameters, same solver settings, 20 ms, metrics over the last 2 ms:

| | baseline (4-5-6) | variant A (3-5-7) | variant B (2-5-8) |
|---|---|---|---|
| leg currents [A] | 5.49, 4.50, -5.49, **1.00, -1.00, 1.00**, 4.49, -5.49, -4.50 | 5.51, 4.51, **-0.11, -4.48, -0.91, 5.50, -0.06**, -5.49, -4.49 | 5.49, **9.99**, -5.49, -4.49, **-1.00**, 5.49, 4.49, **-9.99**, -4.50 |
| control currents [A] | 0.9992, 0.9970, 0.9965, 0.9993, 0.9988 | 0.9992, **1.0603, 0.9057, 1.0303**, 0.9988 | 0.9995, 0.9989, 0.9985, 0.9991, 0.9983 |
| sum of leg currents [A] | 9.8e-15 | 1.2e-14 | 1.2e-14 |
| max abs leg current | 5.9 A | 5.8 A | **11.3 A** |
| sum abs leg currents | 33.0 A | 31.1 A | **50.9 A** |
| duty range | 0.29..0.71 | 0.29..0.71 | 0.31..0.69 |

All three models run and all three regulate their control currents. The relocation claim is therefore
**verified**: it is the same topology, and only the back-substitution changes.

## 5. Criterion 1 - bias cancellation decides the device rating

The series string always drives its outer windings in a fixed direction: current flows **out of** the
X1-side leg and **into** the X2-side leg. The co-located radial winding either flows the same way or the
opposite way, and only the opposite orientation cancels the bias.

| Placement | X1-side leg | co-located radial | leg current | bias? |
|---|---|---|---|---|
| baseline | L4 | `w4 = c2`, star -> leg | `(d3 + d2)/2` | no |
| variant A | L3 | `w3 = a2`, star -> leg | `(d3 - d2)/2` | no |
| variant B | L2 | `w2 = c1`, leg -> star | `i_c1 + i_a3` | **yes, 2*i_bias** |
| any X2-side choice | L8 or L9 | `w9` or `w10`, star -> leg | `-i_a5 - i_c3` | **yes, 2*i_bias** |

Variant B puts +9.99 A and -9.99 A on legs 2 and 8: the devices there must be rated for twice the bias, and
the converter's circulating current rises from 33 A to 51 A (sum of absolute leg currents). Nothing about the
topology breaks - the currents still sum to zero and the control still works - but the silicon is not the
same size.

## 6. Criterion 2 - the dead zone decides the accuracy

Among the bias-free choices, the shared leg current is either the **sum** or the **difference** of the two
adjacent control currents:

```

baseline   leg4 = (d2 + d3)/2      leg5 = -d3      leg6 = (d3 + d4)/2
variant A  leg3 = (d3 - d2)/2                      leg7 = (d3 - d4)/2
```

At the verified operating point all five control currents equal +1 A, so variant A's shared legs carry
`(1-1)/2 = 0`: they sit exactly in the dead zone. The measured consequence is a 6..9 % steady-state error on
DOF2, DOF3 and DOF4 (1.0603, 0.9057, 1.0303 A instead of 1 A). Removing the dead time
(`Td = 0`) collapses those errors back to 0.1..0.7 %, which proves the mechanism.

Two caveats keep this criterion honest:

- The "sum" form is not universally better: it vanishes when the two adjacent control currents are equal and
  opposite. What the placement really chooses is *where* the zero crossing of the shared-leg current sits.
- The third control current `delta3` of the worst-case distribution is free as far as the modulation spread is
  concerned (see the boundary reference), so a relocation can often be combined with a different `delta3` to
  move the leg current off zero.

Among the four bias-free placements - X1 side in {L3, L4} x X2 side in {L6, L7} - the baseline (L4, L6) is
the **only** one whose two shared legs both carry the *sum* of adjacent control currents, and it is therefore
the only one that is simultaneously bias-free and away from the dead zone at the verified operating point.

## 7. Practical recipe for a new placement

1. Choose the two midpoints. Keep them in different star groups so the graph stays a connected tree.
2. Rewire the two winding terminals (`w5.L` and `w6.R`) in the Simscape diagram.
3. Rewrite the nine lines of `winding_voltage_to_bridge` by walking the new tree from a common-mode reference
   node. Nothing else changes.
4. Check criterion 1 (which legs now carry `2*i_bias`) and resize the devices if needed.
5. Check criterion 2 (is any shared leg current a difference of two control currents that can be equal?),
   and prefer a placement - or a free `delta3` - that keeps every leg current away from zero.
6. Re-run `scripts/run_case_study.m` in mode `relocation` after updating its expectations, or simply repeat
   the baseline case study on the new model.
