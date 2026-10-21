function C = b10_def()
%B10_DEF  10-leg CYCLE-FREE AMB topologies that are NOT a trivial split of the
%   9-leg shape, plus the two trivial-split references.
%
%   Class facts: a forest with 10 driven legs, k undriven nodes and 10 winding
%   edges satisfies  edges = nodes - components  ->  k = components.  This file
%   covers the connected case k = 1: one undriven node, which must be a STAR of
%   degree 4 whose two sides are each a COMPLETE DOF pair (only then do the bias
%   and the deltas cancel there).  Node names start with X = undriven.
%
%   KEY FACT: simply DRIVING a star of the 9-leg shape changes NOTHING - the star
%   potential is already inside the span of the leg potentials, so the bus and the
%   currents are identical (see refs 4 and 5).  A 10-leg design only pays off if
%   its WIRING is different, which is what candidates 1..3 are.
X = {'X','L1','L2','L3','L4','L5','L6','L7','L8','L9','L10'};
C = {};
% 1: minimum bus with B9P10's current profile  (2.0252 phi / 5 A / 7.5 A)
C{end+1} = tp_mk('B10-OPT-B', X, { ...
 'w4','L1','X'; 'w6','L6','L2'; 'w5','L6','L4'; 'w2','L5','L1'; 'w9','L3','L6'; ...
 'w10','L1','L3'; 'w1','L7','L1'; 'w8','X','L8'; 'w7','X','L9'; 'w3','L10','X'}, ...
 '10-leg star + two branching legs; dominates B9P10 + driven star');
% 2: floor/current analogue of B9-OPT-A  (2.0000 phi / 10 A / 15 A)
C{end+1} = tp_mk('B10-OPT-A', X, { ...
 'w4','X','L1'; 'w6','L2','L1'; 'w5','L3','L2'; 'w10','L3','L5'; 'w9','L5','L4'; ...
 'w8','L6','X'; 'w7','L7','X'; 'w1','L7','L8'; 'w3','X','L9'; 'w2','L10','L9'}, ...
 '10-leg; dominates B9-OPT-A + driven star');
% 3: absolute minimum bus found  (1.6751 phi / 15 A / 22.5 A)
C{end+1} = tp_mk('B10-OPT-C', X, { ...
 'w6','X','L1'; 'w3','L2','L1'; 'w1','L3','L1'; 'w2','L5','L3'; 'w8','L5','L4'; ...
 'w7','L4','L7'; 'w4','L6','L7'; 'w10','L8','X'; 'w5','X','L9'; 'w9','L10','X'}, ...
 '10-leg minimum-bus orientation');
% 4,5: trivial splits (drive one star of the 9-leg shape) - references only
A = b9opta_def(); C{end+1} = drive_star(A, 'X1');
B = b9p10_def();  C{end+1} = drive_star(B, 'X1');
end

function T2 = drive_star(T, sname)
E = T.E; nd = T.nodes; k = find(strcmp(nd,sname),1); nd{k} = 'L10';
for q = 1:size(E,1)
  if strcmp(E{q,2},sname), E{q,2} = 'L10'; end
  if strcmp(E{q,3},sname), E{q,3} = 'L10'; end
end
T2 = tp_mk([T.name ' + ' sname ' driven'], nd, E, 'trivial split reference');
end