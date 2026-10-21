function T = b9p10_def()
%B9P10_DEF  Authoritative netlist of B9P10 (bridge 9 phase 10), relabelled to
%   the SAME node/pair convention as B9-OPT-A so that the two differ visually
%   and electrically in exactly one place: the polarity of the DOF1 pair on the
%   two leg-to-leg (middle) edges (here a U-turn: L5->L4 and L6->L5).
%   Metrics are identical to the sibling study's B9P10 (87.1239 V = 3.050 phi).
T = tp_mk('B9P10', {'L1','L2','L3','L4','L5','L6','L7','L8','L9','X1','X2'}, { ...
 'w1','L5','L4' ; 'w2','L6','L5' ; 'w3','L4','X1' ; 'w4','L1','X1' ; 'w5','X1','L2' ; ...
 'w6','X1','L3' ; 'w7','L7','X2' ; 'w8','L8','X2' ; 'w9','X2','L6' ; 'w10','X2','L9' }, ...
 'B9P10: same shape, middle pair in the opposite (U-turn) polarity');
end