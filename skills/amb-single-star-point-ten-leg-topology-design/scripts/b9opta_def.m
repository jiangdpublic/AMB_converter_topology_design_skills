function T = b9opta_def()
%B9OPTA_DEF  Authoritative netlist of B9-OPT-A.
%
%   9 driven half-bridge legs (L1..L9) + 2 UNDRIVEN star junctions (X1, X2),
%   10 windings, cycle-free (the winding graph is a TREE -> zero static bias
%   error under winding R/L mismatch).
%
%   Windings w1..w10 ; odd w = a winding, even w = c winding;
%   DOF pairs: (w1,w2)=DOF1  (w3,w4)=DOF2  (w5,w6)=DOF3  (w7,w8)=DOF4
%              (w9,w10)=DOF5 (axial pair).
%
%   E{k,:} = {winding, tail, head}; current positive from tail to head and
%   u_k = phi(tail) - phi(head) = R_k i_k + L_k di_k/dt.
%
%   Shape (identical to B9P10 -- it is the ONLY cycle-free 9-leg shape):
%     star X1: in-pair DOF2 (w3,w4 enter X1) , out-pair DOF3 (w5,w6 leave X1)
%     star X2: in-pair DOF4 (w7,w8 enter X2) , out-pair DOF5 (w9,w10 leave X2)
%     path  : X1 -w3- L4 -w1- L5 -w2- L6 -w9- X2 , plus 3 leaf legs per star
%   The ONLY difference from B9P10 is the polarity of the pair on the two
%   leg-to-leg (middle) edges: here w1 and w2 traverse the path in the SAME
%   direction.  That puts 2*i_b = 10 A on L4/L6 but needs only 2.3501 phi.
T = tp_mk('B9-OPT-A', {'L1','L2','L3','L4','L5','L6','L7','L8','L9','X1','X2'}, { ...
 'w1','L4','L5' ; 'w2','L5','L6' ; 'w3','L4','X1' ; 'w4','L1','X1' ; 'w5','X1','L2' ; ...
 'w6','X1','L3' ; 'w7','L7','X2' ; 'w8','L8','X2' ; 'w9','X2','L6' ; 'w10','X2','L9' }, ...
 'B9-OPT-A: minimum-bus 9-leg cycle-free star-tapped topology');
end