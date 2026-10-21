function [E, dofOf] = b10optb_netlist(variant)
%B10OPTB_NETLIST  Winding netlist of the single-star-point ten-leg AMB model.
%   E{k,1} = winding name (w1..w10; odd = the "a" winding of a DOF, even = "c")
%   E{k,2} = tail node, E{k,3} = head node: the bias current flows tail -> head.
%   Node names: X = the single undriven star node, L1..L10 = the driven legs.
%
%   PRIMARY TOPOLOGY
%     variant 'w9x1' (default)
%       single-star-point ten-leg topology.  The internal id "w9x1" is kept
%       because it is the name used in the development record; it denotes
%       exactly the same netlist as the canonical topology name.
%       It is the reference B10-OPT-B netlist with ONE change: the L3 end of
%       w9 is re-terminated to L1, i.e. w9 becomes L1 -> L6 instead of
%       L3 -> L6.  Every DOF pair then meets a common node with the same
%       orientation, so all five differential ripples collapse to the
%       R*delta*Ts/(2L) limit while the leg currents stay in the 5 A class.
%       Cost: Udc,min rises from 2.0252 phi (reference B) to 2.3501 phi.
%
%   REFERENCE TOPOLOGIES (kept for comparison only)
%     variant 'B'
%       B10-OPT-B as published: the minimum-bus point of the 5 A leg class
%       (2.0252 phi / 5 A / 7.5 A).  Its DOF5 pair (w9, w10) meets L3 with
%       OPPOSITE orientations, so the DOF5 differential ripple is ~0.5 A
%       instead of ~0.05 A.
%     variant 'd569'  (alias 'ripple')
%       the same wiring as 'B' with the current directions of w5, w6 and w9
%       reversed = direction_569 = -1 in the source record.  All five pairs
%       become clean, but the shared legs carry i_a + i_c = 10 A and the bus
%       rises to 3.0252 phi.  It is dominated by the primary topology.
%
%   HISTORICAL VARIANTS
%     variant 'w4x3'          w4 re-terminated from (L1,X) to (L3,X)
%     variant 'w4x3-ripple'   w4->L3 together with direction_569 = -1
%     variant 'user'          early proposal: w5/w9 swapped plus w6 reversed
%
%   The row order of E is arbitrary.  All external interfaces (Scope, logged
%   channels, controller) are numbered by WINDING, never by row, and the
%   builders map row -> winding with S.wOfRow.
if nargin < 1, variant = 'w9x1'; end
switch lower(char(variant))
  case {'w9x1','w9-l1','w9_to_l1','primary','sstl','single-star-point'}
    % Primary: w9's L3 end re-terminated to L1.
    E = { 'w4','L1','X';  'w6','L6','L2'; 'w5','L6','L4'; 'w2','L5','L1'; 'w9','L1','L6'; ...
          'w10','L1','L3'; 'w1','L7','L1'; 'w8','X','L8';  'w7','X','L9';  'w3','L10','X'};
  case {'b','b10-opt-b','reference-b'}
    % Reference B10-OPT-B: minimum bus in the 5 A class, but DOF5 is the
    % opposite-orientation pair on L3 (0.5 A differential ripple).
    E = { 'w4','L1','X';  'w6','L6','L2'; 'w5','L6','L4'; 'w2','L5','L1'; 'w9','L3','L6'; ...
          'w10','L1','L3'; 'w1','L7','L1'; 'w8','X','L8';  'w7','X','L9';  'w3','L10','X'};
  case {'d569','ripple','d569-1','ripple_priority','reference-d569'}
    % Reference: B10-OPT-B wiring with w5, w6 and w9 current directions
    % reversed (direction_569 = -1).  All pairs are clean, but L3 carries
    % i_a + i_c = 10 A and the bus floor is 3.0252 phi.
    E = { 'w4','L1','X';  'w6','L2','L6'; 'w5','L4','L6'; 'w2','L5','L1'; 'w9','L6','L3'; ...
          'w10','L1','L3'; 'w1','L7','L1'; 'w8','X','L8';  'w7','X','L9';  'w3','L10','X'};
  case {'w4x3','w4_to_l3','move-w4'}
    % Historical: w4 re-terminated from (L1,X) to (L3,X).  Its head stays at
    % the star X, so the star still sees the complete DOF2 and DOF4 pairs;
    % only the driven end moves.  In the ripple-priority mode this shares the
    % DOF5 pair current between L3 and L1 instead of loading L3 alone.
    E = { 'w4','L3','X';  'w6','L6','L2'; 'w5','L6','L4'; 'w2','L5','L1'; 'w9','L3','L6'; ...
          'w10','L1','L3'; 'w1','L7','L1'; 'w8','X','L8';  'w7','X','L9';  'w3','L10','X'};
  case {'w4x3-ripple','w4x3_d569','ripple2'}
    % Historical: w4 -> L3 AND direction_569 = -1.
    E = { 'w4','L3','X';  'w6','L2','L6'; 'w5','L4','L6'; 'w2','L5','L1'; 'w9','L6','L3'; ...
          'w10','L1','L3'; 'w1','L7','L1'; 'w8','X','L8';  'w7','X','L9';  'w3','L10','X'};
  case {'user','proposal','rewire'}
    % Historical: w5/w9 swapped plus w6 reversed.  Dominated by reference B.
    E = { 'w4','L1','X';  'w6','L2','L6'; 'w5','L3','L6'; 'w2','L5','L1'; 'w9','L6','L4'; ...
          'w10','L1','L3'; 'w1','L7','L1'; 'w8','X','L8';  'w7','X','L9';  'w3','L10','X'};
  otherwise
    error('b10optb:variant','unknown netlist variant "%s"', char(variant));
end
dofOf = zeros(1,10);
for k = 1:10, dofOf(k) = ceil(sscanf(E{k,1},'w%d')/2); end
end
