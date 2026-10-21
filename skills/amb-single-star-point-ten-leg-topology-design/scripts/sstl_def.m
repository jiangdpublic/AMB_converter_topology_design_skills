function T = sstl_def()
%SSTL_DEF  Descriptor of the single-star-point ten-leg topology (primary).
%   The internal id "w9x1" is kept for continuity with the development record;
%   it denotes exactly the same netlist as the canonical topology name.
E = b10optb_netlist('w9x1');
nd = {'X','L1','L2','L3','L4','L5','L6','L7','L8','L9','L10'};
T = tp_mk('single-star-point ten-leg (w9x1)', nd, E, 'primary topology');
end
