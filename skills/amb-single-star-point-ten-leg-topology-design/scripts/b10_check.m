function b10_check()
%B10_CHECK  10-leg cycle-free topologies: the primary single-star-point ten-leg
%   topology, the three Pareto points of the 10-leg class, and the "just drive
%   one star of the 9-leg shape" references (which are dominated).
%   Prints the criteria vector, the leg currents, the node degrees and the
%   dominance verdict.  U_dc,min is computed twice (analytic spanning-tree
%   recursion b9_floor_fast + LP tp_floor_leg) and the two must agree.
par = tp_par();
nodes = {'X','L1','L2','L3','L4','L5','L6','L7','L8','L9','L10'};
isLeg = logical([0 1 1 1 1 1 1 1 1 1 1]);

% ---- candidates ---------------------------------------------------------
% 1: primary topology (all five pairs clean, 2.3501 phi)
% 2: reference B10-OPT-B (minimum bus in the 5 A class, 2.0252 phi)
% 3/4: the 10 A and 15 A Pareto points, for context
C = {};
C{end+1} = sstl_def();
C{end+1} = mk('B10-OPT-B (reference)', b10optb_netlist('B'));
C{end+1} = mk('B10-OPT-A (10 A class)', {'w4','X','L1';'w6','L2','L1';'w5','L3','L2';'w10','L3','L5'; ...
  'w9','L5','L4';'w8','L6','X';'w7','L7','X';'w1','L7','L8';'w3','X','L9';'w2','L10','L9'});
C{end+1} = mk('B10-OPT-C (15 A class)', {'w6','X','L1';'w3','L2','L1';'w1','L3','L1';'w2','L5','L3'; ...
  'w8','L5','L4';'w7','L4','L7';'w4','L6','L7';'w10','L8','X';'w5','X','L9';'w9','L10','X'});
% ---- references: a 9-leg shape with one of its star nodes simply driven ----
A = b9opta_def(); R{1} = drive_star(A, 'X1', 'B9-OPT-A + X1 driven');
B = b9p10_def();  R{2} = drive_star(B, 'X1', 'B9P10   + X1 driven');

fprintf('=== 10-leg cycle-free: primary, class Pareto points and split references ===\n');
fprintf('%-32s %4s %4s %10s %9s %9s %8s\n','topology','legs','cyc','floor/phi','Udc[V]','Idc_bias','Idc_env');
[cv, ci] = show(C, nodes, par);
fprintf('--- references (constructed by driving one star of the 9-leg shape) ---\n');
[rv, ri] = show(R, nodes, par);
for i = 1:numel(R)
  d = dominates(cv, ci, rv(i), ri(i,:));
  if isempty(d)
    fprintf('   %-24s : no same-current-class candidate dominates it\n', R{i}.name);
  else
    fprintf('   %-24s : strictly dominated by %s (same %.2f/%.2f A class, bus %.4f > %.4f phi)\n', ...
            R{i}.name, C{d}.name, ri(i,1), ri(i,2), rv(i), cv(d));
  end
end
fprintf(['\nNote: the primary topology is NOT the minimum-bus point of the 5 A class;\n' ...
         '      reference B10-OPT-B is (2.0252 vs 2.3501 phi).  The primary is chosen\n' ...
         '      because all five differential ripples are at the 0.05 A limit, whereas\n' ...
         '      reference B has 0.5 A on DOF5.  See references/03.\n']);
end

% ------------------------------------------------------------------ helpers
function [v, i] = show(L, nodes, par)
v = zeros(1,numel(L)); i = zeros(numel(L),2);
for q = 1:numel(L)
  T = L{q};  isLegT = b9_isleg(T);
  [Uf, info] = b9_floor_fast(T.nodes, T.E, par, isLegT);
  Ul = tp_floor_leg(T.nodes, T.E, par, isLegT);
  A = info.A;  Ib = max(abs(A'*(par.i_bias*ones(10,1))));  Ie = 0;  kcl = 0;
  for s = 0:31
    b = dec2bin(s,5); sg = 2*(b-'0')-1; iw = zeros(10,1);
    for j = 1:5, iw(2*j-1)=par.i_bias+sg(j)*par.delta_max/2; iw(2*j)=par.i_bias-sg(j)*par.delta_max/2; end
    Ii = A'*iw(info.widx);
    Ie = max(Ie, max(abs(Ii(isLegT)))); kcl = max(kcl, max(abs(Ii(~isLegT))));
  end
  v(q) = Uf/par.phi; i(q,:) = [Ib Ie];
  fprintf('%-32s %4d %4d %10.4f %9.4f %9.2f %8.2f\n', T.name, numel(T.nodes)-1, size(info.Z,2), v(q), Uf, Ib, Ie);
  if abs(Uf-Ul) > 1e-6, fprintf('   !! LP mismatch: recursion %.4f vs LP %.4f\n', Uf, Ul); end
  if kcl > 1e-9,        fprintf('   !! star KCL residual %.2e  -> topology is not realisable\n', kcl); end
  fprintf('   bias leg currents [A]: '); fprintf('%+.1f ', (A'*(par.i_bias*ones(10,1)))'); fprintf('\n');
  deg = zeros(1,numel(T.nodes));
  for k = 1:size(T.E,1)
    a = find(strcmp(T.nodes,T.E{k,2})); b = find(strcmp(T.nodes,T.E{k,3}));
    deg(a)=deg(a)+1; deg(b)=deg(b)+1;
  end
  fprintf('   node degrees: '); fprintf('%d ', deg); fprintf('\n');
end
end

function d = dominates(cv, ci, rv, ri)
%DOMINATES  Index of the best candidate with the same leg-current rating and a lower bus.
d = []; best = inf;
for q = 1:numel(cv)
  if abs(ci(q,1)-ri(1)) < 1e-9 && abs(ci(q,2)-ri(2)) < 1e-9 && cv(q) < rv - 1e-9
    if cv(q) < best, best = cv(q); d = q; end
  end
end
end

function T = mk(name, E)
T = tp_mk(name, {'X','L1','L2','L3','L4','L5','L6','L7','L8','L9','L10'}, E, 'cand');
end

function T2 = drive_star(T, sname, nm)
%DRIVE_STAR  Promote one undriven star of a 9-leg shape into a driven leg.
E = T.E; nd = T.nodes;
k = find(strcmp(nd, sname),1); nd{k} = 'L10';
for q = 1:size(E,1)
  if strcmp(E{q,2}, sname), E{q,2} = 'L10'; end
  if strcmp(E{q,3}, sname), E{q,3} = 'L10'; end
end
T2 = tp_mk(nm, nd, E, 'split reference');
end
