function b10_reterm_sweep2()
%B10_RETERM_SWEEP2  Move the DRIVEN end of any one winding to any leg, then
%   orient every DOF pair so that its two windings meet a common node with the
%   same orientation (the condition for a clean differential ripple), and report
%   the criteria of the resulting topology.  Only rows that are fully clean and
%   stay in the 5 A leg-current class are printed, sorted by the bus floor.
par = tp_par();
nodes = {'X','L1','L2','L3','L4','L5','L6','L7','L8','L9','L10'};
isLeg = logical([0 1 1 1 1 1 1 1 1 1 1]);
base = { 'w4','L1','X'; 'w6','L6','L2'; 'w5','L6','L4'; 'w2','L5','L1'; 'w9','L3','L6'; ...
         'w10','L1','L3'; 'w1','L7','L1'; 'w8','X','L8'; 'w7','X','L9'; 'w3','L10','X'};
rows = {}; vals = []; nets = {};
for w = 1:10
  r0 = rowOf(base, w);
  ends = {base{r0,2}, base{r0,3}};
  for e = 1:2                       % move the driven end (not the star end)
    for t = 1:10
      E = base;
      other = ends{3-e};
      if strcmp(other,'X'), continue; end          % keep the star end fixed
      E = setrow(E, sprintf('w%d',w), {sprintf('w%d',w), other, sprintf('L%d',t)});
      if strcmp(other, sprintf('L%d',t)), continue; end
      E = orient(E);                                % make every pair same-sign
      if isempty(E), continue; end
      [~, info0] = b9_floor_fast(nodes, E, par, isLeg);
      if size(info0.Z,2) > 0, continue; end         % must stay cycle-free (cyc = 0)
      [S, cls] = eval1(E, nodes, isLeg, par);
      if ~all(cls == 'C'), continue; end
      if S(3) > 7.5 + 1e-9 || S(4) > 1e-9, continue; end    % 5 A class, KCL exact
      rows{end+1} = sprintf('%-20s %7.4f %7.2f %7.2f %6.0e  %s', ...
        sprintf('w%d: %s end -> L%d', w, other, t), S(1)/par.phi, S(2), S(3), S(4), cls);
      vals(end+1) = S(1)/par.phi; nets{end+1} = E;
    end
  end
end
if isempty(rows)
  fprintf('no fully-clean 5 A-class single re-termination found\n');
else
  [~,o] = sort(vals);
  fprintf('Fully clean single-end re-terminations that stay in the 5 A leg class (sorted by bus):\n');
  for k = o'
    fprintf('  %s\n', rows{k});
    E = nets{k};  ne = size(E,1);
    deg = zeros(1,11);
    for q = 1:ne
      deg(find(strcmp(nodes,E{q,2}))) = deg(find(strcmp(nodes,E{q,2}))) + 1;
      deg(find(strcmp(nodes,E{q,3}))) = deg(find(strcmp(nodes,E{q,3}))) + 1;
    end
    fprintf('      netlist: '); fprintf('%s:%s->%s  ', E{q,1}, E{q,2}, E{q,3}); fprintf('\n');
    for q = 1:ne, fprintf('        %-4s %s -> %s\n', E{q,1}, E{q,2}, E{q,3}); end
    fprintf('      cycle rank E-V+C = %d (must be 0);  degrees:', ne - 11 + 1);
    fprintf(' %d', deg); fprintf('\n');
  end
end
end

function E = orient(E)
%ORIENT  Flip windings so that each DOF pair meets a common node same-sign.
wOfRow = zeros(1,10);
for k = 1:10, wOfRow(k) = sscanf(E{k,1},'w%d'); end
for j = 1:5
  a = find(wOfRow == 2*j-1,1); c = find(wOfRow == 2*j,1);
  na = {E{a,2},E{a,3}}; nc = {E{c,2},E{c,3}};
  if strcmp(na{1},nc{1}) || strcmp(na{2},nc{2})          % already same-sign
    continue
  elseif strcmp(na{1},nc{2})                              % common node, flip c
    E = setrow(E, E{c,1}, {E{c,1}, E{c,3}, E{c,2}});
  elseif strcmp(na{2},nc{1})                              % common node, flip a
    E = setrow(E, E{a,1}, {E{a,1}, E{a,3}, E{a,2}});
  else
    E = []; return                                        % pairs cannot be clean
  end
end
end

function r = rowOf(E, w)
r = 0;
for k = 1:size(E,1), if strcmp(E{k,1}, sprintf('w%d',w)), r = k; return; end, end
end

function [out, cls] = eval1(E, nodes, isLeg, par)
wOfRow = zeros(1,10);
for k = 1:10, wOfRow(k) = sscanf(E{k,1},'w%d'); end
[Uf, info] = b9_floor_fast(nodes, E, par, isLeg);
A = info.A; Ib = max(abs(A'*(par.i_bias*ones(10,1)))); Ie = 0; kcl = 0;
for s = 0:31
  b = dec2bin(s,5); sg = 2*(b-'0')-1; iw = zeros(10,1);
  for j = 1:5, iw(2*j-1)=par.i_bias+sg(j)*par.delta_max/2; iw(2*j)=par.i_bias-sg(j)*par.delta_max/2; end
  Ii = A'*iw(info.widx);
  Ie = max(Ie, max(abs(Ii(isLeg)))); kcl = max(kcl, max(abs(Ii(~isLeg))));
end
cls = repmat('-',1,5);
for j = 1:5
  a = find(wOfRow==2*j-1,1); c = find(wOfRow==2*j,1);
  if strcmp(E{a,2},E{c,2}) || strcmp(E{a,3},E{c,3}), cls(j) = 'C'; else, cls(j) = 'L'; end
end
out = [Uf Ib Ie kcl];
end

function E = setrow(E, nm, row)
for k = 1:size(E,1), if strcmp(E{k,1},nm), E(k,:) = row; return; end, end
end
