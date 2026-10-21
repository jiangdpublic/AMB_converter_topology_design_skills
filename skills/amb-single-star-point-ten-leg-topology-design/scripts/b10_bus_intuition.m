function b10_bus_intuition(variant)
%B10_BUS_INTUITION  Show, for one netlist, exactly where the dc-bus requirement
%   comes from -- and in a form a human can check by hand.
%
%   For every envelope corner (each DOF takes +-delta_max) the winding voltages
%   are  u_k = R*i_bias + s_dof * sigma_k * phi/2  with sigma_k = +1 for an a
%   winding and -1 for a c winding.  Solving A*phi = u along the tree gives every
%   leg potential as a signed sum of the u_k on its path from the star, so the
%   bus needed at that corner is the SPREAD max(phi_leg) - min(phi_leg), and
%
%        U_dc,min = max over the 32 corners of that spread.
%
%   The printout lists the symbolic path sums, the binding corner, the two
%   binding legs, the path between them and its breakdown into "phi/2 terms"
%   and "R*i_bias terms" -- i.e. the hand calculation.
if nargin < 1, variant = 'w9x1'; end   % primary topology
par = tp_par();
nodes = {'X','L1','L2','L3','L4','L5','L6','L7','L8','L9','L10'};
E = b10optb_netlist(variant);
nE = size(E,1); N = numel(nodes);
A = zeros(nE,N); widx = zeros(1,nE);
for k = 1:nE
  A(k,find(strcmp(nodes,E{k,2}),1)) =  1;
  A(k,find(strcmp(nodes,E{k,3}),1)) = -1;
  widx(k) = sscanf(E{k,1},'w%d');
end
% ---- symbolic leg potentials: walk the tree from the star -----------------
coef = zeros(N,nE); known = false(1,N); known(1) = true;
for it = 1:20
  for k = 1:nE
    a = find(strcmp(nodes,E{k,2}),1); b = find(strcmp(nodes,E{k,3}),1);
    e = zeros(1,nE); e(k) = 1;
    if known(a) && ~known(b),      coef(b,:) = coef(a,:) - e; known(b) = true;
    elseif ~known(a) && known(b),  coef(a,:) = coef(b,:) + e; known(a) = true;
    end
  end
end
fprintf('=== bus requirement from the topology:  netlist %s ===\n', char(variant));
fprintf('R=%g ohm  i_bias=%g A  delta_max=%g A  phi=%.4f V  phi/2=%.4f V  R*i_bias=%.1f V\n\n', ...
        par.R, par.i_bias, par.delta_max, par.phi, par.phi/2, par.R*par.i_bias);
fprintf('leg potentials (star X = 0, u_k = winding voltage tail->head):\n');
for j = 2:11
  s = '';
  for k = 1:nE
    c = coef(j,k);
    if c > 0, s = [s sprintf(' + u%d', widx(k))];      %#ok<AGROW>
    elseif c < 0, s = [s sprintf(' - u%d', widx(k))];  %#ok<AGROW>
    end
  end
  fprintf('   phi_%-4s =%s\n', nodes{j}, s);
end
% ---- every leg pair: worst-case difference and its n_phi / n_b breakdown --
% (this is the table to consult BEFORE deciding which pair binds)
% a windings (odd w) take +phi/2, c windings (even w) take -phi/2 -- the same
% convention as tp_floor_leg.m, so the printed corner really means
% "s_j = +1  <=>  delta_j = +delta_max"
Edir = zeros(nE,5); sgm = zeros(1,nE);
for k = 1:nE
  j = ceil(widx(k)/2);
  sgn = 2*mod(widx(k),2) - 1;                % odd (a): +1, even (c): -1
  Edir(k,j) = sgn; sgm(k) = sgn;
end
best = zeros(N,N); bs = zeros(N,N);
for s = 0:31
  b = dec2bin(s,5); sg = 2*(b-'0')-1;
  u = par.R*par.i_bias*ones(nE,1) + Edir*(sg'*par.phi/2);
  phi = pinv(A)*u;
  for i = 2:11
    for j = i+1:11
      d = phi(i) - phi(j);
      if abs(d) > abs(best(i,j)), best(i,j) = d; bs(i,j) = s; end
    end
  end
end
rows = {}; vals = [];
for i = 2:11
  for j = i+1:11
    df = coef(i,:) - coef(j,:);
    b = dec2bin(bs(i,j),5); sg = 2*(b-'0')-1;
    nphi = 0; nb = 0;
    for k = 1:nE
      if df(k) ~= 0
        nphi = nphi + df(k)*sgm(k)*sg(ceil(widx(k)/2));
        nb   = nb   + df(k);
      end
    end
    rows{end+1} = sprintf('   %-14s %5d %7d %7d %12.4f %10.4f', ...
        sprintf('%s vs %s', nodes{i}, nodes{j}), sum(df~=0), nphi, nb, best(i,j), abs(best(i,j))/par.phi);
    vals(end+1) = abs(best(i,j));
  end
end
[~,o] = sort(vals,'descend');
fprintf('\nleg pair        terms   n_phi  n_bias     worst dV      /phi   (45 pairs, worst first)\n');
for k = 1:min(12,numel(o)), fprintf('%s\n', rows{o(k)}); end
fprintf('   ... (%d pairs total; "terms" is the raw count, n_phi/n_bias after cancelling)\n', numel(o));

% ---- 32 corners -----------------------------------------------------------
ub = par.R*par.i_bias*ones(nE,1);
Z  = null(A');        % u must lie in range(A): Z'*u = 0  (star KCL feasibility)
best = -inf; bests = 0; bl = [0 0];
for s = 0:31
  b = dec2bin(s,5); sg = 2*(b-'0')-1;
  u = ub + Edir*(sg'*par.phi/2);
  if ~isempty(Z) && norm(Z'*u) > 1e-9*max(1,norm(u))
    fprintf('   corner %2d infeasible (star KCL cannot close)\n', s); continue;
  end
  phi = pinv(A)*u;  legs = phi(2:11);
  [mx,imx] = max(legs); [mn,imn] = min(legs);
  if mx-mn > best, best = mx-mn; bests = s; bl = [imx imn]; end
end
b = dec2bin(bests,5); sg = 2*(b-'0')-1;
fprintf('\nbinding corner: delta signs per DOF = %s\n', mat2str(sg));
fprintf('binding legs   : %s (high) vs %s (low)\n', nodes{bl(1)+1}, nodes{bl(2)+1});
df = coef(bl(1)+1,:) - coef(bl(2)+1,:);
s2 = ''; nPhi = 0; nBias = 0;
for k = 1:nE
  if df(k) ~= 0
    j = ceil(widx(k)/2); sig = 2*mod(widx(k),2) - 1;    % a:+1, c:-1
    s2 = [s2 sprintf('%+d*u%d ', df(k), widx(k))];       %#ok<AGROW>
    nPhi  = nPhi  + df(k)*sig*sg(j);                     % phi/2 coefficient
    nBias = nBias + df(k);                               % R*i_bias coefficient
  end
end
fprintf('path between them: %s\n', s2);
fprintf('   -> %d * (phi/2) %+d * (R*i_bias) = %d*%.4f %+d*%.1f = %.4f V\n', ...
        nPhi, nBias, nPhi, par.phi/2, nBias, par.R*par.i_bias, ...
        nPhi*par.phi/2 + nBias*par.R*par.i_bias);
fprintf('\nU_dc,min = %.4f V = %.4f phi      (LP cross-check: %.4f phi)\n', ...
        best, best/par.phi, tp_floor_leg(nodes,E,par,logical([0 1 1 1 1 1 1 1 1 1 1]))/par.phi);
end
