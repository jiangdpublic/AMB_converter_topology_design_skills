function b10_aux_criteria(variant, Udc)
%B10_AUX_CRITERIA  Auxiliary criteria that deliberately IGNORE the control
%   current (its envelope is hard to know up front, so the main phi-based
%   criterion is not always the right sizing basis).  Everything here uses only
%   the BIAS operating point: all ten winding currents equal i_bias.
%
%   A) U_dc,avg  = bus needed to hold the bias currents alone
%                = (span of the leg potentials at the bias point)
%                = max over leg pairs of |n_b| * (R*i_bias)
%      It is exactly the "phi = 0" special case of the main criterion.
%
%   B) headroom  = with the bus fixed, how much voltage is left to DRIVE each
%      winding once the bias drops have been provided.  Place the lowest leg at
%      0 V (the most favourable placement, always realisable by the duty
%      assignment); driving winding k positive needs its TAIL to reach the top
%      rail while its HEAD stays above the bottom one, so
%          V+_k = U_dc - (p_head(k) - p_min)
%          V-_k = U_dc - (p_max - p_tail(k))
%      Scalar criterion: the minimum over the ten windings (and which winding).
%
%   Everything printed here uses ONLY static quantities: R, i_bias, the
%   topology and the bus you pass in.  Nothing depends on delta_max or f_c.
%
%   b10_aux_criteria('w9x1', 200)
if nargin < 1 || isempty(variant), variant = 'w9x1'; end   % primary topology
if nargin < 2 || isempty(Udc), Udc = 200; end
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
% ---- bias point: every winding carries i_bias -> u_k = R*i_bias -----------
ub  = par.R*par.i_bias*ones(nE,1);
phi = pinv(A)*ub;
p   = phi - min(phi(2:11));                 % lowest LEG at 0 V
S   = max(p(2:11)) - min(p(2:11));          % criterion A
% symbolic coefficients, used only for the |n_b| cross-check
coef = zeros(N,nE); known = false(1,N); known(1) = true;
for it = 1:20
  for k = 1:nE
    a = find(strcmp(nodes,E{k,2}),1); b = find(strcmp(nodes,E{k,3}),1);
    e = zeros(1,nE); e(k) = 1;
    if known(a) && ~known(b),     coef(b,:) = coef(a,:) - e; known(b) = true;
    elseif ~known(a) && known(b), coef(a,:) = coef(b,:) + e; known(a) = true;
    end
  end
end
nbmax = 0; nbpair = '';
for i = 2:11
  for j = i+1:11
    nb = abs(sum(coef(i,:) - coef(j,:)));
    if nb > nbmax, nbmax = nb; nbpair = sprintf('%s vs %s', nodes{i}, nodes{j}); end
  end
end
fprintf('=== auxiliary criteria (control current ignored):  netlist %s ===\n', char(variant));
fprintf('bias point: all ten windings at i_bias = %g A  ->  u_k = R*i_bias = %.2f V\n', ...
        par.i_bias, par.R*par.i_bias);
fprintf('\n(A) U_dc,avg = %.4f V   [= span of the leg potentials =  max|n_b|*R*i_bias = %d*%.2f]\n', ...
        S, nbmax, par.R*par.i_bias);
fprintf('    worst leg pair by |n_b|: %s      binding pair for the span: ', nbpair);
[~,i1] = max(p(2:11)); [~,i2] = min(p(2:11));
fprintf('%s (high) vs %s (low)\n', nodes{i1+1}, nodes{i2+1});
fprintf('\n    bias-point leg potential map (lowest leg = 0 V):\n');
for j = 1:10, fprintf('      %-4s %+7.3f V   (duty %.1f %%)\n', nodes{j+1}, p(j+1), 100*p(j+1)/Udc); end
% ---- criterion B: headroom per winding at the given bus ------------------
fprintf('\n(B) headroom at U_dc = %g V  (lowest leg parked at 0 V)\n', Udc);
fprintf('      %-6s %-14s %10s %10s   %s\n','winding','tail -> head','V+ [V]','V- [V]','note');
Vp = zeros(1,nE); Vm = zeros(1,nE);
for k = 1:nE
  it = find(strcmp(nodes,E{k,2}),1); ih = find(strcmp(nodes,E{k,3}),1);
  Vp(k) = Udc - (p(ih) - 0);
  Vm(k) = Udc - (S - p(it));
end
[mp,ip] = min(Vp); [mm,im] = min(Vm);
for k = 1:nE
  note = '';
  if k == ip, note = '<-- smallest V+'; end
  if k == im, note = [note '  <-- smallest V-']; end
  fprintf('      w%-5d %-14s %10.3f %10.3f   %s\n', widx(k), ...
          sprintf('%s -> %s', E{k,2}, E{k,3}), Vp(k), Vm(k), note);
end
fprintf('    min V+ = %.3f V (w%d, %.1f %% of U_dc) ;  min V- = %.3f V (w%d, %.1f %%)\n', ...
        mp, widx(ip), 100*mp/Udc, mm, widx(im), 100*mm/Udc);
end
