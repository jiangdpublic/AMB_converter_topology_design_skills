function b10_undrive()
%B10_UNDRIVE  Reference analysis: is B10-OPT-B really a 10-leg structure, or is
%   its zero-current degree-4 leg L1 replaceable by a hard-wired star (a 9-leg
%   topology)?  This script is about reference B only; the primary topology has
%   L1 at degree 5 and is not considered here.
par = tp_par(); C = b10_def(); T = C{1};
fprintf('--- B10-OPT-B as defined (10 legs) ---\n');
report(T, par, 'B10-OPT-B        ');
T2 = T; nd = T2.nodes;
for q = 1:numel(nd), if strcmp(nd{q},'L1'), nd{q} = 'X2'; end, end
E = T2.E;
for k = 1:size(E,1)
  if strcmp(E{k,2},'L1'), E{k,2} = 'X2'; end
  if strcmp(E{k,3},'L1'), E{k,3} = 'X2'; end
end
T2 = tp_mk('B10-OPT-B / L1 undriven', nd, E, '9-leg version');
fprintf('--- same wiring with L1 hard-wired as a star (9 legs) ---\n');
report(T2, par, 'B10-OPT-B - L1   ');
end

function report(T, par, nm)
isLegT = b9_isleg(T);
[Uf,info] = b9_floor_fast(T.nodes, T.E, par, isLegT);
Ul = tp_floor_leg(T.nodes, T.E, par, isLegT);
A = info.A; Ib = max(abs(A'*(par.i_bias*ones(10,1)))); Ie = 0; kcl = 0;
for s = 0:31
  b = dec2bin(s,5); sg = 2*(b-'0')-1; iw = zeros(10,1);
  for j = 1:5, iw(2*j-1)=par.i_bias+sg(j)*par.delta_max/2; iw(2*j)=par.i_bias-sg(j)*par.delta_max/2; end
  Ii = A'*iw(info.widx);
  Ie = max(Ie, max(abs(Ii(isLegT)))); kcl = max(kcl, max(abs(Ii(~isLegT))));
end
deg = zeros(1,numel(T.nodes));
for k=1:10
  a=find(strcmp(T.nodes,T.E{k,2})); b=find(strcmp(T.nodes,T.E{k,3}));
  deg(a)=deg(a)+1; deg(b)=deg(b)+1;
end
fprintf('%s legs %2d | cyc %d | floor %.4f phi (%.4f V, LP %.4f) | bias %.2f A | env %.2f A | KCL %.0e\n', ...
  nm, sum(isLegT), size(info.Z,2), Uf/par.phi, Uf, Ul, Ib, Ie, kcl);
fprintf('   free nodes: ');
for q=1:numel(T.nodes), if ~isLegT(q), fprintf('%s ', T.nodes{q}); end, end
fprintf(' | degrees: '); fprintf('%d ', deg); fprintf('\n');
end