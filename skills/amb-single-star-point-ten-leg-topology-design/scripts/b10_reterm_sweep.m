function b10_reterm_sweep()
%B10_RETERM_SWEEP  Re-terminate ONE winding's driven end to every other leg and
%   report the criteria plus the differential-ripple class, for both current
%   direction modes.  Used to see where a proposed re-wiring sits in the
%   10-leg cycle-free design space.
%
%   Ripple class: a DOF pair is CLEAN when its two windings meet at a common node
%   with the SAME orientation (that node may be a leg OR the undriven star); the
%   differential voltage then reduces to the pair's own winding drops and the
%   ripple is R*delta*Ts/(2L) = 0.05 A.  Otherwise the pair spans 3+ legs and the
%   ripple is ~10x larger.
par = tp_par();
nodes = {'X','L1','L2','L3','L4','L5','L6','L7','L8','L9','L10'};
isLeg = logical([0 1 1 1 1 1 1 1 1 1 1]);
base = { 'w4','L1','X'; 'w6','L6','L2'; 'w5','L6','L4'; 'w2','L5','L1'; 'w9','L3','L6'; ...
         'w10','L1','L3'; 'w1','L7','L1'; 'w8','X','L8'; 'w7','X','L9'; 'w3','L10','X'};
fprintf('%-6s %-8s %8s %8s %8s %7s  %-22s %s\n', 'w4end','dir','Udc/phi','Idc_bias','Idc_env','KCLres','ripple class DOF1..5','note');
for t = 1:10
  for d = 0:1
    E = base;  E = setrow(E,'w4',{'w4',sprintf('L%d',t),'X'});
    if d == 1, E = d569(E); end
    [S, cls] = eval1(E, nodes, isLeg, par);
    note = '';
    if isequal(cls,'CCCCC') && S(2) <= 7.5 + 1e-9, note = '<== all clean AND 5 A leg class'; end
    fprintf('%-6s %-8s %8.4f %8.2f %8.2f %7.0e  %-22s %s\n', sprintf('L%d',t), ...
            tern(d==1,'-1 (ripple)','+1 (B)'), S(1)/par.phi, S(2), S(3), S(4), cls, note);
  end
end
end

function [out, cls] = eval1(E, nodes, isLeg, par)
wOfRow = zeros(1,10);
for k = 1:10, wOfRow(k) = sscanf(E{k,1},'w%d'); end
A = zeros(10,11);
for k = 1:10
  a = find(strcmp(nodes,E{k,2}),1); b = find(strcmp(nodes,E{k,3}),1);
  A(k,a) = 1; A(k,b) = -1;
end
[Uf, info] = b9_floor_fast(nodes, E, par, isLeg);
Ul = tp_floor_leg(nodes, E, par, isLeg);
if abs(Uf-Ul) > 1e-6, fprintf('   !! LP mismatch %.4f vs %.4f\n', Uf, Ul); end
A = info.A;
Ib = max(abs(A'*(par.i_bias*ones(10,1)))); Ie = 0; kcl = 0;
for s = 0:31
  b = dec2bin(s,5); sg = 2*(b-'0')-1; iw = zeros(10,1);
  for j = 1:5, iw(2*j-1)=par.i_bias+sg(j)*par.delta_max/2; iw(2*j)=par.i_bias-sg(j)*par.delta_max/2; end
  Ii = A'*iw(info.widx);
  Ie = max(Ie, max(abs(Ii(isLeg)))); kcl = max(kcl, max(abs(Ii(~isLeg))));
end
% ripple class: same-orientation meeting point for each DOF pair
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

function E = d569(E)
E = setrow(E,'w5',{'w5','L4','L6'});
E = setrow(E,'w6',{'w6','L2','L6'});
E = setrow(E,'w9',{'w9','L6','L3'});
end

function s = tern(c,a,b)
if c, s = a; else, s = b; end
end
