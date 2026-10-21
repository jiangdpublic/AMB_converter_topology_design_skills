function [Udc, info] = b9_floor_fast(nodes, E, par, isLeg)
%B9_FLOOR_FAST  Exact bus floor of a TREE topology in which only the leg nodes
%   are driven (phi_leg in [0,Udc]); star/rail nodes are free.  For a tree the
%   node potentials are fixed up to one global shift, and the span of the LEG
%   potentials is shift invariant, so
%        Udc(corner) = max(phi_leg) - min(phi_leg)   (no LP needed)
%   Udc_min = max over the 32 envelope corners.
N = numel(nodes); nE = size(E,1);
A = zeros(nE,N); nb = cell(1,N); ke = cell(1,N);
for k=1:nE
  it = find(strcmp(nodes,E{k,2}),1); ih = find(strcmp(nodes,E{k,3}),1);
  A(k,it)=1; A(k,ih)=-1;
  nb{it} = [nb{it} ih]; ke{it} = [ke{it} k];
  nb{ih} = [nb{ih} it]; ke{ih} = [ke{ih} k];
end
widx = zeros(1,nE); for k=1:nE, widx(k) = sscanf(E{k,1},'w%d'); end
V = zeros(32,nE);
for s=0:31
  b = dec2bin(s,5); sg = 2*(b-'0')-1;
  for k=1:nE
    j = ceil(widx(k)/2);
    if mod(widx(k),2)==1, u = par.R*par.i_bias + sg(j)*par.phi/2;
    else,                 u = par.R*par.i_bias - sg(j)*par.phi/2; end
    V(s+1,k) = u;
  end
end
Z = null(A');
Udc = 0; ok = true;
for s=1:32
  u = V(s,:)';
  if ~isempty(Z) && norm(Z'*u) > 1e-8*max(1,norm(u)), ok=false; Udc=Inf; break; end
  ph = nan(1,N); ph(1) = 0; q = 1;
  while ~isempty(q)
    v = q(1); q(1) = [];
    for m = 1:numel(nb{v})
      w2 = nb{v}(m); k = ke{v}(m);
      if isnan(ph(w2))
        if A(k,v)==1, ph(w2) = ph(v) - u(k); else, ph(w2) = ph(v) + u(k); end
        q(end+1) = w2;
      end
    end
  end
  pl = ph(logical(isLeg));
  Udc = max(Udc, max(pl) - min(pl));
end
info = struct('ok',ok,'A',A,'widx',widx,'Z',Z);
end