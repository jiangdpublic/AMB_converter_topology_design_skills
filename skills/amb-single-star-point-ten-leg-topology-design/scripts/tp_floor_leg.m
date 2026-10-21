function [Udc, A, info] = tp_floor_leg(nodes, E, par, isLeg)
%TP_FLOOR_LEG  Minimum dc bus of a fixed-bias topology in which only the LEG
%   nodes are driven from the bus:  phi_leg in [0, Udc], while star / rail
%   nodes are free (their potential is set by the circuit, not by a leg).
%   For a given envelope corner the required winding voltages u must satisfy
%   Z'*u = 0 (Z = null(A')); the minimum bus is then the smallest window that
%   can hold the potentials of the LEG nodes (the free nodes are unconstrained).
N = numel(nodes); nE = size(E,1);
A = zeros(nE,N);
for k=1:nE
  it = find(strcmp(nodes,E{k,2}),1); ih = find(strcmp(nodes,E{k,3}),1);
  A(k,it)= 1; A(k,ih) = -1;
end
widx = zeros(1,nE);
for k=1:nE, widx(k) = sscanf(E{k,1},'w%d'); end
Edir = zeros(nE,5);
for k=1:nE
  j = ceil(widx(k)/2);
  if mod(widx(k),2)==1, Edir(k,j) = 1; else, Edir(k,j) = -1; end
end
u_bias = par.R*par.i_bias*ones(nE,1);
Z = null(A');
opt = optimoptions('linprog','Display','off');
ilo = find(isLeg);                     % bounded (leg) nodes
U = 0; ok = true;
for s=0:31
  b = dec2bin(s,5); sg = 2*(b-'0')-1;
  u = u_bias + Edir*(sg'*par.phi/2);
  if ~isempty(Z) && norm(Z'*u) > 1e-8*max(1,norm(u)), ok = false; U = Inf; break; end
  % LP:  min U  s.t.  A*phi = u ;  0 <= phi_j <= U  for j in ilo
  f = [zeros(N,1); 1];
  Aeq = [A, zeros(nE,1)]; beq = u;
  A1 = zeros(numel(ilo), N+1); A1(:,ilo) = eye(numel(ilo)); A1(:,N+1) = -1;
  A2 = zeros(numel(ilo), N+1); A2(:,ilo) = -eye(numel(ilo));
  lb = -inf(N+1,1); lb(ilo) = 0; lb(N+1) = 0;
  [x,~,ef] = linprog(f,[A1;A2],zeros(2*numel(ilo),1),Aeq,beq,lb,[],opt);
  if ef ~= 1, ok = false; U = Inf; break; end
  U = max(U, x(end));
end
Udc = U;
info = struct('ok',ok,'isLeg',isLeg,'widx',widx,'A',A,'Z',Z);
end