function b10_variant_check()
%B10_VARIANT_CHECK  Criteria + differential-ripple predictor for the primary
%   topology and its reference variants.
%
%   Bus floor / leg currents come from the archived tools (b9_floor_fast, with
%   tp_floor_leg as the LP cross-check).  The ripple is predicted from the
%   switching model: with one common carrier, leg j is high while tri < d_j, so
%   the differential voltage of DOF pair (a,c) is
%        v_diff(t) = sum_j cvec_j * Udc * (tri(t) < d_j),  cvec = A(a,:)-A(c,:)
%   and L*di/dt = v_diff - R*i is integrated over a carrier period.  Because
%   d_j = 0.5 + (phi_j - mean)/Udc, a pulse lasts |dphi|*Ts/(2*Udc) at amplitude
%   Udc, so the excursion is |dphi|*Ts/(2L): the ripple is set by the DESIGN
%   POTENTIALS, not by the bus value.
par   = tp_par();
nodes = {'X','L1','L2','L3','L4','L5','L6','L7','L8','L9','L10'};
isLeg = isleg_(nodes);
V = variants();
fprintf('R=%g ohm  L=%g uH  i_bias=%g A  delta_max=%g A  f_sw=%g kHz  phi=%.4f V\n', ...
        par.R, 1e6*par.L, par.i_bias, par.delta_max, par.f_sw/1e3, par.phi);
fprintf('measured reference (primary model, delta=1 A, Udc=200 V): DOF ripple p-p = 0.054 0.051 0.054 0.053 0.050 A\n\n');
fprintf('%-30s %8s %8s %8s %7s   %s\n', 'variant','Udc/phi','Idc_bias','Idc_env','KCLres','predicted DOF1..5 ripple p-p [A]');
for i = 1:numel(V)
  report(V(i), nodes, isLeg, par);
end
end

function V = variants()
% base = the PRIMARY topology (single-star-point ten-leg, internal id w9x1).
base = b10optb_netlist('w9x1');
V = struct('name',{},'E',{},'note',{},'probe',{});
V(1).name = 'single-star-point (w9x1)'; V(1).E = base; V(1).probe = 0;
V(1).note = 'PRIMARY: all five pairs same-sign; 2.3501 phi / 5 A / 7.5 A';
V(2).name = 'B10-OPT-B (reference)';    V(2).E = b10optb_netlist('B'); V(2).probe = 1;
V(2).note = 'REFERENCE: minimum bus in the 5 A class; DOF5 0.5 A ripple';
V(3).name = 'direction_569 = -1';       V(3).E = b10optb_netlist('d569'); V(3).probe = 0;
V(3).note = 'REFERENCE: all clean, but shared legs carry 10 A; 3.0252 phi';
V(4).name = 'w4 re-terminated L1->L3';  V(4).E = w4x3(b10optb_netlist('B')); V(4).probe = 0;
V(4).note = 'HISTORICAL: w4 goes L3->X instead of L1->X (B directions)';
V(5).name = 'w4->L3 + d569 = -1';       V(5).E = b10optb_netlist('w4x3-ripple'); V(5).probe = 0;
V(5).note = 'HISTORICAL: w4->L3 and direction_569 = -1';
C = b10_def();                                   % the archived 10-leg family
V(6).name = 'B10-OPT-A (10 A class)';   V(6).E = C{2}.E; V(6).probe = 0;
V(6).note = 'archived 10-leg variant, 2.0000 phi / 10 A / 15 A';
V(7).name = 'B10-OPT-C (15 A class)';   V(7).E = C{3}.E; V(7).probe = 0;
V(7).note = 'archived 10-leg variant, 1.6751 phi / 15 A / 22.5 A';
V(8).name = 'user proposal';            V(8).E = swap_w5_w9(b10optb_netlist('B')); V(8).probe = 0;
V(8).note = 'HISTORICAL: w5<->w9 swapped, w6 reversed; dominated by B';
V(9).name = 'reverse w6 only';          V(9).E = setrow(b10optb_netlist('B'),'w6',{'w6','L2','L6'}); V(9).probe = 0;
V(9).note = 'only DOF3 sharing polarity flipped';
V(10).name = 'reverse w9 only';         V(10).E = setrow(b10optb_netlist('B'),'w9',{'w9','L6','L3'}); V(10).probe = 0;
V(10).note = 'DOF5 same-sign shared at L3, but 15 A legs';
end

function E = swap_w5_w9(E)
E = setrow(E,'w5', {'w5', 'L3','L6'});
E = setrow(E,'w9', {'w9', 'L6','L4'});
E = setrow(E,'w6', {'w6', 'L2','L6'});
end

function E = w4x3(E)
% w4 re-terminated from (L1,X) to (L3,X)
E = setrow(E,'w4',{'w4','L3','X'});
end

function E = d569(E)
% the "ripple priority" setting of the source record (direction_569 = -1)
E = setrow(E,'w5',{'w5','L4','L6'});
E = setrow(E,'w6',{'w6','L2','L6'});
E = setrow(E,'w9',{'w9','L6','L3'});
end

function E = setrow(E, nm, row)
k = find(strcmp(E(:,1), nm), 1); E(k,:) = row;
end

function tf = isleg_(nodes)
tf = true(1,numel(nodes));
for k = 1:numel(nodes), tf(k) = ~strncmp(nodes{k},'X',1); end
end

function report(T, nodes, isLeg, par)
[Uf,info] = b9_floor_fast(nodes, T.E, par, isLeg);
A = info.A; widx = info.widx;
Ul = tp_floor_leg(nodes, T.E, par, isLeg);
Ib = max(abs(A'*(par.i_bias*ones(10,1)))); Ie = 0; kcl = 0;
for s = 0:31
  b = dec2bin(s,5); sg = 2*(b-'0')-1; iw = zeros(10,1);
  for j = 1:5, iw(2*j-1) = par.i_bias + sg(j)*par.delta_max/2; iw(2*j) = par.i_bias - sg(j)*par.delta_max/2; end
  Ii = A'*iw(widx);
  Ie = max(Ie, max(abs(Ii(isLeg)))); kcl = max(kcl, max(abs(Ii(~isLeg))));
end
if isfield(T,'probe') && T.probe
  % --- carrier-phase study on this wiring (no rewiring, modulation only) ---
  iw = zeros(10,1);
  for j = 1:5, iw(2*j-1) = par.i_bias + 0.5; iw(2*j) = par.i_bias - 0.5; end
  u = par.R*iw(widx); phiP = pinv(A)*u; phiP = phiP - min(phiP(isLeg));
  dP = 0.5 + (phiP - (max(phiP(isLeg))+min(phiP(isLeg)))/2)/200;
  fprintf('%-28s carrier-phase study for %s (phase of legs 1/3/6) -> DOF1..5 ripple\n','',T.name);
  grid1 = [0 0 0; 0 0 .5; 0 .5 0; 0 .5 .5; 0 1/3 2/3; 0 .5 .25; 0 .25 .5; 0 2/3 1/3];
  best = inf; bestg = grid1(1,:); bestr = zeros(1,5);
  for q = 1:size(grid1,1)
    ph = zeros(1,numel(dP)); ph([2 4 7]) = grid1(q,:);   % nodes: X,L1..L10 -> legs 1,3,6
    rr = zeros(1,5);
    for j = 1:5, rr(j) = dof_ripple(A, widx, dP, isLeg, par, j, 200, ph); end
    fprintf('%-28s ph=[%.2f %.2f %.2f]  %.3f %.3f %.3f %.3f %.3f\n','',grid1(q,1),grid1(q,2),grid1(q,3),rr);
    if max(rr) < best, best = max(rr); bestg = grid1(q,:); bestr = rr; end
  end
  fprintf('%-28s best max-ripple %.3f A at ph=[%.2f %.2f %.2f]\n\n','',best,bestg(1),bestg(2),bestg(3));
end
% design potentials at the operating point of the simulation (delta = 1 A/DOF)
iw = zeros(10,1);
for j = 1:5, iw(2*j-1) = par.i_bias + 0.5; iw(2*j) = par.i_bias - 0.5; end
u = par.R*iw(widx);
phi = pinv(A)*u;
phi = phi - min(phi(isLeg));                        % potentials referenced to the lowest leg
Udc = 200;                                          % same bus as the Simscape model
d = 0.5 + (phi - (max(phi(isLeg))+min(phi(isLeg)))/2)/Udc;   % duty, centred like the model
rip = zeros(1,5); shr = cell(1,5);
for j = 1:5
  ra = find(widx == 2*j-1); rc = find(widx == 2*j);
  shr{j} = sharing_(A(ra,:)-A(rc,:), nodes);
  rip(j) = dof_ripple(A, widx, d, isLeg, par, j, Udc);
end
flag = '';
if abs(Uf-Ul) > 1e-6, flag = ' [LP MISMATCH]'; end
fprintf('%-28s %8.4f %8.2f %8.2f %7.0e   ', T.name, Uf/par.phi, Ib, Ie, kcl); 
fprintf('%.3f ', rip); fprintf('%s\n', flag);
fprintf('%-28s %s\n', '', T.note);
fprintf('%-28s leg bias currents: ', ''); fprintf('%+.0f ', (A'*(par.i_bias*ones(10,1)))'); fprintf('\n');
fprintf('%-28s sharing: ', '');
for j = 1:5, fprintf('DOF%d=%s ', j, shr{j}); end
fprintf('\n\n');
end

function s = sharing_(cvec, nodes)
ix = find(abs(cvec) > 0);
if numel(ix) == 2 && abs(cvec(ix(1))) == 1 && abs(cvec(ix(2))) == 1
  s = sprintf('2leg%s', strjoin(nodes(ix),'/'));
else
  s = sprintf('%dleg', numel(ix));
end
end

function r = dof_ripple(A, widx, d, isLeg, par, j, Udc, ph)
% ph (optional): carrier phase of every leg, in periods
if nargin < 8, ph = zeros(size(d)); end
ra = find(widx == 2*j-1); rc = find(widx == 2*j);
cvec = A(ra,:) - A(rc,:);
N = numel(d);
if any(abs(cvec(~isLeg)) > 0), r = NaN; return; end      % star must cancel
Ns = 4096; tt = (0:Ns-1)/Ns; tri = 1 - abs(2*tt - 1);
Vn = zeros(N,Ns);
for k = find(isLeg(:))'
  tk = mod(tt - ph(k), 1);  trk = 1 - abs(2*tk - 1);     % phase-shifted carrier
  Vn(k,:) = Udc*(trk < d(k));                            % leg potentials [V]
end
v = cvec*Vn;
Ts = 1/par.f_sw; L = par.L; R = par.R; dt = Ts/Ns;
i = 0; hist = zeros(1,Ns);
for cyc = 1:80
  for k = 1:Ns
    i = i + dt*(v(k) - R*i)/L;
    hist(k) = i;
  end
end
r = max(hist) - min(hist);
end
