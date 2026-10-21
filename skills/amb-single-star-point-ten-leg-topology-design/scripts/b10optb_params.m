function S = b10optb_params(variant)
%B10OPTB_PARAMS  Parameters of the single-star-point ten-leg AMB model.
%   Edit this file, then re-run b10optb_build_all (it rebuilds the duty
%   constants and the model workspace from these numbers).
%
%   The default variant is 'w9x1' = the single-star-point ten-leg topology
%   (the primary topology of this skill).  Reference variants 'B' and 'd569'
%   are accepted by b10optb_netlist for comparison.
%
%   i_ctrl is the per-DOF control current amplitude [A]; make it a vector
%   i_ctrl_by_dof = [1 1 0.5 0 1] to give every DOF its own value.
%
%   The leg potentials are solved on the winding tree, so the same code works
%   for any netlist returned by b10optb_netlist (see that file).
if nargin < 1, variant = 'w9x1'; end
mdl    = 'SINGLE_STAR_TEN_LEG_AMB';
U_dc   = 200;             % dc bus [V] (fixed by the user, not a minimum)
R_w    = 1.0;             % winding resistance [ohm]
L_w    = 500e-6;          % winding inductance [H]
i_bias = 5.0;             % bias current [A]
i_ctrl = 1.0;             % control current amplitude PER DOF [A]  <-- ADJUSTABLE
i_ctrl_by_dof = i_ctrl*ones(1,5);   % e.g. [1 1 0.5 0 1]

i_ref = zeros(1,10);
for j = 1:5
  i_ref(2*j-1) = i_bias + i_ctrl_by_dof(j)/2;
  i_ref(2*j)   = i_bias - i_ctrl_by_dof(j)/2;
end

f_sw = 20e3;  T_sw = 1/f_sw;  Ts_ctrl = T_sw/2;  T_stop = 8e-3;

% steady state: u_k = R_w*i_ref_k is the drop of winding k from tail to head.
% Solve phi(tail) - phi(head) = u_k on the tree, with the star node X at 0.
E    = b10optb_netlist(variant);
nd   = {'X','L1','L2','L3','L4','L5','L6','L7','L8','L9','L10'};
u    = R_w*i_ref;
% the ROW order of E is arbitrary: the winding number of row k fixes which
% reference current (and which u) belongs to that row
wn   = zeros(1,10);
for k = 1:10, wn(k) = sscanf(E{k,1},'w%d'); end
phi  = nan(1,11);  phi(1) = 0;                     % node 1 = X
for it = 1:20
  for k = 1:10
    a = find(strcmp(nd,E{k,2}),1); b = find(strcmp(nd,E{k,3}),1);
    if ~isnan(phi(a)) && isnan(phi(b)), phi(b) = phi(a) - u(wn(k));
    elseif isnan(phi(a)) && ~isnan(phi(b)), phi(a) = phi(b) + u(wn(k));
    end
  end
end
if any(isnan(phi))
  error('b10optb:netlist','netlist "%s" is not a connected tree', char(variant));
end
% KCL at the star node must close by construction, otherwise the bias point
% cannot be held - report it instead of silently simulating the wrong point.
ks = 0;
for k = 1:10
  a = find(strcmp(nd,E{k,2}),1); b = find(strcmp(nd,E{k,3}),1);
  if a == 1, ks = ks - i_ref(wn(k)); end
  if b == 1, ks = ks + i_ref(wn(k)); end
end
if abs(ks) > 1e-9
  warning('b10optb:kcl','star-node KCL residual %.4g A for variant "%s"', ks, char(variant));
end

% ---- closed-loop controller data -------------------------------------
% The 10 leg potentials are the actuators and the 10 winding currents are the
% outputs, but the windings SHARE legs, so the loops are coupled.  With the
% incidence matrix A (rows = windings in w1..w10 order, columns = X, L1..L10)
% the winding voltages are  u = A'*phi  and, dropping the star column,
%     phi_L = M*u ,   M = inv(A_L')
% A per-winding PI therefore becomes 10 decoupled SISO loops by putting M (and
% the bus scaling) between the controller outputs and the leg duties.
Es = E;  wn2 = zeros(1,10);
for k = 1:10, wn2(k) = sscanf(E{k,1},'w%d'); end
[~,ord] = sort(wn2);  Es = E(ord,:);
A = zeros(10,11);
for k = 1:10
  a = find(strcmp(nd,Es{k,2}),1); b = find(strcmp(nd,Es{k,3}),1);
  A(k,a) = 1; A(k,b) = -1;
end
AL = A(:,2:11);                       % drop column 1 = the star node X
M  = inv(AL);                         % AL*phi_L = u  ->  phi_L = M*u
if norm(M*u(:) - phi(2:11)') > 1e-9
  error('b10optb:M','decoupling matrix inconsistent with the tree solution');
end
% the model meters the winding of netlist ROW r under the tag iw<r>, while the
% controller channels are numbered by WINDING (w1..w10) so that M applies:
ctrlRow = zeros(1,10);        % winding number -> netlist row
wOfRow  = zeros(1,10);        % netlist row    -> winding number
for k = 1:10
  ctrlRow(k) = find(strcmp(E(:,1), sprintf('w%d',k)), 1);
  wOfRow(k)  = sscanf(E{k,1}, 'w%d');
end
f_c    = 1500;                        % current-loop bandwidth [Hz] = f_sw/13
Kp     = 2*pi*f_c*L_w/U_dc;           % per-unit PI, zero cancels the R/L pole
Ki     = 2*pi*f_c*R_w/U_dc;
fn_fb  = 6000;  zeta_fb = 0.707;      % feedback anti-aliasing filter
wn_fb  = 2*pi*fn_fb;

% duty of leg k:  v_mid,k = d_k*U_dc, with X as the potential reference,
% centred in [0,1]   (open-loop starting point / fallback)
d_ref = phi(2:11)/U_dc + 0.5;
duty  = d_ref + (0.5 - (max(d_ref)+min(d_ref))/2);

if bdIsLoaded(mdl)
  hws = get_param(mdl,'ModelWorkspace'); hws.clear;
  nm = {'U_dc','R_w','L_w','i_bias','i_ctrl','i_ctrl_by_dof','i_ref','f_sw', ...
        'T_sw','Ts_ctrl','T_stop','u','phi','duty','M','Kp','Ki','f_c', ...
        'wn_fb','zeta_fb'};
  for k = 1:numel(nm), hws.assignin(nm{k}, eval(nm{k})); end
  fprintf('params pushed into %s model workspace\n', mdl);
end
fprintf('  netlist variant "%s"  (%s)\n', char(variant), strjoin(E(:,1)', ' '));
fprintf('  U_dc=%.0f V  i_bias=%.1f A  i_ctrl=%s A/DOF   star KCL residual %.2e A\n', ...
        U_dc, i_bias, mat2str(i_ctrl_by_dof), ks);
fprintf('  i_ref  = '); fprintf('%.3f ', i_ref); fprintf('\n');
fprintf('  phi    = '); fprintf('%.2f ', phi(2:11)); fprintf('   [V]  (span %.2f)\n', ...
        max(phi(2:11))-min(phi(2:11)));
fprintf('  duty%%  = '); fprintf('%.3f ', 100*duty); fprintf('\n');
fprintf('  closed loop: f_c=%g Hz  Kp=%.6f  Ki=%.4f (per-unit)  filter fn=%g Hz zeta=%.3f\n', ...
        f_c, Kp, Ki, fn_fb, zeta_fb);
S = struct('mdl',mdl,'U_dc',U_dc,'R_w',R_w,'L_w',L_w,'i_bias',i_bias,'i_ctrl',i_ctrl, ...
           'i_ctrl_by_dof',i_ctrl_by_dof,'i_ref',i_ref,'f_sw',f_sw,'T_sw',T_sw, ...
           'Ts_ctrl',Ts_ctrl,'T_stop',T_stop,'u',u,'phi',phi,'duty',duty, ...
           'M',M,'Kp',Kp,'Ki',Ki,'f_c',f_c,'wn_fb',wn_fb,'zeta_fb',zeta_fb, ...
           'ctrlRow',ctrlRow,'wOfRow',wOfRow, ...
           'variant',char(variant),'E',{E});
end
