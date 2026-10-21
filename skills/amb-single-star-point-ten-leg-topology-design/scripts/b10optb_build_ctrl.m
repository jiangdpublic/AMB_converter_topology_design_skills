function build = b10optb_build_ctrl(mdl, S)
%B10OPTB_BUILD_CTRL  Ten per-unit PI winding-current loops plus the static
%   decoupling that turns them into ten leg-duty commands.
%
%   Why a decoupling matrix: the ten leg potentials are the actuators and the
%   ten winding currents are the outputs, but the windings SHARE legs, so the
%   loops are coupled.  With A the winding-node incidence matrix (rows in
%   w1..w10 order, columns X,L1..L10) the winding voltages are u = A'*phi, so
%   the leg potentials for a commanded u are  phi_L = M*u,  M = inv(A_L)  -
%   the same back-substitution the open-loop design uses.  Putting U_dc*M
%   between the controller outputs (per-unit winding voltages) and the leg
%   duties makes each of the ten loops a decoupled SISO loop.
%
%   Per winding:  iw<k> -> 2nd-order anti-alias filter -> ZOH(Ts) -> Memory ->
%   PI (Kp, Ki, clamp +-1, conditional-integration anti-windup) -> matrix ->
%   common-mode centring -> Saturation[0,1] -> Goto duty<k>
%
%   The subsystem has NO ports: it reads the winding currents from the iw<k>
%   Goto tags and publishes the duties as duty<k> tags, so the carefully routed
%   top level does not change.
FROM = 'simulink/Signal Routing/From';
GOTO = 'simulink/Signal Routing/Goto';
CON  = 'simulink/Sources/Constant';
SUM  = 'simulink/Math Operations/Sum';
GAI  = 'simulink/Math Operations/Gain';
SAT  = 'simulink/Discontinuities/Saturation';
TF   = 'simulink/Continuous/Transfer Fcn';
ZOH  = 'simulink/Discrete/Zero-Order Hold';
MEM  = 'simulink/Discrete/Memory';
MUX  = 'simulink/Signal Routing/Mux';
DMX  = 'simulink/Signal Routing/Demux';
MM   = 'simulink/Math Operations/MinMax';
BIA  = 'simulink/Math Operations/Bias';
SWI  = 'simulink/Signal Routing/Switch';
PRO  = 'simulink/Math Operations/Product';
LOG  = 'simulink/Logic and Bit Operations/Logical Operator';
CTC  = 'simulink/Logic and Bit Operations/Compare To Constant';
INT  = 'simulink/Continuous/Integrator';
SUB  = 'simulink/Ports & Subsystems/Subsystem';

sub = [mdl '/Ctrl'];
add_block(SUB, sub, 'Position', [160 1980 1180 3420]);
delete_line(sub,'In1/1','Out1/1');
delete_block([sub '/In1']); delete_block([sub '/Out1']);
q = @(b) get_param([sub '/' b],'PortHandles');

Ts = S.T_sw;
numT = S.wn_fb^2;  denT = [1 2*S.zeta_fb*S.wn_fb S.wn_fb^2];
stOut = zeros(10,2);
for k = 1:10
  s = num2str(k);  yr = 60 + 130*(k-1);
  % --- reference and feedback ---
  add_block(CON,[sub '/ref' s],'Position',[40 yr-8 110 yr+16]);
  set_param([sub '/ref' s],'Value',sprintf('%.6f',S.i_ref(k)));
  add_block(FROM,[sub '/iw' s],'Position',[40 yr+28 80 yr+48]);
  % channel k is winding w<k>, and the meters publish by winding number
  set_param([sub '/iw' s],'GotoTag',sprintf('iw%d',k));
  add_block(TF,[sub '/flt' s],'Position',[140 yr+4 240 yr+44]);
  set_param([sub '/flt' s],'Numerator',mat2str(numT,8),'Denominator',mat2str(denT,8));
  add_block(ZOH,[sub '/zoh' s],'Position',[280 yr+12 340 yr+36]);
  set_param([sub '/zoh' s],'SampleTime',num2str(Ts));
  add_block(MEM,[sub '/mem' s],'Position',[370 yr+12 430 yr+36]);
  % --- error and PI ---
  add_block(SUM,[sub '/err' s],'Position',[470 yr-6 510 yr+34]);
  set_param([sub '/err' s],'Inputs','+-');
  add_block(GAI,[sub '/kp' s],'Position',[550 yr-36 610 yr-12]);
  set_param([sub '/kp' s],'Gain',num2str(S.Kp));
  % --- conditional-integration anti-windup (skill amb-half-h section 1.5) ---
  aw = [sub '/aw' s];
  add_block(SUB, aw, 'Position', [550 yr+8 610 yr+48]);
  delete_line(aw,'In1/1','Out1/1');
  set_param([aw '/In1'],'Name','err');
  set_param([aw '/Out1'],'Name','Int_out');
  aq = @(b) get_param([aw '/' b],'PortHandles');
  add_block(SWI,[aw '/Switch'],'Position',[120 30 160 70]);
  set_param([aw '/Switch'],'Criteria','u2 ~= 0','Threshold','0');
  add_block(CON,[aw '/Constant'],'Position',[30 20 70 40]);  set_param([aw '/Constant'],'Value','0');
  add_block(GAI,[aw '/Gain1'],'Position',[30 80 70 100]);   set_param([aw '/Gain1'],'Gain',num2str(S.Ki));
  add_block(INT,[aw '/Integrator'],'Position',[210 30 250 70]);
  set_param([aw '/Integrator'],'LimitOutput','on','UpperSaturationLimit','1', ...
            'LowerSaturationLimit','-1','InitialCondition','0');
  add_block(CTC,[aw '/C1'],'Position',[300 10 350 30]);
  set_param([aw '/C1'],'relop','<=','const','-1');
  add_block(CTC,[aw '/C2'],'Position',[300 50 350 70]);
  set_param([aw '/C2'],'relop','>=','const','1');
  add_block(LOG,[aw '/OR'],'Position',[390 20 420 60]);
  set_param([aw '/OR'],'Operator','OR');
  add_block(PRO,[aw '/Product'],'Position',[300 100 330 140]);
  set_param([aw '/Product'],'Inputs','2');
  add_block(CTC,[aw '/C3'],'Position',[360 100 410 120]);
  set_param([aw '/C3'],'relop','>=','const','0');
  add_block(LOG,[aw '/AND'],'Position',[440 30 470 70]);
  set_param([aw '/AND'],'Operator','AND');
  % aw wiring
  w([aw],'err/1','Gain1/1');        % integrator input = Ki*err
  w([aw],'Constant/1','Switch/1');  w([aw],'Gain1/1','Switch/3');
  w([aw],'Switch/1','Integrator/1');
  w([aw],'Integrator/1','C1/1');    w([aw],'Integrator/1','C2/1');
  w([aw],'Integrator/1','Product/1');
  w([aw],'C1/1','OR/1');            w([aw],'C2/1','OR/2');
  w([aw],'err/1','Product/2');
  w([aw],'Product/1','C3/1');
  w([aw],'OR/1','AND/1');           w([aw],'C3/1','AND/2');
  w([aw],'AND/1','Switch/2');
  w([aw],'Integrator/1','Int_out/1');
  % --- PI sum, clamp ---
  add_block(SUM,[sub '/us' s],'Position',[650 yr-6 690 yr+34]);
  set_param([sub '/us' s],'Inputs','++');
  add_block(SAT,[sub '/st' s],'Position',[720 yr+4 780 yr+24]);
  set_param([sub '/st' s],'UpperLimit','1','LowerLimit','-1');
  % --- row wiring ---
  w(sub,['ref' s '/1'],['err' s '/1']);
  w(sub,['iw' s '/1'],['flt' s '/1']);
  w(sub,['flt' s '/1'],['zoh' s '/1']);
  w(sub,['zoh' s '/1'],['mem' s '/1']);
  w(sub,['mem' s '/1'],['err' s '/2']);
  w(sub,['err' s '/1'],['kp' s '/1']);
  w(sub,['err' s '/1'],['aw' s '/1']);
  w(sub,['kp' s '/1'],['us' s '/1']);
  w(sub,['aw' s '/1'],['us' s '/2']);
  w(sub,['us' s '/1'],['st' s '/1']);
  p = get_param(q(['st' s]).Outport(1),'Position'); stOut(k,:) = p;
end

% ---- ten commanded winding voltages -> leg potentials -----------------
% the Mux is placed first, then every row is routed to its own input channel
add_block(MUX,[sub '/MuxU'],'Position',[860 30 880 1390],'Inputs','10');
mxIn = zeros(10,2);
for k = 1:10, mxIn(k,:) = get_param(q(['MuxU']).Inport(k),'Position'); end
for k = 1:10
  xc = 800 + 8*(k-1);
  a = stOut(k,:); b = mxIn(k,:);
  if abs(a(2)-b(2)) < 1
    w(sub,['st' num2str(k) '/1'],['MuxU/' num2str(k)]);
  else
    wpts(sub,['st' num2str(k) '/1'],['MuxU/' num2str(k)], ...
         [a; xc a(2); xc b(2); b]);
  end
end
add_block(GAI,[sub '/Gm'],'Position',[930 660 1030 720]);
set_param([sub '/Gm'],'Gain',mat2str(S.U_dc*S.M,8),'Multiplication','Matrix(K*u)');
add_block(GAI,[sub '/pu'],'Position',[1070 660 1130 720]);
set_param([sub '/pu'],'Gain',num2str(1/S.U_dc));
add_block(DMX,[sub '/D1'],'Position',[1170 640 1175 740],'Outputs','10');
add_block(MM,[sub '/MX'],'Position',[1210 600 1250 630],'Function','max','Inputs','10');
add_block(MM,[sub '/MN'],'Position',[1210 660 1250 690],'Function','min','Inputs','10');
add_block(SUM,[sub '/off'],'Position',[1290 610 1320 650],'Inputs','++');
add_block(GAI,[sub '/half'],'Position',[1350 615 1400 645]); set_param([sub '/half'],'Gain','0.5');
add_block(SUM,[sub '/dc'],'Position',[1430 630 1460 670],'Inputs','+-');
add_block(BIA,[sub '/bias'],'Position',[1490 635 1540 665]); set_param([sub '/bias'],'Bias','0.5');
add_block(SAT,[sub '/dsat'],'Position',[1570 635 1630 665]);
set_param([sub '/dsat'],'UpperLimit','1','LowerLimit','0');
add_block(DMX,[sub '/D2'],'Position',[1670 560 1675 760],'Outputs','10');
w(sub,'MuxU/1','Gm/1');
w(sub,'Gm/1','pu/1');
w(sub,'pu/1','D1/1');
for k = 1:10
  w(sub,['D1/' num2str(k)],['MX/' num2str(k)]);
  w(sub,['D1/' num2str(k)],['MN/' num2str(k)]);
end
w(sub,'MX/1','off/1');  w(sub,'MN/1','off/2');
w(sub,'off/1','half/1');
w(sub,'pu/1','dc/1');   w(sub,'half/1','dc/2');
w(sub,'dc/1','bias/1');
w(sub,'bias/1','dsat/1');
w(sub,'dsat/1','D2/1');
for k = 1:10
  g = [sub '/Gd' num2str(k)];
  add_block(GOTO,g,'Position',[1720 520+22*(k-1) 1760 540+22*(k-1)]);
  set_param(g,'GotoTag',sprintf('duty%d',k),'TagVisibility','global');
  w(sub,['D2/' num2str(k)],['Gd' num2str(k) '/1']);
end
% ---- guard: a silently unconnected input would quietly disable a loop ----
bl = find_system(sub,'LookUnderMasks','all','FollowLinks','on','Type','Block');
nbad = 0;
for b = 1:numel(bl)
  if strcmp(get_param(bl{b},'BlockType'),'From'), continue; end
  ph = get_param(bl{b},'PortHandles');
  for j = 1:numel(ph.Inport)
    l = get_param(ph.Inport(j),'Line');
    if isempty(l) || all(l <= 0)
      fprintf(2,'  !! unconnected input %d of %s\n', j, bl{b});
      nbad = nbad + 1;
    end
  end
end
if nbad > 0
  error('b10optb:ctrl','%d unconnected controller inputs', nbad);
end
build = sub;
end

% ------------------------------------------------------------------ helpers
function w(sys, a, b)
h = add_line(sys, a, b);
try, set_param(h,'Points',[get_param(a,'Position'); get_param(b,'Position')]); catch, end
end

function wpts(sys, a, b, P)
h = add_line(sys, a, b);
keep = [true; any(diff(P,1,1) ~= 0, 2)];
try, set_param(h,'Points',P(keep,:)); catch e, fprintf(2,'  ! %s\n', e.message); end
end
