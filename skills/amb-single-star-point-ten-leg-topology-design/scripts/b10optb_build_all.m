function S = b10optb_build_all(varargin)
%B10OPTB_BUILD_ALL  Rebuild + wire + check (+ simulate) the single-star-point
%   ten-leg AMB drive.
%
%   b10optb_build_all              build the primary topology, wire, check, simulate
%   b10optb_build_all('nosim')     build, wire, overlap check only
%   b10optb_build_all('B')         build reference B10-OPT-B instead
%   b10optb_build_all('d569')      build reference direction_569 = -1 instead
%
%   Model SINGLE_STAR_TEN_LEG_AMB is built at Simscape device level and every
%   wire is routed explicitly (no autorouting), so that wires of *different*
%   nets never lie on top of each other - see b10optb_line_overlap.
%
%     Leg1..Leg10   half bridge: 2x IGBT (Ideal, Switching) + 2 freewheel
%                   Diode + 2 Simulink-PS Converter.  Gates come from Goto
%                   tags gU<k>/gL<k>.  Connection ports (left edge, top to
%                   bottom): P = bus +, M = midpoint, N = bus -
%     W1..W10       winding branch: Current Sensor + Inductor + Resistor.
%                   Connection ports T (tail) / H (head), signal out i.
%     PWM block     one 20 kHz carrier, per leg a duty From tag, 2 comparators,
%                   2 x15 gate drivers -> Goto gU<k>/gL<k>
%     Metering      Goto iw<k> at every winding -> From -> Mux -> Scope_Iw
%                   and To Workspace iw_log
%
%   Primary netlist (tail -> head), node X = the single star node:
%     w4 L1->X   w6 L6->L2   w5 L6->L4   w2 L5->L1   w9 L1->L6
%     w10 L1->L3 w1 L7->L1   w8 X->L8    w7 X->L9    w3 L10->X
%
%   w9x1 is the internal id of the primary topology.  It is exactly the
%   single-star-point ten-leg topology; reference B has w9 = L3 -> L6.
dosim = true; variant = 'w9x1';
for a = 1:nargin
  s = char(varargin{a});
  if strcmpi(s,'nosim'), dosim = false; else, variant = s; end
end

mdl = 'SINGLE_STAR_TEN_LEG_AMB';
outdir = b10optb_outdir('model', mfilename('fullpath'));
mfile  = fullfile(outdir,[mdl '.slx']);
bdclose('all');
if exist(mfile,'file'), delete(mfile); end
new_system(mdl); load_system(mdl);
load_system('ee_lib'); load_system('fl_lib'); load_system('nesl_utility');
S = b10optb_params(variant);             % also fills the model workspace

% ---------------------------------------------------------------- libraries
IG   = lb('ee_lib','IGBT (Ideal, Switching)',4);
DI   = lb('fl_lib','Diode',5);
CS   = lb('fl_lib','Current Sensor',5);
IN   = lb('fl_lib','Inductor',5);
RE   = lb('fl_lib','Resistor',5);
GREF = lb('fl_lib','Electrical Reference',5);
CP   = lb('nesl_utility','Connection Port',2);
SO   = lb('nesl_utility','Solver Configuration',2);
SP   = lb('nesl_utility','Simulink-PS Converter',2);
PS   = lb('nesl_utility','PS-Simulink Converter',2);
FROM = 'simulink/Signal Routing/From';
GOTO = 'simulink/Signal Routing/Goto';
RO   = 'simulink/Logic and Bit Operations/Relational Operator';
GAI  = 'simulink/Math Operations/Gain';
CON  = 'simulink/Sources/Constant';
RSEQ = 'simulink/Sources/Repeating Sequence';
MUX  = 'simulink/Signal Routing/Mux';

% ------------------------------------------------------------------ geometry
LX0 = 160; LP = 240; LW = 180; LY0 = 120; LH = 300;    % legs (row of 10)
YBP = 70;  YBN = 440;                                  % P / N bus rails
XC  = 2600; XCS = 20;                                  % routing corridor (x)
YB  = 520;  YBS = 14;                                  % routing channels (y)
XT  = 3010;                                            % star node trunk
WX  = 3100; WY0 = 820; WP = 190; WW = 320; WH = 160;   % windings (column)
MXX = 3700;                                            % Mux
MODX = 200; MODY = 950; MODP = 100;                    % PWM area

% ------------------------------------------------------------ source / earth
add_block('ee_lib/Sources/Voltage Source',[mdl '/Udc'],'Position',[20 200 70 300]);
set_param([mdl '/Udc'],'dc_voltage',num2str(S.U_dc));
add_block(GREF,[mdl '/GND'],'Position',[10 470 60 500]);
add_block(SO,[mdl '/Solver'],'Position',[10 560 60 600]);

% ------------------------------------------------------------------- legs
E = b10optb_netlist(variant);            % winding netlist of this variant
Ppos = zeros(10,2); Mpos = zeros(10,2); Npos = zeros(10,2);
for k = 1:10
  x   = LX0 + LP*(k-1);
  sub = sprintf('%s/Leg%d', mdl, k);
  add_block('simulink/Ports & Subsystems/Subsystem',sub,'Position',[x LY0 x+LW LY0+LH]);
  delete_line(sub,'In1/1','Out1/1');
  delete_block([sub '/In1']); delete_block([sub '/Out1']);
  add_block(CP,[sub '/P'],'Position',[40  70  80  90]);
  add_block(CP,[sub '/M'],'Position',[40 300  80 320]);
  add_block(CP,[sub '/N'],'Position',[40 530  80 550]);
  % Near-ideal devices: this is a topology / current-distribution model.  If it
  % were run open loop, a device drop would scale the winding currents directly
  % (Ron = 0.3 ohm on the freewheel diode alone is 30 % of the winding).  The
  % packaged build is closed loop, which compensates the drop; b10optb_devtest
  % swaps in realistic devices to verify that.
  add_block(DI,[sub '/DU'],'Position',[140 100 200 140]);  set_param([sub '/DU'],'Vf','1e-3','Ron','1e-3');
  add_block(DI,[sub '/DL'],'Position',[140 420 200 460]);  set_param([sub '/DL'],'Vf','1e-3','Ron','1e-3');
  add_block(IG,[sub '/IGU'],'Position',[300 160 380 230]); set_param([sub '/IGU'],'Vth','6','Ron','1e-3','Vf','1e-3');
  add_block(IG,[sub '/IGL'],'Position',[300 380 380 450]); set_param([sub '/IGL'],'Vth','6','Ron','1e-3','Vf','1e-3');
  % the gate tags are consumed inside this subsystem, so they must be global:
  % a "local" Goto at the model root is visible only at its own level.
  add_block(FROM,[sub '/gU'],'Position',[120 240 160 260]);
  set_param([sub '/gU'],'GotoTag',sprintf('gU%d',k));
  add_block(FROM,[sub '/gL'],'Position',[120 470 160 490]);
  set_param([sub '/gL'],'GotoTag',sprintf('gL%d',k));
  add_block(SP,[sub '/SPU'],'Position',[200 235 260 265]);
  add_block(SP,[sub '/SPL'],'Position',[200 465 260 495]);
  % ---- internal wiring (all polylines in leg-local coordinates) ----
  hP = gp(sub,'P','RConn',1);  hM = gp(sub,'M','RConn',1);  hN = gp(sub,'N','RConn',1);
  duA = gp(sub,'DU','LConn',1); duK = gp(sub,'DU','RConn',1);
  dlA = gp(sub,'DL','LConn',1); dlK = gp(sub,'DL','RConn',1);
  igC = gp(sub,'IGU','RConn',1); igE = gp(sub,'IGU','RConn',2); igG = gp(sub,'IGU','LConn',1);
  lc  = gp(sub,'IGL','RConn',1); le  = gp(sub,'IGL','RConn',2); lg  = gp(sub,'IGL','LConn',1);
  spuO = gp(sub,'SPU','RConn',1); spuI = gp(sub,'SPU','Inport',1);
  splO = gp(sub,'SPL','RConn',1); splI = gp(sub,'SPL','Inport',1);
  guO  = gp(sub,'gU','Outport',1); glO = gp(sub,'gL','Outport',1);
  wire(sub, duK, hP, zx(duK,hP,240));                 % freewheel diode K -> bus +
  wire(sub, igC, hP, zx(igC,hP,420));                 % upper IGBT collector -> bus +
  wire(sub, duA, hM, zx(duA,hM,120));                 % freewheel diode A -> midpoint
  wire(sub, igE, hM, zx(igE,hM,400));                 % upper IGBT emitter  -> midpoint
  wire(sub, lc,  hM, zx(lc ,hM,440));                 % lower IGBT collector-> midpoint
  wire(sub, dlK, hM, zx(dlK,hM,220));                 % freewheel diode K -> midpoint
  wire(sub, dlA, hN, zx(dlA,hN,120));                 % freewheel diode A -> bus -
  wire(sub, le,  hN, zx(le ,hN,420));                 % lower IGBT emitter  -> bus -
  wire(sub, guO, spuI, [pp(guO); pp(spuI)]);          % gate drive
  wire(sub, spuO, igG, zx(spuO,igG,280));
  wire(sub, glO, splI, [pp(glO); pp(splI)]);
  wire(sub, splO, lg,  zx(splO,lg,280));
  % external port positions
  ph = get_param(sub,'PortHandles');
  Ppos(k,:) = pp(ph.LConn(1)); Mpos(k,:) = pp(ph.LConn(2)); Npos(k,:) = pp(ph.LConn(3));
end

% --------------------------------------------------------------- windings
Tpos = zeros(10,2); Hpos = zeros(10,2); Ipos = zeros(10,2);
for k = 1:10
  y   = WY0 + WP*(k-1);
  sub = sprintf('%s/W%d', mdl, k);
  add_block('simulink/Ports & Subsystems/Subsystem',sub,'Position',[WX y WX+WW y+WH]);
  delete_line(sub,'In1/1','Out1/1');
  delete_block([sub '/In1']); delete_block([sub '/Out1']);
  add_block(CP,[sub '/T'],'Position',[40  70  80  90]);
  add_block(CP,[sub '/H'],'Position',[40 240  80 260]);
  add_block(CS,[sub '/CS'],'Position',[120  60 180 100]);
  add_block(IN,[sub '/IN'],'Position',[240  70 300  90]); set_param([sub '/IN'],'L','500e-6');
  add_block(RE,[sub '/RE'],'Position',[360  70 420  90]); set_param([sub '/RE'],'R','1');
  add_block(PS,[sub '/PS'],'Position',[200  10 260  40]);
  add_block('simulink/Sinks/Out1',[sub '/i'],'Position',[400 15 430 35]);
  hT = gp(sub,'T','RConn',1); hH = gp(sub,'H','RConn',1);
  csP = gp(sub,'CS','LConn',1); csI = gp(sub,'CS','RConn',1); csN = gp(sub,'CS','RConn',2);
  inP = gp(sub,'IN','LConn',1); inN = gp(sub,'IN','RConn',1);
  reP = gp(sub,'RE','LConn',1); reN = gp(sub,'RE','RConn',1);
  psI = gp(sub,'PS','LConn',1); psO = gp(sub,'PS','Outport',1);
  hI  = gp(sub,'i','Inport',1);
  wire(sub, hT,  csP, [pp(hT); pp(csP)]);             % tail -> current sensor +
  wire(sub, csI, psI, zx(csI,psI,190));               % sensor I -> PS converter
  wire(sub, csN, inP, zx(csN,inP,216));               % sensor - -> inductor
  wire(sub, inN, reP, [pp(inN); pp(reP)]);            % inductor -> resistor
  wire(sub, reN, hH,  zx(reN,hH,460));                % resistor -> head
  wire(sub, psO, hI,  [pp(psO); pp(hI)]);             % PS converter -> i outport
  ph = get_param(sub,'PortHandles');
  Tpos(k,:) = pp(ph.LConn(1)); Hpos(k,:) = pp(ph.LConn(2)); Ipos(k,:) = pp(ph.Outport(1));
end

% ------------------------------------------------------------ PWM modulator
add_block(RSEQ,[mdl '/Carrier'],'Position',[MODX MODY-110 MODX+80 MODY-70]);
set_param([mdl '/Carrier'],'rep_seq_t','[0 2.5e-5 5e-5]','rep_seq_y','[0 1 0]');
add_block(GOTO,[mdl '/GotoTri'],'Position',[MODX+140 MODY-100 MODX+180 MODY-80]);
set_param([mdl '/GotoTri'],'GotoTag','tri');
wire(mdl, gp(mdl,'Carrier','Outport',1), gp(mdl,'GotoTri','Inport',1), []);
for k = 1:10
  yr = MODY + MODP*(k-1); s = num2str(k);
  % duty comes from the closed-loop controller (Goto duty<k> inside /Ctrl)
  add_block(FROM,[mdl '/d' s],'Position',[MODX yr-12 MODX+70 yr+12]);
  set_param([mdl '/d' s],'GotoTag',sprintf('duty%d',k));
  add_block(FROM,[mdl '/tri' s],'Position',[MODX yr+28 MODX+44 yr+48]);
  set_param([mdl '/tri' s],'GotoTag','tri');
  add_block(RO,[mdl '/cu' s],'Position',[MODX+120 yr-34 MODX+190 yr-10]);
  set_param([mdl '/cu' s],'Operator','<');
  add_block(RO,[mdl '/cl' s],'Position',[MODX+120 yr+10 MODX+190 yr+34]);
  set_param([mdl '/cl' s],'Operator','>');
  add_block(GAI,[mdl '/gu' s],'Position',[MODX+230 yr-34 MODX+270 yr-10]);
  set_param([mdl '/gu' s],'Gain','15','OutDataTypeStr','double');
  add_block(GAI,[mdl '/gl' s],'Position',[MODX+230 yr+10 MODX+270 yr+34]);
  set_param([mdl '/gl' s],'Gain','15','OutDataTypeStr','double');
  add_block(GOTO,[mdl '/Gu' s],'Position',[MODX+310 yr-32 MODX+350 yr-12]);
  set_param([mdl '/Gu' s],'GotoTag',sprintf('gU%d',k),'TagVisibility','global');
  add_block(GOTO,[mdl '/Gl' s],'Position',[MODX+310 yr+12 MODX+350 yr+32]);
  set_param([mdl '/Gl' s],'GotoTag',sprintf('gL%d',k),'TagVisibility','global');
  dO = gp(mdl,['d' s],'Outport',1); tO = gp(mdl,['tri' s],'Outport',1);
  cu1 = gp(mdl,['cu' s],'Inport',1); cu2 = gp(mdl,['cu' s],'Inport',2);
  cl1 = gp(mdl,['cl' s],'Inport',1); cl2 = gp(mdl,['cl' s],'Inport',2);
  cuO = gp(mdl,['cu' s],'Outport',1); clO = gp(mdl,['cl' s],'Outport',1);
  guI = gp(mdl,['gu' s],'Inport',1);  guO = gp(mdl,['gu' s],'Outport',1);
  glI = gp(mdl,['gl' s],'Inport',1);  glO = gp(mdl,['gl' s],'Outport',1);
  guG = gp(mdl,['Gu' s],'Inport',1);  glG = gp(mdl,['Gl' s],'Inport',1);
  pdo = pp(dO); pto = pp(tO);
  pcu1 = pp(cu1); pcu2 = pp(cu2); pcl1 = pp(cl1); pcl2 = pp(cl2);
  wire(mdl, dO, cu2, [pdo; MODX+90 pdo(2); MODX+90 pcu2(2); pcu2]);
  wire(mdl, dO, cl2, [pdo; MODX+90 pdo(2); MODX+90 pcl2(2); pcl2]);
  wire(mdl, tO, cu1, [pto; MODX+60 pto(2); MODX+60 pcu1(2); pcu1]);
  wire(mdl, tO, cl1, [pto; MODX+60 pto(2); MODX+60 pcl1(2); pcl1]);
  wire(mdl, cuO, guI, [pp(cuO); pp(guI)]);
  wire(mdl, clO, glI, [pp(clO); pp(glI)]);
  wire(mdl, guO, guG, [pp(guO); pp(guG)]);
  wire(mdl, glO, glG, [pp(glO); pp(glG)]);
end

% ------------------------------------------------- closed-loop controller
b10optb_build_ctrl(mdl, S);        % 10 PI loops -> M -> duty<k> Goto tags

% -------------------------------------------------------------- bus wiring
srcP = gp(mdl,'Udc','LConn',1); srcN = gp(mdl,'Udc','RConn',1);
gnd  = gpc(mdl,'GND'); solv = gpc(mdl,'Solver');
wire(mdl, srcP, gp(mdl,'Leg1','LConn',1), zy(srcP, gp(mdl,'Leg1','LConn',1), YBP));
for k = 1:9
  a = gp(mdl,sprintf('Leg%d',k),'LConn',1);
  b = gp(mdl,sprintf('Leg%d',k+1),'LConn',1);
  wire(mdl, a, b, zy(a,b,YBP));
end
wire(mdl, srcN, gnd, zx2(srcN, gnd, 20, 400));
wire(mdl, gnd, solv, zx2(gnd, solv, 90, 520));
wire(mdl, gnd, gp(mdl,'Leg1','LConn',3), zy(gnd, gp(mdl,'Leg1','LConn',3), YBN));
for k = 1:9
  a = gp(mdl,sprintf('Leg%d',k),'LConn',3);
  b = gp(mdl,sprintf('Leg%d',k+1),'LConn',3);
  wire(mdl, a, b, zy(a,b,YBN));
end

% ------------------------------------------------- winding netlist routing
% port positions are read here, not during construction: Simulink adjusts the
% connection-port geometry of a subsystem while it is being filled, so values
% read right after add_block are stale.
nc = 0;
for k = 1:10
  tl = E{k,2}; hd = E{k,3};
  wT = gp(mdl,sprintf('W%d',k),'LConn',1); bT = pp(wT);
  wH = gp(mdl,sprintf('W%d',k),'LConn',2); bH = pp(wH);
  if ~strcmp(tl,'X')
    ia = str2double(tl(2:end));
    hM = gp(mdl,sprintf('Leg%d',ia),'LConn',2); a = pp(hM);
    nc = nc + 1; xv = XC + XCS*(nc-1); yc = YB + YBS*(nc-1);
    wire(mdl, hM, wT, [a; a(1)-25 a(2); a(1)-25 yc; xv yc; xv bT(2); bT]);
  end
  if ~strcmp(hd,'X')
    ih = str2double(hd(2:end));
    hM = gp(mdl,sprintf('Leg%d',ih),'LConn',2); b = pp(hM);
    nc = nc + 1; xv = XC + XCS*(nc-1); yc = YB + YBS*(nc-1);
    wire(mdl, wH, hM, [bH; xv bH(2); xv yc; b(1)-25 yc; b(1)-25 b(2); b]);
  end
end
% star node X: chain the four X terminals through one vertical trunk at XT
xt = [];
for k = 1:10
  if strcmp(E{k,2},'X'), xt(end+1,:) = [k 1]; end
  if strcmp(E{k,3},'X'), xt(end+1,:) = [k 2]; end
end
for j = 1:size(xt,1)-1
  if xt(j,2)   == 1, ha = gp(mdl,sprintf('W%d',xt(j,1)),'LConn',1);
  else,              ha = gp(mdl,sprintf('W%d',xt(j,1)),'LConn',2); end
  if xt(j+1,2) == 1, hb = gp(mdl,sprintf('W%d',xt(j+1,1)),'LConn',1);
  else,              hb = gp(mdl,sprintf('W%d',xt(j+1,1)),'LConn',2); end
  a = pp(ha); b = pp(hb);
  wire(mdl, ha, hb, [a; XT a(2); XT b(2); b]);
end

% ------------------------------------------------------ current metering
for k = 1:10
  s = num2str(k);
  g = [mdl '/GotoIw' s];
  pI = pp(gp(mdl,sprintf('W%d',k),'Outport',1));
  add_block(GOTO,g,'Position',[pI(1)+30 pI(2)-10 pI(1)+70 pI(2)+10]);
  % Tag by WINDING NUMBER, not by netlist row: the subsystem W<k> carries the
  % winding named E{k,1}, and every consumer (scope, controller) wants w1..w10.
  % The controller reads these from inside /Ctrl -> the tag must be global.
  set_param(g,'GotoTag',sprintf('iw%d',S.wOfRow(k)),'TagVisibility','global');
  wire(mdl, gp(mdl,sprintf('W%d',k),'Outport',1), gp(mdl,['GotoIw' s],'Inport',1), []);
end
mMid = WY0 + (9*WP + WH)/2;                 % vertical centre of the winding column
mTop = mMid - 250; mBot = mMid + 250;       % compact mux, one From right beside it
add_block(MUX,[mdl '/MuxI'],'Position',[MXX mTop MXX+20 mBot],'Inputs','10');
for k = 1:10
  s = num2str(k);
  ym = pp(gp(mdl,'MuxI','Inport',k));
  g = [mdl '/FromIw' s];
  add_block(FROM,g,'Position',[MXX-70 ym(2)-10 MXX-30 ym(2)+10]);
  set_param(g,'GotoTag',sprintf('iw%d',k));      % channel k = winding w<k>
  h = wire(mdl, gp(mdl,['FromIw' s],'Outport',1), gp(mdl,'MuxI','Inport',k), []);
  % label the signal so the Scope legend reads w1..w10
  try, set_param(h,'Name',sprintf('w%d',k)); catch, end
end
my = mBot/2 + mTop/2;
add_block('simulink/Sinks/Scope',[mdl '/Scope_Iw'],'Position',[MXX+200 my-60 MXX+260 my+20]);
add_block('simulink/Sinks/To Workspace',[mdl '/Iw_log'],'Position',[MXX+200 my+60 MXX+280 my+100]);
set_param([mdl '/Iw_log'],'VariableName','iw_log','SaveFormat','Structure With Time');
mxO = gp(mdl,'MuxI','Outport',1);
scI = gp(mdl,'Scope_Iw','Inport',1); lgI = gp(mdl,'Iw_log','Inport',1);
pmo = pp(mxO); psc = pp(scI); plg = pp(lgI);
wire(mdl, mxO, scI, [pmo; MXX+120 pmo(2); MXX+120 psc(2); psc]);
wire(mdl, mxO, lgI, [pmo; MXX+130 pmo(2); MXX+130 plg(2); plg]);

% ------------------------------------------------------------------ finish
set_param(mdl,'StopTime',num2str(S.T_stop),'Solver','VariableStepAuto', ...
          'MaxStep','1e-6','RelTol','1e-4','AbsTol','auto');
for it = 1:4, snap_lines(mdl); end     % pull line endpoints onto their ports
save_system(mdl, mfile);
fprintf('built+wired: %s   (netlist variant "%s")\n', mfile, S.variant);
n = b10optb_line_overlap(mdl, true);
ok = b10optb_check_topology(mdl, variant);
if dosim
  out = sim(mdl); y = out.iw_log; t = y.time; d = y.signals.values;
  colOfW = 1:10;      % Scope/Mux channel k IS winding w<k> (meters tag iw<winding>)
  i0 = find(t >= 0.8*t(end), 1);
  dt = diff(t(i0:end)); xx = d(i0:end,:);
  mw = (sum(xx(1:end-1,:).*dt, 1)/sum(dt));
  mw = mw(colOfW);                        % time-weighted, netlist order
  fprintf('winding currents w1..w10 [A]: '); fprintf('%.3f ', mw); fprintf('\n');
  fprintf('reference        w1..w10 [A]: '); fprintf('%.3f ', S.i_ref); fprintf('\n');
  fprintf('max error = %.4f A (%.2f %%)\n', max(abs(mw-S.i_ref)), ...
          100*max(abs(mw-S.i_ref)./S.i_ref));
  fprintf('ripple p-p [A]             : '); fprintf('%.3f ', max(xx)-min(xx)); fprintf('\n');
  if max(abs(mw)) < 1e-3
    fprintf(2, ['!! all winding currents are ~0.  The usual cause is the gate wiring:\n' ...
                '   a "local" Goto tag at the model root is NOT visible inside a\n' ...
                '   subsystem, so set TagVisibility = ''global'' on the gate Gotos\n' ...
                '   (b10optb_build_all does this) and check the dc source value.\n']);
  end
  fprintf('overlaps=%d  netlist_ok=%d\n', n, ok);
else
  fprintf('overlaps=%d  netlist_ok=%d  (simulation skipped)\n', n, ok);
end
end

% ================================================================= helpers
function p = lb(lib,nm,d)
c = find_system(lib,'LookUnderMasks','all','FollowLinks','on','SearchDepth',d,'Name',nm);
p = strrep(c{1},char(10),' ');
end

function h = gp(mdl, blk, grp, idx)
ph = get_param([mdl '/' blk],'PortHandles');
if ~isfield(ph,grp) || numel(ph.(grp)) < idx
  error('b10optb:port','%s has no %s(%d)', blk, grp, idx);
end
h = ph.(grp)(idx);
end

function p = pp(h)
p = get_param(h,'Position');
end

function P = zx(a, b, xm)
% two-bend route with a vertical middle segment at x = xm
pa = pp(a); pb = pp(b);
P = [pa; xm pa(2); xm pb(2); pb];
end

function P = zx2(a, b, xm, ym)
% source goes down to ym, then across to xm, then to b
pa = pp(a); pb = pp(b);
P = [pa; pa(1) ym; xm ym; xm pb(2); pb];
end

function P = zy(a, b, ym)
% two-bend route with a horizontal middle segment at y = ym
pa = pp(a); pb = pp(b);
P = [pa; pa(1) ym; pb(1) ym; pb];
end

function snap_lines(mdl)
%SNAP_LINES  Re-attach the drawn endpoints of every line to its block ports.
%   Simulink re-lays out subsystem connection ports while the diagram is being
%   edited, which can leave a line drawn a few pixels away from its port.
lh = find_system(mdl,'FindAll','on','Type','line');
lineIdx = containers.Map('KeyType','double','ValueType','double');
for i = 1:numel(lh), lineIdx(double(lh(i))) = i; end
portPos = containers.Map('KeyType','double','ValueType','any');
blks = find_system(mdl,'LookUnderMasks','all','FollowLinks','on','Type','Block');
for b = 1:numel(blks)
  ph = get_param(blks{b},'PortHandles');
  fn = fieldnames(ph);
  for q = 1:numel(fn)
    hs = ph.(fn{q});
    for j = 1:numel(hs)
      p = get_param(hs(j),'Position');
      ls = get_param(hs(j),'Line'); ls = ls(ls > 0);
      for t = 1:numel(ls)
        k = double(ls(t));
        if ~isKey(lineIdx, k), continue; end
        if isKey(portPos, k), portPos(k) = [portPos(k); p];
        else,                 portPos(k) = p; end
      end
    end
  end
end
ks = portPos.keys;
for t = 1:numel(ks)
  h = ks{t};
  P = get_param(h,'Points');
  if isempty(P) || size(P,1) < 2, continue; end
  pp = portPos(h); moved = false;
  for u = 1:size(pp,1)
    d1 = sum((P(1,:)  - pp(u,:)).^2);
    d2 = sum((P(end,:) - pp(u,:)).^2);
    if d1 <= d2 && d1 > 0.25
      P(1,:) = pp(u,:); moved = true;
    elseif d2 < d1 && d2 > 0.25
      P(end,:) = pp(u,:); moved = true;
    end
  end
  if moved
    keep = [true; any(diff(P,1,1) ~= 0, 2)];
    try, set_param(h,'Points',P(keep,:)); catch, end
  end
end
end

function h = wire(mdl, a, b, P)
h = add_line(mdl, a, b);
if nargin >= 4 && ~isempty(P)
  keep = [true; any(diff(P,1,1) ~= 0, 2)];   % drop repeated (degenerate) points
  P = P(keep,:);
  if size(P,1) >= 2
    try
      set_param(h,'Points',P);
    catch e
      fprintf(2,'  ! points rejected on line %g: %s\n', h, e.message);
    end
  end
end
end

function h = gpc(mdl, blk)
% first connection port of a block, whichever side it is on
ph = get_param([mdl '/' blk],'PortHandles');
c  = [ph.LConn ph.RConn];
if isempty(c), error('b10optb:port','%s has no connection port', blk); end
h = c(1);
end
