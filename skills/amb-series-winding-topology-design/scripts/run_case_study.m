function run_case_study(mode, irefArg, udcArg)
% Reproduces the numbers quoted in
%   references/case-series-winding-amb.md                      (steady state, KCL, modulation)
%   references/case-series-winding-modulation-boundary.md      (boundary endpoints)
% from the model bundled in model/.
%
% Relocatable: the model and its parameter script are located relative to this
% file, and every simulation runs on an INSTRUMENTED COPY in the system temp
% folder, so the bundled .slx is never modified.
%
% Verified with MATLAB/Simulink R2025b (25.2), Simscape and Simscape Electrical 25.2.
%
% Usage:
%   run_case_study                    baseline case, 12 ms          (about 2 min)
%   run_case_study('boundary')        baseline plus the sweep endpoints (about 8 min)
%   run_case_study('point', iref)     one operating point, U_dc from the parameters
%   run_case_study('point', iref, udc) one operating point at a given bus voltage
%
% Sections printed:
%   A. steady state - winding currents, control currents, bias, unidirectionality
%   B. leg currents - the chain KCL check I_leg(m) = i_m - i_(m-1)
%   C. modulation   - threshold b, gate duty, dead-time duty loss, spread
%   D. regression   - every metric against the values documented in the references

if nargin < 1, mode = 'baseline'; end
if nargin < 2, irefArg = 1.0; end
if nargin < 3, udcArg = []; end
Here      = fileparts(mfilename('fullpath'));
ModelPath = ambFindModel(Here);
ParamPath = fullfile(fileparts(ModelPath),'parameters_for_AMB_topology.m');
assert(isfile(ParamPath), 'parameters_for_AMB_topology.m must sit next to the model.');
P = loadAmbParams(ParamPath);
tally(NaN,0);   % start this run from zero
fprintf('Model under test : %s\n', ModelPath);
fprintf(['Parameters: f_c = %g Hz, K_p = %.6g, K_i = %.6g, U_dc = %g V, ' ...
         'bias = %g A, f_sw = %g Hz\n'], P.f_c, P.K_p, P.K_i, P.U_dc, P.bias_current, P.SW_frequency);

tmp = fullfile(tempdir,'amb_swt_case_study');
if ~exist(tmp,'dir'), mkdir(tmp); end
% Simulate from the temp folder: the accelerator writes slprj/ and a .slxc into the
% current folder, and neither should land inside the skill.
startDir = pwd;
cd(tmp);
restoreDir = onCleanup(@() cd(startDir));
m = instrumentCopy(ModelPath, tmp);
fprintf('Instrumented copy: %s\n', m);
cleanup = onCleanup(@() close_system(m,0));

switch lower(mode)
  case 'baseline'
    M = struct();
    M.A = runPoint(m, 1.0, P, tmp);
    reportSteadyState(M.A);
    reportLegs(M.A);
    reportModulation(M.A);
    checkAgainstDocs(M);

  case 'point'
    R = runPoint(m, irefArg, P, tmp, udcArg);
    reportSteadyState(R);
    reportLegs(R);
    reportModulation(R);
    checkPoint(R);

  case 'boundary'
    M = struct();
    M.A = runPoint(m, 1.0, P, tmp);
    reportSteadyState(M.A);
    reportLegs(M.A);
    reportModulation(M.A);
    fprintf('\n--- boundary sweep ---\n');
    M.B = struct();
    M.B.iref06 = runPoint(m, 0.6,  P, tmp);
    M.B.iref5  = runPoint(m, 5.0,  P, tmp);
    M.B.iref10 = runPoint(m, 10.0, P, tmp);
    M.B.udc100 = runPoint(m, 1.0,  P, tmp, 100);
    M.B.udc55  = runPoint(m, 1.0,  P, tmp, 55);
    reportBoundary(M.B);
    assignin('base','U_dc',P.U_dc);   % leave the workspace as we found it
    checkAgainstDocs(M);

  otherwise
    error('run_case_study:MODE', 'unknown mode ''%s''', mode);
end
fprintf('\nDONE\n');
end

function checkPoint(R)
% Regresses a single operating point against the documented sweep values.
fprintf('\n=== point check ===\n');
docs = { 0.6, 200,  0.5995, 0.30258;
         1.0, 200,  0.9995, 0.30258;
         5.0, 200,  4.9978, 0.30258;
        10.0, 200,  9.9957, 0.30802;
         1.0, 100,  0.9975, 0.56180;
         1.0,  55,  0.9974, 0.97218 };
hit = false;
for i = 1:size(docs,1)
  if abs(R.iref-docs{i,1}) < 1e-9 && abs(R.udc-docs{i,2}) < 1e-9
    hit = true;
    ck(sprintf('i_a - i_c at i_ref=%g A, U_dc=%g V', R.iref, R.udc), R.diff(1), docs{i,3}, 0.020);
    ck(sprintf('spread at i_ref=%g A, U_dc=%g V',    R.iref, R.udc), R.spread, docs{i,4}, 0.020);
  end
end
if ~hit
  fprintf('  (no documented value for this operating point; metrics reported above)\n');
end
[npass, nfail] = tally();
fprintf('\n%d point checks passed, %d failed\n', npass, nfail);
if nfail > 0, error('run_case_study:REGRESSION', '%d check(s) failed.', nfail); end
end

% ================= simulation =================
function R = runPoint(m, iref, P, tmp, udc) %#ok<INUSD>
if nargin < 5, udc = P.U_dc; end
assignin('base','U_dc',udc);
consts = {'Constant6','Constant1','Constant2','Constant3','Constant4'};
for k = 1:numel(consts), set_param([m '/' consts{k}], 'Value', num2str(iref)); end
set_param(m,'StopTime','0.012');
try
  set_param(m,'SimulationMode','accelerator');
catch
  set_param(m,'SimulationMode','normal');
end
fprintf('  simulating i_ref = %g A, U_dc = %g V ... ', iref, udc);
t0 = tic; out = sim(m); fprintf('%.0f s\n', toc(t0));

R = struct('iref',iref,'udc',udc);
sl = out.simlog;
t0w = 0.0105; t1w = 0.012;

% --- winding currents ---
R.ia = zeros(1,5); R.ic = zeros(1,5); R.iaMin = zeros(1,5);
allI = [];
for d = 1:5
  [tA,vA] = windingCurrent(sl, windName(2*d-1));
  [tC,vC] = windingCurrent(sl, windName(2*d));
  R.ia(d) = twm(tA,vA,t0w,t1w);
  R.ic(d) = twm(tC,vC,t0w,t1w);
  mA = tA>=t0w & tA<=t1w;
  R.iaMin(d) = min(vA(mA));
  allI = [allI; vA; vC]; %#ok<AGROW>
end
R.diff = R.ia - R.ic;
R.bias = (R.ia + R.ic)/2;
R.windingMin = min(allI);

% --- leg currents ---
R.leg = zeros(1,11);
for k = 0:10
  n = simNode(sl, legName(k));
  i1 = deviceMean(n,'IGBT_Ideal_Switching1','ice',t0w,t1w);
  i2 = deviceMean(n,'IGBT_Ideal_Switching2','ice',t0w,t1w);
  d1 = deviceMean(n,'Diode1','i',t0w,t1w);
  d2 = deviceMean(n,'Diode2','i',t0w,t1w);
  R.leg(k+1) = i1 + d2 - i2 - d1;
end
chain = reshape([R.ia; R.ic], 1, 10);          % a1,c1,a2,c2,...,a5,c5
R.legExpected = [chain(1), chain(2:10)-chain(1:9), -chain(10)];
R.kclResidual = max(abs(R.leg - R.legExpected));

% --- modulation ---
tb = out.bvec.time; B = out.bvec.signals.values;
R.bMean = zeros(1,11); R.dMean = zeros(1,11);
for j = 1:11, R.bMean(j) = twm(tb, B(:,j), t0w, t1w); end
for k = 0:10
  g = out.(sprintf('g%d',k));
  R.dMean(k+1) = twm(g.time, g.signals.values, t0w, t1w)/15;
end
msk = tb>=t0w & tb<=t1w;
R.bMin = min(B(msk,:),[],'all');
R.bMax = max(B(msk,:),[],'all');
R.spread = max(B(msk,:) - min(B(msk,:),[],2), [], 'all');
R.dutyLoss = mean(R.bMean - R.dMean);
u = B(msk,1:10) - B(msk,2:11);
R.uMax = max(abs(u),[],'all');
R.clamped = sum(abs(abs(u(:))-1) < 1e-9);
R.chainDuty = R.bMean(1) - R.bMean(11);
end

% ================= reporting =================
function reportSteadyState(R)
fprintf('\n=== A. steady state (i_ref = %g A, U_dc = %g V, window 10.5-12 ms) ===\n', R.iref, R.udc);
fprintf('  DOF      i_a        i_c     i_a-i_c      bias\n');
for d = 1:5
  fprintf('   %d  %9.4f %9.4f %9.4f %9.4f\n', d, R.ia(d), R.ic(d), R.diff(d), R.bias(d));
end
fprintf('  minimum winding current over the whole run = %.4f A\n', R.windingMin);
end

function reportLegs(R)
fprintf('\n=== B. leg currents and the chain KCL check ===\n');
fprintf('  leg    measured   i_m - i_(m-1)\n');
for k = 1:11
  fprintf('  %3d  %9.4f %12.4f\n', k-1, R.leg(k), R.legExpected(k));
end
fprintf('  largest KCL residual = %.2e A\n', R.kclResidual);
end

function reportModulation(R)
fprintf('\n=== C. modulation ===\n');
fprintf('  b_min = %.5f   b_max = %.5f   spread = %.5f\n', R.bMin, R.bMax, R.spread);
fprintf('  gate duty loss b_j - d_j = %.5f   (dead-time share Td/Ts)\n', R.dutyLoss);
fprintf('  largest |u_k| = %.4f   clamped samples = %d\n', R.uMax, R.clamped);
fprintf('  total chain duty d_0 - d_10 = %.5f\n', R.chainDuty);
end

function reportBoundary(B)
fprintf('\n=== D. boundary sweep ===\n');
fprintf('  %-20s %10s %10s %12s\n','case','i_a-i_c','spread','tracking err');
show = @(nm,R) fprintf('  %-20s %10.4f %10.5f %11.3f %%\n', nm, R.diff(1), R.spread, ...
                      (R.diff(1)-R.iref)/abs(R.iref)*100);
show('i_ref = 0.6 A', B.iref06);
show('i_ref = 5 A',   B.iref5);
show('i_ref = 10 A',  B.iref10);
show('U_dc = 100 V',  B.udc100);
show('U_dc = 55 V',   B.udc55);
end

% ================= regression against the references =================
function checkAgainstDocs(M)
fprintf('\n=== regression against the bundled case studies ===\n');
R = M.A;
ck('i_a of DOF1 (A)',            R.ia(1),      5.4980, 0.030);
ck('i_c of DOF1 (A)',            R.ic(1),      4.4990, 0.030);
ck('control current DOF1 (A)',   R.diff(1),    0.9995, 0.010);
ck('bias of DOF1 (A)',           R.bias(1),    4.9985, 0.030);
ck('leg 0 current (A)',          R.leg(1),     5.4991, 0.050);
ck('leg 1 current (A)',          R.leg(2),    -1.0000, 0.030);
ck('leg 2 current (A)',          R.leg(3),     0.9996, 0.030);
ck('leg 10 current (A)',         R.leg(11),   -4.4968, 0.050);
ck('b_min',                      R.bMin,       0.34871, 0.002);
ck('b_max',                      R.bMax,       0.65129, 0.002);
ck('modulation spread',          R.spread,     0.30258, 0.002);
ck('dead-time duty loss',        R.dutyLoss,   0.01994, 0.002);
ck('chain duty d_0 - d_10',      R.chainDuty,  0.30258, 0.003);
ckTrue('chain KCL residual < 1e-3 A', R.kclResidual < 1e-3, sprintf('%.2e A', R.kclResidual));
ckTrue('winding currents stay positive', R.windingMin > 0, sprintf('min %.4f A', R.windingMin));
ckTrue('no controller output is clamped', R.clamped == 0, sprintf('%d samples', R.clamped));
if isfield(M,'B')
  B = M.B;
  ck('boundary i_ref = 0.6 A',   B.iref06.diff(1),  0.5995, 0.010);
  ck('boundary i_ref = 5 A',     B.iref5.diff(1),   4.9978, 0.030);
  ck('boundary i_ref = 10 A',    B.iref10.diff(1),  9.9957, 0.050);
  ck('boundary spread at 10 A',  B.iref10.spread,   0.30802, 0.003);
  ck('bus boundary spread at 100 V', B.udc100.spread, 0.56180, 0.020);
  ck('bus boundary spread at 55 V',  B.udc55.spread,  0.97218, 0.020);
end
[npass, nfail] = tally();
fprintf('\n%d regression checks passed, %d failed\n', npass, nfail);
if nfail > 0
  error('run_case_study:REGRESSION', '%d regression check(s) failed.', nfail);
end
end

function ck(name, got, want, tol)
if abs(got - want) <= tol
  tally(1,0);
  fprintf('  PASS  %-36s got %10.5f   want %10.5f\n', name, got, want);
else
  tally(0,1);
  fprintf('  FAIL  %-36s got %10.5f   want %10.5f\n', name, got, want);
end
end

function ckTrue(name, cond, detail)
if cond
  tally(1,0);
  fprintf('  PASS  %-36s %s\n', name, detail);
else
  tally(0,1);
  fprintf('  FAIL  %-36s %s\n', name, detail);
end
end

function [npass, nfail] = tally(dpass, dfail)
% Single owner of the counters.  tally(NaN,0) resets them, so that a second run in
% the same MATLAB session does not keep accumulating.
persistent np nf
if isempty(np), np = 0; nf = 0; end
if nargin > 0
  if isnan(dpass)
    np = 0; nf = 0;
  else
    np = np + dpass; nf = nf + dfail;
  end
end
npass = np; nfail = nf;
end

% ================= helpers =================
function m = instrumentCopy(modelPath, tmp)
% Copies the bundled model into the temp folder and adds the To Workspace taps
% this script needs; the bundled asset is never written to.
m = 'SWT_case_study';
if bdIsLoaded(m), close_system(m,0); end
target = fullfile(tmp,[m '.slx']);
if exist(target,'file'), delete(target); end
copyfile(modelPath, target, 'f');
load_system(target);
tap(m,'Add3',1,'bvec');
tap(m,'Mux1',1,'evec');
for k = 0:10
  b = 'PWM_gen';
  if k > 0, b = sprintf('PWM_gen%d',k); end
  tap(m,b,1,sprintf('g%d',k));
end
for k = 1:10, tap(m, windName(k), 1, sprintf('iw%d',k-1)); end
set_param(m,'SimscapeLogType','All');
end

function tap(m, srcblk, srcport, varname)
twb = [m '/TW_' varname];
if getSimulinkBlockHandle(twb) ~= -1, return; end
add_block('simulink/Sinks/To Workspace', twb, 'VariableName', varname, ...
  'SaveFormat','Structure With Time', 'MaxDataPoints','inf', 'Position',[1400 40 1460 70]);
ph = get_param([m '/' srcblk],'PortHandles');
dh = get_param(twb,'PortHandles');
add_line(m, ph.Outport(srcport), dh.Inport(1), 'autorouting','on');
end

function n = simNode(sl, name)
n = sl.(name);
end

function [t, v] = windingCurrent(sl, wname)
n = simNode(sl, wname);
s = n.child('Inductor2').child('i').series;
t = s(1).time; v = s(1).values; t = t(:); v = v(:);
end

function y = deviceMean(node, child, sername, t0, t1)
s = node.child(child).child(sername).series;
t = s(1).time; v = s(1).values; t = t(:); v = v(:);
y = twm(t, v, t0, t1);
end

function y = twm(t, x, t0, t1)
% Time-weighted mean over [t0 t1]; a variable-step solver clusters samples at
% the switching edges, so a plain mean would be biased.
t = t(:); x = x(:);
msk = t >= t0 & t <= t1;
tt = t(msk); xx = x(msk);
if numel(tt) < 2, y = mean(xx); return; end
dt = diff(tt); dt = [dt; dt(end)];
y = sum(xx.*dt)/sum(dt);
end

function nm = legName(k)
if k == 0, nm = 'Bridge_module'; else, nm = sprintf('Bridge_module%d',k); end
end

function nm = windName(k)
if k == 1, nm = 'RL_load'; else, nm = sprintf('RL_load%d',k-1); end
end

function path = ambFindModel(here)
cand = dir(fullfile(fileparts(here),'model','*.slx'));
assert(~isempty(cand), 'no .slx found next to this script (expected ../model/*.slx)');
[~, idx] = max([cand.bytes]);
path = fullfile(cand(idx).folder, cand(idx).name);
end

function P = loadAmbParams(scriptPath)
% Runs the parameter script in the BASE workspace (that is where the model reads
% its variables) and returns the resolved values this script needs.
evalin('base', sprintf('run(''%s'')', strrep(scriptPath, '''', '''''')));
get_ = @(n) evalin('base', n);
P = struct('K_p', get_('K_p'), 'K_i', get_('K_i'), 'U_dc', get_('U_dc'), ...
           'bias_current', get_('bias_current'), 'SW_frequency', get_('SW_frequency'), ...
           'f_c', get_('f_c'), 'R_value', get_('R_value'), 'L_value', get_('L_value'));
end