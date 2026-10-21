% Reproduces the numbers quoted in references/case-reversed-shared-bridge-amb.md
% from the model bundled in model/.
%
% Relocatable: the model and its parameter script are located relative to this
% file, and every simulation runs on an instrumented COPY in the system temp
% folder, so the bundled .slx is never modified.
%
% Verified with MATLAB/Simulink R2025b (25.2), Simscape and Simscape Electrical 25.2.
% Runtime: about 70 s (four simulations: 0.1 s, 2 ms device level, 30 ms
% degenerate reference, 0.1 ms algebraic-loop probe).
%
% Output:
%   A. steady state  - currents, control currents, pair sums, duty law,
%                      feedback-averaging bias, shared-leg current statistics,
%                      start-up peaks, ripple spectrum
%   B. device level  - dead time, effective shared-node duty, device currents
%   C. degenerate    - i_ref = 0 (the case that aborts with a MinMax modulator)
%   D. mechanics     - algebraic-loop probe
%   E. regression    - every metric against the values documented in the case study

%% ---------- locate the bundled model and load its parameters ----------
Here = fileparts(mfilename('fullpath'));
ModelPath = ambFindModel(Here);
ParamsPath = fullfile(fileparts(ModelPath),'parameters_for_AMB_topology.m');
assert(isfile(ParamsPath), 'parameters_for_AMB_topology.m must sit next to the model.');
ambToolchain();
loadAmbParams(ParamsPath);
fprintf('Model under test: %s\n', ModelPath);
fprintf(['Parameters: f_c = %g Hz, K_p = %.6g, K_i = %.6g, U_dc = %g V, bias = %g A, ' ...
    'f_sw = %g Hz\n'], f_c, K_p, K_i, U_dc, bias_current, SW_frequency);

tmp = fullfile(tempdir,'amb_rsb_case_study');
if ~exist(tmp,'dir'), mkdir(tmp); end
m = instrumentCopy(ModelPath, tmp);
fprintf('Instrumented copy: %s\n', m);

M = struct();
M.A = runSteadyState(m, tmp);
M.B = runDeviceLevel(m, tmp);
M.C = runDegenerate(m, tmp);
M.D = runAlgebraicProbe(m, tmp);

close_system(m,0);
checkAgainstDocs(M, bias_current, SW_frequency, SW_Ts);
fprintf('\nDONE\n');

%% ---------------- run A: steady state ----------------
function M = runSteadyState(m, tmp)
fprintf('\n=== A. steady state (0.1 s, last 10 ms analysed) ===\n');
set_param(m,'StopTime','0.1','SimscapeLogType','none');
t0 = tic; out = sim(m); tel = toc(t0);
S = logMap(out);
names = windingNames();
t = S.('i_a1').t;
tEnd = t(end); tWin = tEnd - 10e-3;
tw = @(x) twm(t, x, tWin, tEnd);
sel = t >= tWin;
mtype = moduleTypes();

M = struct();
M.wall = tel; M.steps = numel(t);
M.fracSmall = mean(diff(t) < 1e-9);
for i = 1:numel(names)
    x = S.(names{i}).x;
    if mod(i,2)==1, ref = 5.5; else, ref = 4.5; end
    M.(names{i}) = struct('mean',tw(x),'ref',ref,'pp',max(x(sel))-min(x(sel)), ...
                          'min',min(x),'max',max(x));
end
M.pairDiff = zeros(1,5); M.pairSum = zeros(1,5); M.pairRipple = zeros(1,5);
for k = 1:5
    ia = S.(sprintf('i_a%d',k)).x; ic = S.(sprintf('i_c%d',k)).x; d = ia - ic;
    M.pairDiff(k) = tw(d); M.pairSum(k) = tw(ia) + tw(ic);
    M.pairRipple(k) = max(d(sel)) - min(d(sel));
end
% ---- modulation law ----
U = zeros(numel(t),10);
for i=1:10, U(:,i) = S.(sprintf('U%d',i)).x; end
sw = zeros(numel(t),11); sw(:,1) = S.sw0.x;
for i=1:10, sw(:,i+1) = S.(sprintf('sw%d',i)).x/15; end
Ueff = U; Ueff(:,6:10) = -Ueff(:,6:10);
meff = 0.5*(max(Ueff,[],2) + min(Ueff,[],2));
duty = zeros(1,11); for i=1:11, duty(i) = tw(sw(:,i)); end
M.m = tw(meff); M.dutyShared = duty(1);
% gate duty law: a T module drives its upper switch straight from the comparator
% (duty = 0.5 - m + u_eff), a B module drives its lower switch from the inverted
% comparator (duty = 1 - (0.5 - m + u_eff) = 0.5 + m + u).
predDuty = zeros(1,10);
for i=1:10
    thr = 0.5 - tw(meff) + tw(Ueff(:,i));
    if mtype(i) == 1, predDuty(i) = thr; else, predDuty(i) = 1 - thr; end
end
M.dutyErr = max(abs(duty(2:11) - predDuty));
M.dutySharedErr = abs(duty(1) - (0.5 - tw(meff)));
M.maxU = max(arrayfun(@(i) abs(tw(U(:,i))),1:10));
% ---- shared-leg current (direct measurement) ----
h = S.i_shared_h.x; l = S.i_shared_l.x; leg = l - h;
M.legMean = tw(leg); M.legRms = sqrt(tw(leg.^2)); M.legMax = max(leg(sel)); M.legMin = min(leg(sel));
M.legWholeMax = max(leg); M.legWholeMin = min(leg);
M.legStartPeak = max(leg(t < 1e-3));
iw = zeros(numel(t),1);
for i=1:10, if mtype(i)==1, iw = iw + S.(names{i}).x; else, iw = iw - S.(names{i}).x; end, end
M.kclResidual = abs(tw(leg) - tw(iw));
M.nonReversed = sum(arrayfun(@(i) tw(S.(names{i}).x), 1:10));
% ---- feedback averaging bias ----
tk = (0:wp('SW_Ts'):tEnd)'; tk = tk(tk>tWin);
xa = mean(interp1(t, S.i_a1.x, tk, 'linear'));
M.avgBiasA = xa - tw(S.i_a1.x);
tk4 = (0:wp('SW_Ts')/4:tEnd)'; tk4 = tk4(tk4>tWin & tk4<=tEnd);
x4 = interp1(t, S.i_a1.x, tk4, 'linear'); n4 = floor(numel(x4)/4)*4;
M.avg4BiasA = mean(mean(reshape(x4(1:n4),4,[]),1)) - tw(S.i_a1.x);
% ---- start-up ----
M.startPeakA = max(S.i_a1.x(t<1e-3)); M.startPeakB = max(S.i_a4.x(t<1e-3));
t63 = t(find(S.i_a1.x >= 0.63*5.5, 1)); M.t63 = t63;
% ---- ripple spectrum of one T-module winding ----
tr = (tEnd-4e-3):1e-6:tEnd;
xr = interp1(t, S.i_a1.x, tr, 'linear'); xr = xr - mean(xr);
N = numel(xr); X = abs(fft(xr))*2/N; f = (0:N-1)/(tr(2)-tr(1))/N;
[~,ix] = max(X(2:round(N/2))); M.rippleFreqT = f(ix+1); M.rippleAmpT = X(ix+1);
xr = interp1(t, S.i_a4.x, tr, 'linear'); xr = xr - mean(xr);
X = abs(fft(xr))*2/N; f = (0:N-1)/(tr(2)-tr(1))/N;
[~,ix] = max(X(2:round(N/2))); M.rippleFreqB = f(ix+1);

fprintf('  wall %.1f s, %d steps, %.1f %% below 1 ns\n', M.wall, M.steps, 100*M.fracSmall);
fprintf('  i_a1 %.4f A, i_c1 %.4f A, i_c3 %.4f A, i_a4 %.4f A\n', M.i_a1.mean, M.i_c1.mean, M.i_c3.mean, M.i_a4.mean);
fprintf('  control current: same-type pair %.4f A, split pair %.4f A\n', M.pairDiff(1), M.pairDiff(3));
fprintf('  duty law error %.2e (shared %.2e); m = %.6f\n', M.dutyErr, M.dutySharedErr, M.m);
fprintf('  shared leg: mean %.4f A, RMS %.4f A, peak %.4f A (start-up peak %.3f A)\n', M.legMean, M.legRms, M.legMax, M.legStartPeak);
fprintf('  reduction vs non-reversed (%.1f A): %.1fx mean\n', M.nonReversed, M.nonReversed/M.legMean);
fprintf('  feedback averaging bias: 1-point %+.4f A, 4-point %+.4f A\n', M.avgBiasA, M.avg4BiasA);
fprintf('  start-up peaks: i_a1 %.3f A, i_a4 %.3f A; 63%% at %.1f us\n', M.startPeakA, M.startPeakB, 1e6*M.t63);
fprintf('  ripple: T module at %.0f Hz (%.4f A), B module at %.0f Hz\n', M.rippleFreqT, M.rippleAmpT, M.rippleFreqB);
end

%% ---------------- run B: device level ----------------
function M = runDeviceLevel(m, tmp)
fprintf('\n=== B. device level (2 ms with full Simscape logging) ===\n');
set_param(m,'StopTime','0.002','SimscapeLogType','all','SimscapeLogLimitData','off');
out = sim(m);
sl = out.simlog;
M = struct();
[tv, vsh] = simNode(sl, {'IGBT_Ideal_Switching3','E','v'});
[~, gU]   = simNode(sl, {'IGBT_Ideal_Switching3','G'});
[~, gL]   = simNode(sl, {'IGBT_Ideal_Switching4','G'});
[~, i3]   = simNode(sl, {'IGBT_Ideal_Switching3','ice'});
[~, i4]   = simNode(sl, {'IGBT_Ideal_Switching4','ice'});
[~, d3]   = simNode(sl, {'Diode3','i'});
[~, d4]   = simNode(sl, {'Diode4','i'});
[~, vcc]  = simNode(sl, {'Voltage_Source','p','v'});
a = tv(1); b = tv(end);
M.deadFrac = dutyOf(tv, (gU<7.5)&(gL<7.5), a, b);
M.nodeDuty = dutyOf(tv, vsh > vcc(1)/2, a, b);
M.nodeMean = twm(tv, vsh, a, b);
M.igbt3 = twm(tv, i3, a, b); M.igbt4 = twm(tv, i4, a, b);
M.diode3 = twm(tv, d3, a, b); M.diode4 = twm(tv, d4, a, b);
M.peakShared = max([max(i3) max(i4) max(d3) max(d4)]);
M.reverseDevice = min([min(i3) min(i4) min(d3) min(d4)]);
fprintf('  dead time %.5f of the period (%.2f us), effective node duty %.5f\n', M.deadFrac, 1e6*wp('SW_Ts')*M.deadFrac, M.nodeDuty);
fprintf('  device means: IGBT3 %.4f, IGBT4 %.4f, Diode3 %.4f, Diode4 %.4f A; peak %.3f A\n', M.igbt3, M.igbt4, M.diode3, M.diode4, M.peakShared);
set_param(m,'SimscapeLogType','none','SimscapeLogLimitData','on');
end

%% ---------------- run C: degenerate reference ----------------
function M = runDegenerate(m, tmp)
fprintf('\n=== C. degenerate reference i_ref = 0 (30 ms) ===\n');
refBlocks = iRefConstants(m);
for i=1:numel(refBlocks), set_param(refBlocks{i},'Value','0'); end
set_param(m,'StopTime','0.03');
out = sim(m); S = logMap(out);
names = windingNames(); t = S.('i_a1').t;
tEnd = t(end); tWin = tEnd - 5e-3;
tw = @(x) twm(t, x, tWin, tEnd);
M = struct(); M.means = zeros(1,10); M.diffs = zeros(1,5);
for i=1:10, M.means(i) = tw(S.(names{i}).x); end
for k=1:5, M.diffs(k) = tw(S.(sprintf('i_a%d',k)).x) - tw(S.(sprintf('i_c%d',k)).x); end
M.legMean = tw(S.i_shared_l.x - S.i_shared_h.x);
M.legPeak = max(S.i_shared_l.x - S.i_shared_h.x);
M.minCurrent = min(cellfun(@(n) min(S.(n).x), names));
M.completed = true;
for i=1:numel(refBlocks), set_param(refBlocks{i},'Value','1'); end
fprintf('  completed; currents %s A\n', mat2str(M.means,6));
fprintf('  max |control current| %.2e A; shared leg mean %+.4f A, peak %.3f A\n', max(abs(M.diffs)), M.legMean, M.legPeak);
end

%% ---------------- run D: algebraic-loop probe ----------------
function M = runAlgebraicProbe(m, tmp)
fprintf('\n=== D. algebraic-loop probe ===\n');
old = get_param(m,'AlgebraicLoopMsg');
set_param(m,'AlgebraicLoopMsg','error','StopTime','1e-4');
M = struct(); M.ok = true; M.msg = '';
try
    sim(m);
catch err
    M.ok = false; M.msg = regexprep(err.message,'<[^>]*>','');
end
set_param(m,'AlgebraicLoopMsg',old,'StopTime','0.1');
fprintf('  no algebraic loop: %d\n', M.ok);
end

%% ---------------- regression against the documented values ----------------
function checkAgainstDocs(M, bias, fsw, Ts)
A = M.A; B = M.B; C = M.C; D = M.D;
fprintf('\n=== E. regression against references/case-reversed-shared-bridge-amb.md ===\n');
np = 0; nf = 0;
    function ck(cond, fmt, varargin)
        if cond
            np = np + 1;
        else
            nf = nf + 1;
            fprintf('  FAIL: %s\n', sprintf(fmt, varargin{:}));
        end
    end
    function ckRel(got, want, tol, label)
        ck(abs(got-want) <= tol, '%s: %.5f (documented %.5f, tol %.5f)', label, got, want, tol);
    end
% winding currents
ckRel(A.i_a1.mean, 5.5108, 0.02, 'i_a1 mean');
ckRel(A.i_c1.mean, 4.5108, 0.02, 'i_c1 mean');
ckRel(A.i_c3.mean, 4.4886, 0.02, 'i_c3 mean');
ckRel(A.i_a4.mean, 5.4886, 0.02, 'i_a4 mean');
ck(all(abs([A.i_a1.mean A.i_c1.mean A.i_c3.mean A.i_a4.mean] - [5.5 4.5 4.5 5.5]) < 0.05), 'all winding currents within 1 % of their references');
% pairs
ckRel(A.pairDiff(1), 1.0000, 0.01, 'same-type pair control current');
ckRel(A.pairDiff(3), 1.0222, 0.02, 'split-pair control current');
ckRel(A.pairSum(1), 10.0216, 0.05, 'T-T pair sum');
ckRel(A.pairSum(4), 9.9772, 0.05, 'B-B pair sum');
ckRel(A.pairRipple(1), 0.0605, 0.01, 'same-type differential ripple');
ckRel(A.pairRipple(3), 0.5490, 0.05, 'split-pair differential ripple');
ck(A.pairRipple(3) > 5*A.pairRipple(1), 'the split pair ripple is much larger than a same-type pair');
% modulation
ck(A.dutyErr < 1e-6, 'winding duty law holds to 1e-6 (error %.2e)', A.dutyErr);
ck(A.dutySharedErr < 1e-6, 'shared-leg duty law holds to 1e-6 (error %.2e)', A.dutySharedErr);
ckRel(A.dutyShared, 0.479190, 1e-6, 'shared-leg command duty');
ckRel(A.m, 0.020810, 1e-6, 'common-mode injection m');
ck(A.maxU < 1, 'controller outputs inside the clamp (max %.4f)', A.maxU);
% currents: unidirectional and matched
ck(min([A.i_a1.min A.i_c1.min A.i_c3.min A.i_a4.min]) >= -1e-6, 'winding currents never negative');
for i=1:5, ck(abs(A.pairSum(i) - 2*bias) < 0.05, 'pair %d sum within 0.5 %% of 2*bias', i); end
% shared leg
ckRel(A.legMean, 1.1111, 0.02, 'shared-leg mean current');
ckRel(A.legRms, 1.3825, 0.05, 'shared-leg RMS current');
ckRel(A.legMax, 2.6397, 0.15, 'shared-leg peak current');
ck(A.legMin > -0.05, 'shared-leg current is unipolar in steady state (min %.4f)', A.legMin);
ckRel(A.nonReversed, 10*bias, 0.2, 'non-reversed baseline is 10*bias');
ck(A.nonReversed/A.legMean > 40, 'shared-leg mean stress reduced by more than 40x (%.1fx)', A.nonReversed/A.legMean);
ck(A.kclResidual < 1e-9, 'shared-leg KCL residual < 1e-9 (%.2e)', A.kclResidual);
ckRel(A.legStartPeak, 6.487, 0.3, 'shared-leg start-up peak');
% feedback averaging
ck(abs(A.avgBiasA) > 0.02, 'single-point sampling shows the documented bias (%+.4f A)', A.avgBiasA);
ck(abs(A.avg4BiasA) < 0.02, '4-sample averaging bias below 0.02 A (%+.4f A)', A.avg4BiasA);
% start-up
ckRel(A.startPeakA, 5.566, 0.15, 'i_a1 start-up peak');
ckRel(A.startPeakB, 5.708, 0.15, 'i_a4 start-up peak');
ck(A.startPeakA < 1.05*5.5, 'i_a1 start-up overshoot below 5 %% (%.1f %%)', 100*(A.startPeakA/5.5-1));
ckRel(1e6*A.t63, 86, 20, '63 %% rise time [us]');
% ripple
ckRel(A.rippleFreqT, 2*fsw, 2000, 'T-module ripple frequency');
ckRel(A.rippleFreqB, fsw, 2000, 'B-module ripple frequency');
ckRel(A.rippleAmpT, 0.1260, 0.02, 'T-module ripple amplitude');
% solver
ck(A.steps > 5e5 && A.steps < 3e6, 'major step count plausible (%d)', A.steps);
ck(A.fracSmall > 0.2 && A.fracSmall < 0.9, 'sub-nanosecond step fraction plausible (%.1f %%)', 100*A.fracSmall);
% device level
ckRel(B.deadFrac, 0.040, 0.005, 'dead-time fraction');
ckRel(B.nodeDuty, 0.50081, 0.01, 'effective shared-node duty');
ck(B.peakShared < 7, 'shared-leg device peak below 7 A (%.3f A)', B.peakShared);
ck(B.reverseDevice > -1e-3, 'no reverse device current (min %.2e A)', B.reverseDevice);
% degenerate
ck(C.completed, 'i_ref = 0 run completes (no zero-crossing abort)');
ck(max(abs(C.means - bias)) < 0.02, 'i_ref = 0 settles on the bias current (max error %.4f A)', max(abs(C.means - bias)));
ck(max(abs(C.diffs)) < 0.01, 'i_ref = 0 control currents are zero');
ck(abs(C.legMean) < 0.05, 'i_ref = 0 shared-leg mean current is zero (%+.4f A)', C.legMean);
ck(C.minCurrent >= -1e-6, 'i_ref = 0 run keeps the winding currents non-negative');
% mechanics
ck(D.ok, 'no algebraic loop (%s)', D.msg);
fprintf('\n%d checks passed, %d failed\n', np, nf);
if nf > 0
    error('run_case_study: %d regression checks failed', nf);
end
fprintf('ALL CHECKS PASSED\n');
end

%% ---------------- local helpers ----------------
function v = wp(name)
% Read one of the eight base-workspace parameters defined by parameters_for_AMB_topology.m
v = evalin('base', name);
end

function path = ambFindModel(here)
cands = { fullfile(here,'..','model'), fullfile(here,'..','assets'), fullfile(here,'..'), here };
for i=1:numel(cands)
    f = dir(fullfile(cands{i},'Reversed_shared_Bridge_with_RL_load.slx'));
    if ~isempty(f)
        path = fullfile(f(1).folder,f(1).name);
        return;
    end
end
error('Reversed_shared_Bridge_with_RL_load.slx not found next to this script.');
end

function loadAmbParams(scriptPath)
run(scriptPath);
end

function ambToolchain()
assert(~isempty(ver('simscape')), 'Simscape is required to run the RSB verification scripts.');
end

function m = instrumentCopy(modelPath, tmp)
% Byte-identical copy under a new name + logging marks on the interesting signals.
m = 'RSB_case_study';
if bdIsLoaded(m), close_system(m,0); end
target = fullfile(tmp,[m '.slx']);
if exist(target,'file'), delete(target); end
copyfile(modelPath, target, 'f');
load_system(target);
set_param(m,'SignalLogging','on','SignalLoggingName','logsout');
tags = [{'sw0'}, arrayfun(@(k) sprintf('sw%d',k), 1:10, 'uni', 0), ...
        {'i_shared_h','i_shared_l'}, windingNames()];
fr = find_system(m,'LookUnderMasks','all','FollowLinks','on','BlockType','From');
n = 0;
for i=1:numel(fr)
    try
        tg = get_param(fr{i},'GotoTag');
        if any(strcmp(tg,tags))
            lh = get_param(fr{i},'LineHandles');
            if lh.Outport(1) > 0
                set_param(lh.Outport(1),'Name',tg);
                Simulink.sdi.markSignalForStreaming(get_param(fr{i},'PortHandles').Outport(1),'on');
                n = n + 1;
            end
        end
    catch
    end
end
for k=1:10
    b = sprintf('%s/PI_ctrl%d', m, k);
    ph = get_param(b,'PortHandles');
    set_param(get_param(ph.Outport(1),'Line'),'Name',sprintf('U%d',k));
    Simulink.sdi.markSignalForStreaming(ph.Outport(1),'on');
end
fprintf('  marked %d tag signals plus 10 controller outputs\n', n);
end

function S = logMap(out)
S = struct();
el = out.logsout;
for i=1:el.numElements
    e = el{i}; nm = strtrim(e.Name);
    if isempty(nm) || isfield(S,nm), continue; end
    S.(nm) = struct('t', e.Values.Time, 'x', squeeze(e.Values.Data));
end
end

function names = windingNames()
names = {'i_a1','i_c1','i_a2','i_c2','i_a3','i_c3','i_a4','i_c4','i_a5','i_c5'};
end

function mt = moduleTypes()
mt = [1 1 1 1 1 0 0 0 0 0];   % 1 = T module, 0 = B module (order of windingNames)
end

function refs = iRefConstants(m)
refs = {};
for k = 1:5
    if k == 1, blk = [m '/AMB_ref_gen']; else, blk = sprintf('%s/AMB_ref_gen%d', m, k-1); end
    src = '';
    pc = get_param(blk,'PortConnectivity');
    for i=1:numel(pc)
        v = pc(i).SrcBlock; if iscell(v), v = [v{:}]; end
        for q=1:numel(v)
            if v(q) > 0
                nm = get_param(v(q),'Name');
                if contains(nm,'Constant'), src = getfullname(v(q)); end
            end
        end
    end
    assert(~isempty(src), 'control-current reference constant for %s not found', blk);
    refs{end+1} = src;
end
end

function [t, v] = simNode(sl, path)
n = sl;
for i=1:numel(path), n = n.(path{i}); end
t = n.series.time; v = double(n.series.values); v = v(:);
end

function m = twm(t, x, t0, t1)
sel = t >= t0 & t <= t1;
m = trapz(t(sel), x(sel))/(t1-t0);
end

function d = dutyOf(t, mask, t0, t1)
sel = t >= t0 & t <= t1;
d = trapz(t(sel), double(mask(sel)))/(t1-t0);
end