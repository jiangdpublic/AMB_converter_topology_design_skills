% Reproduces the numbers quoted in references/case-half-h-bridge-amb.md from the model
% bundled in assets/.
%
% Relocatable: the model and its parameter script are located relative to this
% file, and every simulation runs on an instrumented COPY in the system temp
% folder, so the bundled .slx is never modified.  The same file also works when
% it sits in a project folder that keeps the model one level up.
%
% Verified with MATLAB/Simulink R2025b, Simscape and Simscape Electrical 25.2.
% Runtime: about 20 s (two simulations, 20 ms and 35 ms of model time).
%
% Output:
%   A. steady state   - currents, duty law, ripple, spectrum, switching slopes
%   B. small-signal step of the control current
%   C. regression of A and B against the values documented in the case study

%% ---------- locate the bundled model and load its parameters ----------
Here = fileparts(mfilename('fullpath'));
ModelPath = ambFindModel(Here);
ParamsPath = fullfile(fileparts(ModelPath),'parameters_for_AMB_topology.m');
assert(isfile(ParamsPath), 'parameters_for_AMB_topology.m must sit next to the model.');
ambToolchain();
loadAmbParams(ParamsPath);   % defines R_value L_value U_dc f_c K_p K_i SW_Ts half_SW_Ts bias_current
fprintf('Model under test: %s\n', ModelPath);
fprintf(['Parameters: f_c = %g Hz, K_p = %.6g, K_i = %.6g, U_dc = %g V, bias = %g A, ' ...
    'f_sw = %g Hz\n'], f_c, K_p, K_i, U_dc, bias_current, SW_frequency);

tEndSteady = 0.02;                                  % run A stop time (s)
tEndStep   = 0.035;                                 % run B stop time (s)
stepCfg    = struct('time',0.02,'before',1,'after',0.5);
tmp        = fullfile(tempdir,'amb_half_h_case_study');

metrics = runAll(ModelPath, tmp, tEndSteady, tEndStep, stepCfg, SW_Ts);
cleanupCaseStudy(tmp);
checkAgainstDocs(metrics);
fprintf('\nDONE\n');

%% ---------------- local functions ----------------
function M = runAll(modelPath, tmp, tEndSteady, tEndStep, stepCfg, swTs)
try
    %% ---------- run A: constant reference ----------
    outA = runCase(modelPath, tmp, 'Half_H_case_a', tEndSteady, []);
    t  = outA.iw.time;
    ia = outA.iw.signals(1).values;
    ic = outA.iw.signals(2).values;
    s1 = interp1(outA.sw1_1.time, double(outA.sw1_1.signals.values)/15, t, 'previous');
    s2 = interp1(outA.sw2_1.time, double(outA.sw2_1.signals.values)/15, t, 'previous');

    w  = t(1:end-1) > 0.9*tEndSteady;               % steady-state window
    dt = diff(t); T = sum(dt(w));
    tw = @(x) sum(x(w).*dt(w))/sum(dt(w));          % time-weighted mean
    d1 = sum(s1(1:end-1).*dt.*w)/T;
    bo = sum((s1(1:end-1)>0 & s2(1:end-1)>0).*dt.*w)/T;
    bf = sum((s1(1:end-1)==0 & s2(1:end-1)==0).*dt.*w)/T;
    iaMean = tw(ia(1:end-1)); icMean = tw(ic(1:end-1));

    fprintf('\n===== A. steady state (time-weighted) =====\n');
    fprintf('i_a          = %.5f A   (reference 5.5)   error %+.2f %%\n', iaMean, 100*(iaMean-5.5)/5.5);
    fprintf('i_c          = %.5f A   (reference 4.5)   error %+.2f %%\n', icMean, 100*(icMean-4.5)/4.5);
    fprintf('i_a - i_c    = %.5f A   (reference 1.0)   error %+.2f %%\n', iaMean-icMean, 100*(iaMean-icMean-1));
    fprintf('U_cmd        = %.5f   duty(sw1) = %.4f (= 0.5 + 0.5*U = %.4f)\n', ...
        tw(outA.Ua1.signals.values(1:end-1)), d1, 0.5+0.5*tw(outA.Ua1.signals.values(1:end-1)));
    fprintf('both-on      = %.4f (= U)      both-off = %.4f\n', bo, bf);
    fprintf('PI err       = %.6f   integrator = %.5f (clamp 1.0)\n', ...
        tw(outA.err.signals.values(1:end-1)), tw(outA.integ.signals.values(1:end-1)));
    fprintf('V_avg model  = %.3f V   vs   i_a*R = %.3f V\n', bo*198.4+(1-bo)*(-3.0), iaMean);
    rippleA = max(ia(t>0.9*tEndSteady)) - min(ia(t>0.9*tEndSteady));
    rippleC = max(ia(t>0.9*tEndSteady)-ic(t>0.9*tEndSteady)) - min(ia(t>0.9*tEndSteady)-ic(t>0.9*tEndSteady));
    fprintf('ripple       = %.4f A p-p (i_a)   %.4f A p-p (i_a - i_c)\n', rippleA, rippleC);
    fprintf('samples      = %d (%.1f per switching period)\n', numel(t), numel(t)/(tEndSteady/swTs));
    ts = outA.samp.time; vs = outA.samp.signals.values;
    fprintf('sampled mean = %.5f A  vs true mean = %.5f A  -> residual %+.5f A (%+.2f %%)\n', ...
        mean(vs(ts>0.9*tEndSteady)), iaMean, mean(vs(ts>0.9*tEndSteady))-iaMean, ...
        100*(mean(vs(ts>0.9*tEndSteady))-iaMean)/iaMean);

    tu = (0.9*tEndSteady : 1e-6 : 0.975*tEndSteady)';
    iu = interp1(t, ia, tu); N = numel(iu);
    Y = abs(fft(iu-mean(iu)))/N*2; fq = (0:N-1)/(N*1e-6); [~,o] = sort(Y,'descend');
    fprintf('spectrum     : %.0f Hz (%.2f %%), %.0f Hz (%.2f %%)\n', ...
        fq(o(1)), 100*Y(o(1))/mean(iu), fq(o(2)), 100*Y(o(2))/mean(iu));

    st = s1*2+s2;
    seg = find(t > 0.95*tEndSteady & t < 0.95*tEndSteady + 150e-6);
    d = find(diff(st(seg))~=0); prev = seg(1); slopeOn = NaN;
    fprintf('state sequence / slopes:\n');
    for k = 1:min(4,numel(d))
        idx = seg(d(k));
        slope = (ia(idx)-ia(prev))/(t(idx)-t(prev))/1e6;
        fprintf('   state->%d  dur=%6.2f us  slope=%+.4f A/us\n', st(idx), (t(idx)-t(prev))*1e6, slope);
        if st(idx) == 3 && isnan(slopeOn), slopeOn = slope; end
        prev = idx;
    end

    %% ---------- run B: small-signal step of the control current ----------
    outB = runCase(modelPath, tmp, 'Half_H_case_b', tEndStep, stepCfg);
    t2 = outB.iw.time;
    d2 = outB.iw.signals(1).values - outB.iw.signals(2).values;
    tu2 = (stepCfg.time-0.001 : 1e-6 : tEndStep)';
    yu  = interp1(t2, d2, tu2);
    yk  = movmean(yu,25);
    y0  = mean(yu(tu2 < stepCfg.time - 1e-4));
    yf  = mean(yu(tu2 > tEndStep - 0.003));
    stp = abs(yf-y0);
    i0  = find(tu2 >= stepCfg.time, 1);
    a   = abs(yk-y0);
    g   = @(fr) (find(a(i0:end) >= fr*stp, 1) - 1);      % index = microseconds
    t10 = g(0.1); t63 = g(0.632); t90 = g(0.9);
    overshoot = 100*max(0,(yf-min(yk(i0:end)))/stp);
    lastBad = find(abs(yk-yf) > 0.02*stp, 1, 'last');

    fprintf('\n===== B. small-signal step i_ref %g -> %g A at %g ms =====\n', ...
        stepCfg.before, stepCfg.after, stepCfg.time*1e3);
    fprintf('from %.4f to %.4f A (step %.4f)\n', y0, yf, yf-y0);
    fprintf('t10 = %.0f us   t63 = %.0f us   t90 = %.0f us\n', t10, t63, t90);
    fprintf('overshoot = %.2f %%    settling +/-2%% = %.2f ms\n', ...
        overshoot, (tu2(lastBad)-stepCfg.time)*1e3);

    M = struct('ia',iaMean, 'ic',icMean, 'ictrl',iaMean-icMean, 'duty',d1, 'bothOn',bo, ...
        'rippleA',rippleA, 'rippleC',rippleC, 'peakFreq',fq(o(1)), 'slopeOn',slopeOn, ...
        't63',t63, 'overshoot',overshoot, 'samples',numel(t));
catch err
    cleanupCaseStudy(tmp);
    rethrow(err);
end
end

function out = runCase(modelPath, tmp, dst, tEnd, stepCfg)
% Simulate an instrumented COPY of the bundled model; the asset is untouched.
if ~exist(tmp,'dir'), mkdir(tmp); end
if bdIsLoaded(dst), close_system(dst,0); end
target = fullfile(tmp,[dst '.slx']);
if exist(target,'file'), delete(target); end
copyfile(modelPath, target, 'f');
oldFolder = pwd;
restoreFolder = onCleanup(@() cd(oldFolder));
cd(tmp);                                        % keep slprj / .slxc inside the temp folder
load_system(target);
try
    set_param([dst '/winding_currents'],'DataLogging','on','DataLoggingVariableName','iw', ...
        'DataLoggingSaveFormat','Structure With Time','DataLoggingLimitDataPoints','off', ...
        'DataLoggingDecimateData','off');
    addTw([dst '/TW_Ua1'],   'Ua1');   add_line(dst,'PI_ctrl/1','TW_Ua1/1','autorouting','on');
    addTw([dst '/TW_sw1_1'], 'sw1_1'); add_line(dst,'PWM_gen/1','TW_sw1_1/1','autorouting','on');
    addTw([dst '/TW_sw2_1'], 'sw2_1'); add_line(dst,'PWM_gen/2','TW_sw2_1/1','autorouting','on');
    sub = [dst '/PI_ctrl'];
    addTw([sub '/TW_err'],   'err');   add_line(sub,'Add2/1','TW_err/1','autorouting','on');
    addTw([sub '/TW_integ'], 'integ'); add_line(sub,'Ictrl_anti_windup/1','TW_integ/1','autorouting','on');
    addTw([sub '/TW_samp'],  'samp');  add_line(sub,'Rate Transition/1','TW_samp/1','autorouting','on');
    if ~isempty(stepCfg)
        c1 = [dst '/Constant1']; p = get_param(c1,'Position');
        delete_line(dst,'Constant1/1','AMB_ref_gen/1'); delete_block(c1);
        add_block('simulink/Sources/Step',[dst '/StepRef'],'Position',p, ...
            'Time',num2str(stepCfg.time),'Before',num2str(stepCfg.before), ...
            'After',num2str(stepCfg.after),'SampleTime','0');
        add_line(dst,'StepRef/1','AMB_ref_gen/1','autorouting','on');
    end
    sIn = Simulink.SimulationInput(dst);
    sIn = sIn.setModelParameter('StopTime', num2str(tEnd));
    t0 = tic; out = sim(sIn); elapsed = toc(t0);
    fprintf('run %s: StopTime = %g s, elapsed %.1f s\n', dst, tEnd, elapsed);
catch err
    close_system(dst,0);
    rethrow(err);
end
close_system(dst,0);
end

function addTw(blk, name)
if getSimulinkBlockHandle(blk) > 0, delete_block(blk); end
add_block('simulink/Sinks/To Workspace', blk, 'VariableName', name, ...
    'SaveFormat','Structure With Time','MaxDataPoints','inf','Position',[1200 40 1260 60]);
end

function cleanupCaseStudy(tmp)
% Unload any model still open from the temp folder, then delete the folder.
if ~exist(tmp,'dir'), return; end
files = dir(fullfile(tmp,'*.slx'));
for k = 1:numel(files)
    [~, nm] = fileparts(files(k).name);
    if bdIsLoaded(nm), close_system(nm,0); end
end
rmdir(tmp,'s');
end

function checkAgainstDocs(M)
% Regression against the values documented in references/case-half-h-bridge-amb.md.
doc = {
%   label                     measured          documented   relTol
    'i_a (A)',                M.ia,             5.507,       0.02
    'i_c (A)',                M.ic,             4.506,       0.02
    'i_a - i_c (A)',          M.ictrl,          1.0012,      0.02
    'duty(sw1)',              M.duty,           0.5212,      0.02
    'both-on duty (= U)',     M.bothOn,         0.0425,      0.05
    'ripple i_a p-p (A)',     M.rippleA,        0.4096,      0.05
    'spectrum peak (Hz)',     M.peakFreq,       39973,       0.02
    'energise slope (A/us)',  M.slopeOn,        0.3858,      0.10
    'step t63 (us)',          M.t63,            69,          0.20
    'step overshoot (%)',     M.overshoot,      8.86,        0.25
};
fprintf('\n===== C. regression against the documented case-study values =====\n');
nbad = 0;
for k = 1:size(doc,1)
    ok = abs(doc{k,2}-doc{k,3}) <= doc{k,4}*max(abs(doc{k,3}),eps);
    if ok, verdict = 'PASS'; else, verdict = 'FAIL'; nbad = nbad + 1; end
    fprintf('  %s  %-22s measured %12.5f   documented %12.5f   tol +/-%4.1f %%\n', ...
        verdict, doc{k,1}, doc{k,2}, doc{k,3}, doc{k,4}*100);
end
fprintf('%d/%d checks passed\n', size(doc,1)-nbad, size(doc,1));
if nbad > 0
    error(['%d case-study value(s) differ from the documented run. Check the MATLAB ' ...
        'release and the bundled model.'], nbad);
end
end

% ---------------- file-local helpers shared with verify_model_facts.m ----------------

function modelPath = ambFindModel(here)
% Locate the bundled model.  Accepted layouts, in order:
%   <script>/../assets/  skill layout      (scripts/ sits next to assets/)
%   <script>/..          authored layout   (model kept in the project folder)
%   <script>/            script copied next to the model
candidates = { fullfile(here,'..','assets'), fullfile(here,'..'), here };
modelPath = '';
for k = 1:numel(candidates)
    c = fullfile(candidates{k},'Half_H_Bridge_with_RL_load.slx');
    if isfile(c), modelPath = c; break; end
end
assert(~isempty(modelPath), ['Half_H_Bridge_with_RL_load.slx not found. Expected it in ' ...
    '<skill>/assets/ (i.e. next to this script''s parent folder), in the script''s ' ...
    'parent folder, or next to the script itself.']);
% collapse the '..' segments so the reported path is clean
[folder, name, ext] = fileparts(modelPath);
oldFolder = pwd; cd(folder); folder = pwd; cd(oldFolder);
modelPath = fullfile(folder, [name ext]);
end

function loadAmbParams(scriptPath)
% Run the model's parameter script and copy its variables into the base
% workspace, where the model resolves them.  Running it through a helper keeps
% THIS script's own variables alive: the parameter script starts with clearvars,
% which would otherwise wipe the paths resolved above.
[here, name, ext] = fileparts(scriptPath);
oldFolder = pwd;
restoreFolder = onCleanup(@() cd(oldFolder));
cd(here);
run([name ext]);
assignin('base','R_value',R_value);   assignin('base','L_value',L_value);
assignin('base','U_dc',U_dc);         assignin('base','f_c',f_c);
assignin('base','K_p',K_p);           assignin('base','K_i',K_i);
assignin('base','SW_frequency',SW_frequency);
assignin('base','SW_Ts',SW_Ts);       assignin('base','half_SW_Ts',half_SW_Ts);
assignin('base','bias_current',bias_current);
end

function ambToolchain()
% Echo the toolchain the case study was verified with (informational only).
V = ver;
rel = version('-release');
names = {'Simulink','Simscape','Simscape Electrical'};
parts = {sprintf('MATLAB %s', version)};
for k = 1:numel(names)
    idx = find(strcmpi({V.Name}, names{k}), 1);
    if isempty(idx)
        parts{end+1} = sprintf('%s n/a', names{k});                %#ok<AGROW>
    else
        parts{end+1} = sprintf('%s %s', names{k}, V(idx).Version); %#ok<AGROW>
    end
end
fprintf('Toolchain: %s\n', strjoin(parts, ' | '));
if ~strcmp(rel,'2025b')
    fprintf('NOTE: the documented values were verified with R2025b (%s detected).\n', rel);
end
end
