function verify_model_facts()
% Asserts every block, parameter, port and netlist edge quoted in
% references/block-and-parameter-map.md against the models bundled in model/.
% No simulation is performed. Prints a PASS/FAIL summary and raises an error
% when any check fails.
%
% Relocatable: models and the parameter script are located relative to this
% file, and every model is loaded from a copy in the system temp folder, so the
% bundled .slx files are never written to. Companion script: run_case_study.m.
%
% Verified with MATLAB R2025b (25.2), Simulink 25.2, Simscape 25.2 and
% Simscape Electrical 25.2 (Linux glnxa64). Runtime: about 30 s.

Here      = fileparts(mfilename('fullpath'));
SkillRoot = fileparts(Here);
ModelDir  = fullfile(SkillRoot,'model');
Primary   = 'Ten_phase_nine_leg_with_RL_load';
Variants  = {'Ten_phase_nine_leg_relocated_L3_L7','Ten_phase_nine_leg_relocated_L2_L8'};

tally(NaN,0);                                     % reset the counters
% Work from a scratch folder so that compiler artefacts (.slxc, slprj) never
% land in the skill folder.
work = fullfile(tempdir,'amb_b9p10_work');
if ~exist(work,'dir'), mkdir(work); end
oldDir = cd(work); restoreDir = onCleanup(@() cd(oldDir));   %#ok<NASGU>
fprintf('Skill assets: %s\n', ModelDir);

%% ---------- A. parameter script -------------------------------------------
P = loadAmbParams(fullfile(ModelDir,'parameters_for_AMB_topology.m'));
tally(P.R_value == 1,                 'R_value = 1 ohm');
tally(P.L_value == 500e-6,            'L_value = 500 uH');
tally(P.U_dc == 200,                  'U_dc = 200 V');
tally(P.bias_current == 5,            'bias_current = 5 A');
tally(P.f_c == 1500,                  'f_c = 1500 Hz');
tally(P.SW_frequency == 20e3,         'SW_frequency = 20 kHz');
tally(P.SW_Ts == 50e-6,               'SW_Ts = 50 us');
tally(P.half_SW_Ts == 25e-6,          'half_SW_Ts = 25 us');
tally(abs(P.K_p - 2*pi*P.f_c*P.L_value/P.U_dc) < 1e-12, 'K_p = 2*pi*f_c*L/U_dc');
tally(abs(P.K_i - 2*pi*P.f_c*P.R_value/P.U_dc) < 1e-12, 'K_i = 2*pi*f_c*R/U_dc');

%% ---------- B. authoritative netlist of the primary model -----------------
nets = readRootNets(fullfile(ModelDir,[Primary '.slx']));
tally(numel(nets) == 159, 'root system has 159 net elements');
assertNet(nets,'RL_load#lconn:1',  {'Bridge_module#rconn:2'},                       'w1.L on leg 1 midpoint');
assertNet(nets,'RL_load#rconn:1',  {'RL_load1#rconn:1','RL_load2#lconn:1','RL_load3#lconn:1'}, 'X1 star node members');
assertNet(nets,'RL_load1#lconn:1', {'Bridge_module1#rconn:2'},                      'w2.L on leg 2 midpoint');
assertNet(nets,'RL_load2#rconn:1', {'Bridge_module2#lconn:2'},                      'w3.R on leg 3 midpoint');
assertNet(nets,'RL_load3#rconn:1', {'Bridge_module3#lconn:2'},                      'w4.R on leg 4 midpoint');
assertNet(nets,'RL_load4#lconn:1', {'Bridge_module3#rconn:2'},                      'w5.L on leg 4 midpoint');
assertNet(nets,'RL_load4#rconn:1', {'Bridge_module4#lconn:2'},                      'w5.R on leg 5 midpoint');
assertNet(nets,'RL_load5#lconn:1', {'Bridge_module4#rconn:2'},                      'w6.L on leg 5 midpoint');
assertNet(nets,'RL_load5#rconn:1', {'Bridge_module5#lconn:2'},                      'w6.R on leg 6 midpoint');
assertNet(nets,'RL_load6#lconn:1', {'Bridge_module5#rconn:2'},                      'w7.L on leg 6 midpoint');
assertNet(nets,'RL_load6#rconn:1', {'RL_load7#rconn:1','RL_load8#lconn:1','RL_load9#lconn:1'}, 'X2 star node members');
assertNet(nets,'RL_load7#lconn:1', {'Bridge_module6#rconn:2'},                      'w8.L on leg 7 midpoint');
assertNet(nets,'RL_load8#rconn:1', {'Bridge_module7#lconn:2'},                      'w9.R on leg 8 midpoint');
assertNet(nets,'RL_load9#rconn:1', {'Bridge_module8#lconn:2'},                      'w10.R on leg 9 midpoint');
assertNet(nets,'Voltage Source#lconn:1', {'Bridge_module#lconn:1','Bridge_module1#lconn:1'}, 'DC+ rail start');
assertNet(nets,'Electrical Reference#lconn:1', {'Bridge_module#lconn:3','Bridge_module1#lconn:3', ...
        'Voltage Source#rconn:1','Solver Configuration#rconn:1'},                    'DC- rail = 0 V reference');
assertNet(nets,'Bridge_module8#rconn:1', {'Open Circuit2#lconn:1'},                  'DC+ rail end terminated');
assertNet(nets,'Bridge_module8#rconn:3', {'Open Circuit1#lconn:1'},                  'DC- rail end terminated');
assertNet(nets,'Bridge_module#lconn:2', {'Open Circuit3#lconn:1'},                   'leg 1 unused midpoint tap');
assertNet(nets,'Bridge_module7#rconn:2', {'Open Circuit5#lconn:1'},                  'leg 8 unused midpoint tap');

%% ---------- C. live model: inventory, parameters, charts, config ----------
m = loadCopy(fullfile(ModelDir,[Primary '.slx']),'amb_b9p10_facts_primary');
tally(numel(find_system(m,'SearchDepth',1,'Type','Block')) == 146, 'root block count = 146');
want = {'Constant',7;'Demux',1;'From',38;'Gain',1;'Goto',28;'Product',1;'Scope',1; ...
        'SimscapeBlock',14;'Step',3;'SubSystem',49;'Sum',3};
for i = 1:size(want,1)
    n = numel(find_system(m,'SearchDepth',1,'Type','Block','BlockType',want{i,1}));
    tally(n == want{i,2}, sprintf('root %s blocks = %d', want{i,1}, want{i,2}));
end
groups = { 'AMB_ref_gen',5; 'Bridge_module',9; 'RL_load',10; 'PI_ctrl',10; 'PWM_gen',9; 'ref_delay',3 };
for i = 1:size(groups,1)
    n = 0;
    for k = 0:groups{i,2}-1
        if k == 0, nm = groups{i,1}; else, nm = sprintf('%s%d',groups{i,1},k); end
        n = n + double(~isempty(find_system(m,'SearchDepth',1,'Name',nm)));
    end
    tally(n == groups{i,2}, sprintf('%s blocks found = %d', groups{i,1}, groups{i,2}));
end
tally(~isempty(find_system(m,'SearchDepth',1,'Name','winding_voltage_to_bridge')), 'ubridge chart block exists');
tally(~isempty(find_system(m,'SearchDepth',1,'Name','winding_currents')),          'winding_currents scope exists');
q = @(b,p) get_param([m '/' b], p);
tally(strcmp(q('Voltage Source','dc_voltage'),'U_dc'),                    'DC source driven by U_dc');
tally(strcmp(q('PWM_gen/Repeating Sequence','rep_seq_t'),'[0 half_SW_Ts SW_Ts]'), 'carrier time vector');
tally(strcmp(q('PWM_gen/Repeating Sequence','rep_seq_y'),'[0 1 0]'),       'carrier values = symmetric triangle');
tally(strcmp(q('PWM_gen/On-Off Delay','Td_on'),'1e-6'),                   'dead time 1 us (high side)');
tally(strcmp(q('PWM_gen/On-Off Delay1','Td_on'),'1e-6'),                  'dead time 1 us (low side)');
tally(strcmp(q('PI_ctrl/Saturation','UpperLimit'),'1') && strcmp(q('PI_ctrl/Saturation','LowerLimit'),'-1'), ...
      'PI output clamp = [-1 1] pu');
tally(strcmp(q('PI_ctrl/Second-Order Filter','fn'),'6000') && strcmp(q('PI_ctrl/Second-Order Filter','zeta'),'0.707'), ...
      'feedback filter 6 kHz / 0.707');
tally(strcmp(q('PI_ctrl/Rate Transition','OutPortSampleTime'),'SW_Ts'),   'feedback sampled at SW_Ts');
tally(strcmp(q('PI_ctrl/Gain4','Gain'),'K_p'),                            'PI proportional gain = K_p');
tally(strcmp(q('Gain1','Gain'),'0.5'),                                    'min-max injection gain 0.5');
tally(strcmp(q('Constant5','Value'),'0.5'),                               'injection offset constant 0.5');
tally(strcmp(q('Constant7','Value'),'[1 1 1 1 1 1 1 1 1]'),               'nine-element broadcast constant');
tally(strcmp(q('Demux','Outputs'),'9'),                                   'demux to nine legs');
tally(strcmp(q('PWM_gen/Gain','Gain'),'15') && strcmp(q('PWM_gen/Gain1','Gain'),'15'), 'gate level scaling = 15');
tally(strcmp(q('PWM_gen/Relational Operator2','Operator'),'>='),          'duty compared with >= carrier');
tally(strcmp(q('AMB_ref_gen/Gain','Gain'),'0.5'),                         'differential reference factor 0.5');
tally(strcmp(q('AMB_ref_gen/Constant','Value'),'bias_current'),           'reference bias = bias_current');
ig = find_system([m '/Bridge_module'],'SearchDepth',1,'Regexp','on','Name','IGBT');
tally(strcmp(get_param(ig{1},'Vf'),'0.8') && strcmp(get_param(ig{1},'Ron'),'0.001') && strcmp(get_param(ig{1},'Vth'),'6'), ...
      'IGBT parameters');
tally(strcmp(q('Bridge_module/Diode1','Vf'),'0.6') && strcmp(q('Bridge_module/Diode1','Ron'),'0.3'), 'diode parameters');
nDelays = 0;
for k = 0:8
    if k == 0, pwm = 'PWM_gen'; else, pwm = sprintf('PWM_gen%d',k); end
    for j = {'On-Off Delay','On-Off Delay1'}
        nDelays = nDelays + double(strcmp(get_param([m '/' pwm '/' j{1}],'Td_on'),'1e-6'));
    end
end
tally(nDelays == 18, 'all 18 dead-time blocks set to 1 us');
sf = sfroot;
ubExpect = sprintf(['function ubridge = u_w2b(uw1, uw2, uw3, uw4, uw5, uw6, uw7, uw8, uw9, uw10)\n\n' ...
 'ubias = 0;\n\nubridge = [0 0 0 0 0 0 0 0 0];\n\nubridge(5) = ubias;\n\n' ...
 'ubridge(4) = ubridge(5) + uw5;\nubridge(3) = ubridge(4) + uw4 - uw3;\n' ...
 'ubridge(2) = ubridge(4) + uw4 + uw2;\nubridge(1) = ubridge(4) + uw4 + uw1;\n\n' ...
 'ubridge(6) = ubridge(5) - uw6;\nubridge(7) = ubridge(6) - uw7 + uw8;\n' ...
 'ubridge(8) = ubridge(6) - uw7 - uw9;\nubridge(9) = ubridge(6) - uw7 - uw10;\n\n\n\nend\n']);
tally(strcmp(strtrim(sf.find('-isa','Stateflow.EMChart','Path',[m '/winding_voltage_to_bridge']).Script), strtrim(ubExpect)), ...
      'ubridge node-voltage back-substitution source');
mm = sf.find('-isa','Stateflow.EMChart','Path',[m '/MATLAB Function']).Script;
tally(~isempty(strfind(mm,'sort')) && ~isempty(strfind(mm,'cmin')) && ~isempty(strfind(mm,'cmax')), 'minmax chart source');
gt = cellfun(@(b) get_param(b,'GotoTag'), find_system(m,'SearchDepth',1,'BlockType','Goto'), 'uni', 0);
wantTags = {};
for k = 1:5, wantTags{end+1} = sprintf('i_a%d',k); wantTags{end+1} = sprintf('i_c%d',k); end
for k = 1:9, wantTags{end+1} = sprintf('pwmh%d',k); wantTags{end+1} = sprintf('pwml%d',k); end
tally(isequal(sort(gt(:)), sort(wantTags(:))), 'Goto tag set (currents + 9 gate pairs)');
tally(strcmp(get_param(m,'SimulationMode'),'accelerator'),        'SimulationMode = accelerator');
cs = getActiveConfigSet(m);
tally(strcmp(get_param(cs,'Solver'),'VariableStepAuto'),          'Solver = VariableStepAuto');
tally(strcmp(get_param(cs,'StopTime'),'0.05'),                    'StopTime = 0.05 s');
tally(strcmp(get_param(cs,'MaxStep'),'1e-6'),                     'MaxStep = 1e-6');
tally(strcmp(get_param(cs,'RelTol'),'1e-4'),                      'RelTol = 1e-4');
tally(strcmp(get_param(cs,'ZeroCrossControl'),'UseLocalSettings'),'ZeroCrossControl = UseLocalSettings');
close_system(m,0);

%% ---------- D. relocation variants ----------------------------------------
ubA = sprintf(['function ubridge = u_w2b(uw1, uw2, uw3, uw4, uw5, uw6, uw7, uw8, uw9, uw10)\n\n' ...
 'ubridge = [0 0 0 0 0 0 0 0 0];\n\nubridge(5) = 0;\n\n' ...
 'ubridge(3) = ubridge(5) + uw5;\nubridge(7) = ubridge(5) - uw6;\n\n' ...
 'ubridge(1) = ubridge(3) + uw3 + uw1;\nubridge(2) = ubridge(3) + uw3 + uw2;\nubridge(4) = ubridge(3) + uw3 - uw4;\n\n' ...
 'ubridge(6) = ubridge(7) - uw8 + uw7;\nubridge(8) = ubridge(7) - uw8 - uw9;\nubridge(9) = ubridge(7) - uw8 - uw10;\n\nend\n']);
ubB = sprintf(['function ubridge = u_w2b(uw1, uw2, uw3, uw4, uw5, uw6, uw7, uw8, uw9, uw10)\n\n' ...
 'ubridge = [0 0 0 0 0 0 0 0 0];\n\nubridge(5) = 0;\n\n' ...
 'ubridge(2) = ubridge(5) + uw5;\nubridge(8) = ubridge(5) - uw6;\n\n' ...
 'ubridge(1) = ubridge(2) - uw2 + uw1;\nubridge(3) = ubridge(2) - uw2 - uw3;\nubridge(4) = ubridge(2) - uw2 - uw4;\n\n' ...
 'ubridge(6) = ubridge(8) + uw9 + uw7;\nubridge(7) = ubridge(8) + uw9 + uw8;\nubridge(9) = ubridge(8) + uw9 - uw10;\n\nend\n']);
vNets = readRootNets(fullfile(ModelDir,[Variants{1} '.slx']));
assertNet(vNets,'RL_load4#lconn:1', {'Bridge_module2#rconn:2','Open Circuit10#lconn:1'}, 'variant A: w5.L moved to leg 3');
assertNet(vNets,'RL_load5#rconn:1', {'Bridge_module6#rconn:2','RL_load7#lconn:1'}, 'variant A: w6.R moved to leg 7');
assertNet(vNets,'RL_load3#rconn:1', {'Bridge_module3#lconn:2'}, 'variant A: leg 4 keeps w4 only');
vNets = readRootNets(fullfile(ModelDir,[Variants{2} '.slx']));
assertNet(vNets,'RL_load4#lconn:1', {'Bridge_module1#rconn:2','RL_load1#lconn:1'}, 'variant B: w5.L moved to leg 2');
assertNet(vNets,'RL_load5#rconn:1', {'Bridge_module7#lconn:2','RL_load8#rconn:1'}, 'variant B: w6.R moved to leg 8');
for i = 1:2
    v = loadCopy(fullfile(ModelDir,[Variants{i} '.slx']), sprintf('amb_b9p10_facts_v%d',i));
    src = sf.find('-isa','Stateflow.EMChart','Path',[v '/winding_voltage_to_bridge']).Script;
    if i == 1, exp = ubA; else, exp = ubB; end
    tally(strcmp(strtrim(src), strtrim(exp)), sprintf('variant %d ubridge source rewritten', i));
    tally(isempty(find_system(v,'SearchDepth',1,'Regexp','on','Name','^LOG_')), sprintf('variant %d carries no logging blocks', i));
    tally(numel(find_system(v,'SearchDepth',1,'Type','Block')) == 146, sprintf('variant %d block count = 146', i));
    close_system(v,0);
end

%% ---------- summary --------------------------------------------------------
tally(NaN,1);
end

%% ===================== local functions ====================================
function P = loadAmbParams(f)
run(f);
P = struct('R_value',R_value,'L_value',L_value,'U_dc',U_dc,'bias_current',bias_current, ...
           'f_c',f_c,'K_p',K_p,'K_i',K_i,'SW_frequency',SW_frequency,'SW_Ts',SW_Ts,'half_SW_Ts',half_SW_Ts);
end

function m = loadCopy(modelPath, name)
scratch = fullfile(tempdir,'amb_b9p10_facts_scratch');
if ~exist(scratch,'dir'), mkdir(scratch); end
target = fullfile(scratch,[name '.slx']);
if bdIsLoaded(name), close_system(name,0); end
if isfile(target), delete(target); end
copyfile(modelPath, target, 'f');
assert(isfile(target), 'failed to stage a model copy in the temp folder');
load_system(target);
m = name;
end

function nets = readRootNets(modelPath)
tmp = fullfile(tempdir,'amb_b9p10_xml');
if exist(tmp,'dir'), rmdir(tmp,'s'); end
mkdir(tmp);
unzip(modelPath, tmp);
xml = fileread(fullfile(tmp,'simulink','systems','system_root.xml'));
tags = regexp(xml, '<Block[^>]*>', 'match');
sid2name = containers.Map('KeyType','char','ValueType','char');
for i = 1:numel(tags)
    s = regexp(tags{i}, 'SID="(\d+)"', 'tokens','once');
    n = regexp(tags{i}, 'Name="([^"]*)"', 'tokens','once');
    if isempty(s) || isempty(n), continue; end
    nm = strrep(n{1}, '&#xA;', ' ');
    nm = strrep(nm, char(10), ' ');
    sid2name(s{1}) = nm;
end
ln = regexp(xml, '<Line[^>]*>(.*?)</Line>', 'tokens');
nets = cell(numel(ln),1);
for i = 1:numel(ln)
    tk = regexp(ln{i}{1}, '<P Name="(Src|Dst)">(\d+)#([^<]+)</P>', 'tokens');
    mem = cell(1,numel(tk));
    for j = 1:numel(tk)
        if isKey(sid2name, tk{j}{2}), nm = sid2name(tk{j}{2}); else, nm = ['SID' tk{j}{2}]; end
        mem{j} = sprintf('%s#%s', nm, tk{j}{3});
    end
    nets{i} = mem;
end
end

function assertNet(nets, port, expected, label)
got = {};
for i = 1:numel(nets)
    if any(strcmp(nets{i}, port))
        got = setdiff(nets{i}, {port});
        break;
    end
end
ok = isequal(sort(got(:)), sort(expected(:)));
if ~ok
    fprintf('      net of %s = {%s}\n', port, strjoin(got(:)', ', '));
    fprintf('      expected    = {%s}\n', strjoin(expected(:)', ', '));
end
tally(ok, label);
end

function tally(ok, label)
persistent nPass nFail
if isnan(ok)
    if label == 0
        nPass = 0; nFail = 0;
    else
        fprintf('\n%d checks passed, %d failed\n', nPass, nFail);
        assert(nFail == 0, '%d model-fact checks FAILED', nFail);
        fprintf('verify_model_facts: ALL PASS\n');
    end
    return
end
if ok
    nPass = nPass + 1;
else
    nFail = nFail + 1;
    fprintf('FAIL  %s\n', label);
end
end
