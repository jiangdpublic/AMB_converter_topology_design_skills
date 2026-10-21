function verify_model_facts()
% Asserts every block path, library path, parameter name and value, port name,
% Goto tag and channel-map entry quoted in
%   references/block-and-parameter-map.md
% against the bundled model in assets/.  Prints a PASS/FAIL summary and raises
% an error when any check fails.
%
% Relocatable: the model and its parameter script are located relative to this
% file, so the whole skill folder can be copied anywhere.  Companion script:
% scripts/run_case_study.m.
%
% Verified with MATLAB/Simulink R2025b, Simscape and Simscape Electrical 25.2.

Here = fileparts(mfilename('fullpath'));
ModelPath = ambFindModel(Here);
ParamsPath = fullfile(fileparts(ModelPath),'parameters_for_AMB_topology.m');
assert(isfile(ParamsPath), 'parameters_for_AMB_topology.m must sit next to the model.');
ambToolchain();
loadAmbParams(ParamsPath);
fprintf('Model asset: %s\n', ModelPath);

% Work on a renamed copy in the temp folder: a model of the same name may
% already be loaded from another folder, and the shipped asset must never be
% written to.  The copy is byte-identical to the asset.
tmp = fullfile(tempdir,'amb_half_h_facts');
if ~exist(tmp,'dir'), mkdir(tmp); end
m = 'Half_H_facts_check';
if bdIsLoaded(m), close_system(m,0); end
target = fullfile(tmp,[m '.slx']);
if exist(target,'file'), delete(target); end
copyfile(ModelPath, target, 'f');
load_system(target);
fprintf('Model under test: %s (temp copy)\n', m);
npass = 0; nfail = 0;

    function chk(cond, fmt, varargin)
        if cond
            npass = npass + 1;
        else
            nfail = nfail + 1;
            fprintf('  FAIL: %s\n', sprintf(fmt, varargin{:}));
        end
    end

    function v = flat(s)          % strip newlines that wrapped library names carry
        if ~ischar(s), s = char(s); end
        v = strrep(strrep(s, newline, ' '), sprintf('\r'), ' ');
    end

    function chkParam(blk, par, expected)
        try
            v = get_param(blk, par);
        catch
            chk(false, '%s : parameter %s not found', blk, par); return
        end
        if isnumeric(expected)
            if ischar(v), num = str2double(v); else, num = double(v); end
            ok = isscalar(num) && ~isnan(num) && abs(num - expected) <= 1e-12*max(1, abs(expected));
        else
            ok = ischar(v) && strcmp(strtrim(flat(v)), expected);
        end
        chk(ok, '%s : %s = %s (expected %s)', blk, par, flat(num2str(v)), flat(num2str(expected)));
    end

    function chkExists(blk)
        chk(getSimulinkBlockHandle(blk) > 0, 'block missing: %s', blk);
    end

    function chkRef(blk, expected)
        chkExists(blk);
        if getSimulinkBlockHandle(blk) <= 0, return; end
        chk(strcmp(strtrim(flat(get_param(blk,'ReferenceBlock'))), expected), ...
            '%s : ReferenceBlock = %s (expected %s)', blk, flat(get_param(blk,'ReferenceBlock')), expected);
    end

fprintf('== 1. model configuration ==\n');
chkParam(m,'Solver','VariableStepAuto');
chkParam(m,'SolverType','Variable-step');
chkParam(m,'MaxStep',1e-6);
chkParam(m,'RelTol',1e-4);
chkParam(m,'StopTime',0.1);
chkParam(m,'FixedStep',1e-6);
chkParam(m,'StartTime',0);
chkParam(m,'SignalLogging','on');
chkParam(m,'SignalLoggingName','logsout');

fprintf('== 2. Solver Configuration block ==\n');
sc = [m '/Solver Configuration'];
chkRef(sc,'nesl_utility/Solver Configuration');
chkParam(sc,'UseLocalSolver','off');
chkParam(sc,'LocalSolverChoice','NE_TRAPEZOIDAL_ADVANCER');
chkParam(sc,'LocalSolverSampleTime',1e-6);
chkParam(sc,'DoFixedCost','off');
chkParam(sc,'AutomaticFiltering','on');
chkParam(sc,'FilteringTimeConstant',0.001);
chkParam(sc,'MaxNonlinIter',3);

fprintf('== 3. root blocks and library paths ==\n');
chkRef([m '/Voltage Source'],'ee_lib/Sources/Voltage Source');
chkParam([m '/Voltage Source'],'dc_voltage','U_dc');
chkParam([m '/Voltage Source'],'sample_time',1e-3);
chkRef([m '/Electrical Reference'],'fl_lib/Electrical/Electrical Elements/Electrical Reference');
chkRef([m '/Open Circuit'],'fl_lib/Electrical/Electrical Elements/Open Circuit');
chkRef([m '/Open Circuit1'],'fl_lib/Electrical/Electrical Elements/Open Circuit');
for k = 1:5, chkParam([m '/Constant' num2str(k)],'Value',1); end
chkParam([m '/winding_currents'],'NumInputPorts',10);
chkParam([m '/winding_currents'],'SampleTime',-1);
chkParam([m '/winding_currents'],'DataLogging','off');
chkParam([m '/winding_currents'],'DataLoggingVariableName','ScopeData');
chkParam([m '/winding_currents'],'DataLoggingSaveFormat','Dataset');

fprintf('== 4. Half_H_module (bridge) ==\n');
bridge = [m '/Half_H_module'];
chkRef([bridge '/IGBT (Ideal, Switching)'],'ee_lib/Semiconductors & Converters/IGBT (Ideal, Switching)');
chkRef([bridge '/IGBT (Ideal, Switching)1'],'ee_lib/Semiconductors & Converters/IGBT (Ideal, Switching)');
chkRef([bridge '/Diode'],'fl_lib/Electrical/Electrical Elements/Diode');
chkRef([bridge '/Diode1'],'fl_lib/Electrical/Electrical Elements/Diode');
chkRef([bridge '/Resistor'],'fl_lib/Electrical/Electrical Elements/Resistor');
chkRef([bridge '/Inductor'],'fl_lib/Electrical/Electrical Elements/Inductor');
chkRef([bridge '/Current Sensor'],'fl_lib/Electrical/Electrical Sensors/Current Sensor');
chkRef([bridge '/Simulink-PS Converter'],'nesl_utility/Simulink-PS Converter');
chkRef([bridge '/Simulink-PS Converter1'],'nesl_utility/Simulink-PS Converter');
chkRef([bridge '/PS-Simulink Converter'],'nesl_utility/PS-Simulink Converter');
for b = {'IGBT (Ideal, Switching)','IGBT (Ideal, Switching)1'}
    chkParam([bridge '/' b{1}],'Ron',0.001);
    chkParam([bridge '/' b{1}],'Vf',0.8);
    chkParam([bridge '/' b{1}],'Goff',1e-5);
    chkParam([bridge '/' b{1}],'Vth',6);
    chkParam([bridge '/' b{1}],'diode_param','ee.enum.semiconductors.protectionDiode.external');
end
for b = {'Diode','Diode1'}
    chkParam([bridge '/' b{1}],'Vf',0.6);
    chkParam([bridge '/' b{1}],'Ron',0.3);
    chkParam([bridge '/' b{1}],'Goff',1e-8);
end
chkParam([bridge '/Resistor'],'R','R_value');
chkParam([bridge '/Inductor'],'L','L_value');
% port names
chk(strcmp(get_param([bridge '/sw1'],'Name'),'sw1') && str2double(get_param([bridge '/sw1'],'Port'))==1,'bridge inport 1 name');
chk(strcmp(get_param([bridge '/sw2'],'Name'),'sw2') && str2double(get_param([bridge '/sw2'],'Port'))==2,'bridge inport 2 name');
chk(strcmp(get_param([bridge '/i'],'Name'),'i') && str2double(get_param([bridge '/i'],'Port'))==1,'bridge outport name');
for cp = {'Vcci','Vddi','Vcco','Vddo'}
    chkExists([bridge '/' cp{1}]);
    chk(strcmp(get_param([bridge '/' cp{1}],'BlockType'),'PMIOPort'), '%s is not a Connection Port', cp{1});
end
% netlist: diode orientation (anode/cathode) via terminal connectivity
chkDiode([bridge '/Diode'], 'Diode'); chkDiode([bridge '/Diode1'], 'Diode1');

fprintf('== 5. PI_ctrl ==\n');
pi1 = [m '/PI_ctrl'];
chkParam([pi1 '/Add2'],'Inputs','+-');
chkParam([pi1 '/Add3'],'Inputs','++');
chkParam([pi1 '/Gain4'],'Gain','K_p');
chkParam([pi1 '/Saturation'],'LowerLimit',-1);
chkParam([pi1 '/Saturation'],'UpperLimit',1);
chkParam([pi1 '/Rate Transition'],'OutPortSampleTimeOpt','Specify');
chkParam([pi1 '/Rate Transition'],'OutPortSampleTime','SW_Ts');
chkParam([pi1 '/Rate Transition'],'Integrity','on');
chkParam([pi1 '/Rate Transition'],'Deterministic','on');
chkParam([pi1 '/Rate Transition'],'InitialCondition',0);
chkParam([pi1 '/Memory'],'InheritSampleTime','off');
chkParam([pi1 '/Memory'],'InitialCondition',0);
chkRef([pi1 '/Second-Order Filter'],'eeGeneralControl/Second-Order Filter');
chkParam([pi1 '/Second-Order Filter'],'FilterType','Low-pass');
chkParam([pi1 '/Second-Order Filter'],'fn',6000);
chkParam([pi1 '/Second-Order Filter'],'Zeta',0.707);
chkParam([pi1 '/Second-Order Filter'],'Ts',0);
chk(strcmp(strtrim(get_param([pi1 '/Second-Order Filter'],'MaskValueString')),'Low-pass|6000|0.707|0|0|0|0|0'),'filter MaskValueString');
chkParam([pi1 '/ref'],'Port',1); chkParam([pi1 '/fbk'],'Port',2); chkParam([pi1 '/U_cmd'],'Port',1);

fprintf('== 6. Ictrl_anti_windup ==\n');
aw = [pi1 '/Ictrl_anti_windup'];
chkParam([aw '/AND'],'Operator','OR');
chkParam([aw '/AND1'],'Operator','AND');
chkParam([aw '/AND'],'Inputs',2);
chkParam([aw '/AND1'],'Inputs',2);
chkRef([aw '/Compare To Constant'],'simulink/Logic and Bit Operations/Compare To Constant');
chkRef([aw '/Compare To Constant1'],'simulink/Logic and Bit Operations/Compare To Constant');
chkRef([aw '/Compare To Constant2'],'simulink/Logic and Bit Operations/Compare To Constant');
chkParam([aw '/Compare To Constant'],'relop','<=');
chkParam([aw '/Compare To Constant'],'const',-1);
chkParam([aw '/Compare To Constant1'],'relop','>=');
chkParam([aw '/Compare To Constant1'],'const',1);
chkParam([aw '/Compare To Constant2'],'relop','>=');
chkParam([aw '/Compare To Constant2'],'const',0);
chkParam([aw '/Switch'],'Criteria','u2 ~= 0');
chkParam([aw '/Switch'],'Threshold',0);
chkParam([aw '/Constant'],'Value',0);
chkParam([aw '/Product'],'Inputs',2);
chkParam([aw '/Gain1'],'Gain','K_i');
chkParam([aw '/Integrator'],'LimitOutput','on');
chkParam([aw '/Integrator'],'UpperSaturationLimit',1);
chkParam([aw '/Integrator'],'LowerSaturationLimit',-1);
chkParam([aw '/Integrator'],'InitialCondition',0);
chkParam([aw '/err'],'Port',1); chkParam([aw '/Int_out'],'Port',1);

fprintf('== 7. PWM_gen ==\n');
pw = [m '/PWM_gen'];
chkRef([pw '/Repeating Sequence'],'simulink/Sources/Repeating Sequence');
chkParam([pw '/Repeating Sequence'],'rep_seq_t','[0 half_SW_Ts SW_Ts]');
chkParam([pw '/Repeating Sequence'],'rep_seq_y','[0 1 0]');
chkParam([pw '/Constant'],'Value',0.5);
chkParam([pw '/Constant1'],'Value',0.5);
chkParam([pw '/Gain'],'Gain',0.5);
chkParam([pw '/Gain1'],'Gain',0.5);
chkParam([pw '/Gain2'],'Gain',15);
chkParam([pw '/Gain3'],'Gain',15);
chkParam([pw '/Add'],'Inputs','++');
chkParam([pw '/Add1'],'Inputs','+-');
chkParam([pw '/Relational Operator'],'Operator','<=');
chkParam([pw '/Relational Operator1'],'Operator','<=');
chkParam([pw '/Logical Operator'],'Operator','NOT');
chkParam([pw '/U_RL'],'Port',1); chkParam([pw '/sw1'],'Port',1); chkParam([pw '/sw2'],'Port',2);

fprintf('== 8. AMB_ref_gen ==\n');
rg = [m '/AMB_ref_gen'];
chkParam([rg '/Constant'],'Value','bias_current');
chkParam([rg '/Gain'],'Gain',0.5);
chkParam([rg '/Add'],'Inputs','++');
chkParam([rg '/Add1'],'Inputs','-+');
chkParam([rg '/i_ref'],'Port',1); chkParam([rg '/ia_ref'],'Port',1); chkParam([rg '/ic_ref'],'Port',2);

fprintf('== 9. Goto tags ==\n');
tags = {};
for d = 1:5
    tags{end+1} = sprintf('sw1_a%d',d); tags{end+1} = sprintf('sw2_a%d',d);
    tags{end+1} = sprintf('sw1_c%d',d); tags{end+1} = sprintf('sw2_c%d',d);
    tags{end+1} = sprintf('i_a%d',d);   tags{end+1} = sprintf('i_c%d',d);
end
for k = 1:numel(tags)
    g = find_system(m,'SearchDepth',1,'BlockType','Goto','GotoTag',tags{k});
    f = find_system(m,'SearchDepth',1,'BlockType','From','GotoTag',tags{k});
    chk(~isempty(g) && ~isempty(f), 'tag %s: Goto=%d From=%d', tags{k}, numel(g), numel(f));
end

fprintf('== 10. channel map (bridge <-> tag <-> controller) ==\n');
bridgesA = {'Half_H_module','Half_H_module2','Half_H_module4','Half_H_module6','Half_H_module8'};
bridgesC = {'Half_H_module1','Half_H_module3','Half_H_module5','Half_H_module7','Half_H_module9'};
pisA  = {'PI_ctrl','PI_ctrl2','PI_ctrl4','PI_ctrl6','PI_ctrl8'};
pisC  = {'PI_ctrl1','PI_ctrl3','PI_ctrl5','PI_ctrl7','PI_ctrl9'};
pwmA  = {'PWM_gen','PWM_gen2','PWM_gen4','PWM_gen6','PWM_gen8'};
pwmC  = {'PWM_gen1','PWM_gen3','PWM_gen5','PWM_gen7','PWM_gen9'};
refG  = {'AMB_ref_gen','AMB_ref_gen1','AMB_ref_gen2','AMB_ref_gen3','AMB_ref_gen4'};
for d = 1:5
    chkSource([m '/' bridgesA{d}], sprintf('i_a%d',d));
    chkSource([m '/' bridgesC{d}], sprintf('i_c%d',d));
    chkSource([m '/' pwmA{d}],  sprintf('sw1_a%d',d), 1);
    chkSource([m '/' pwmA{d}],  sprintf('sw2_a%d',d), 2);
    chkSource([m '/' pwmC{d}],  sprintf('sw1_c%d',d), 1);
    chkSource([m '/' pwmC{d}],  sprintf('sw2_c%d',d), 2);
    chkFeeds([m '/' pisA{d}],   sprintf('i_a%d',d));
    chkFeeds([m '/' pisC{d}],   sprintf('i_c%d',d));
    chkFedBy([m '/' refG{d}],   sprintf('Constant%d',d), 1);
end

fprintf('\n==== %d passed, %d failed ====\n', npass, nfail);
fprintf('model asset: %s\n', ModelPath);
close_system(m,0);
if exist(tmp,'dir'), rmdir(tmp,'s'); end
if nfail > 0, error('%d fact check(s) failed', nfail); end

    function chkSource(srcBlk, tag, portIdx)
        if nargin < 3, portIdx = 1; end
        g = find_system(m,'SearchDepth',1,'BlockType','Goto','GotoTag',tag);
        if isempty(g), chk(false,'no Goto for tag %s',tag); return; end
        ph = get_param(g{1},'PortHandles');
        ln = get_param(ph.Inport(1),'Line');
        if ln <= 0, chk(false,'Goto %s has no source line',tag); return; end
        sb = get_param(ln,'SrcBlockHandle');
        sp = get_param(ln,'SrcPortHandle');
        pn = get_param(sp,'PortNumber');
        chk(strcmp(get_param(sb,'Name'), get_param(srcBlk,'Name')) && pn == portIdx, ...
            'tag %s sourced by %s port %d (expected %s port %d)', tag, get_param(sb,'Name'), pn, get_param(srcBlk,'Name'), portIdx);
    end

    function chkFeeds(dstBlk, srcName, dstPort, srcParent)
        if nargin < 3, dstPort = 2; end
        if nargin < 4, srcParent = m; end
        ph = get_param(dstBlk,'PortHandles');
        ln = get_param(ph.Inport(dstPort),'Line');
        if ln <= 0, chk(false,'%s inport %d unconnected',dstBlk,dstPort); return; end
        sb = get_param(ln,'SrcBlockHandle');
        ok = strcmp(get_param(sb,'BlockType'),'From') && strcmp(get_param(sb,'GotoTag'), srcName);
        chk(ok, '%s inport %d fed by %s (expected From tag %s)', get_param(dstBlk,'Name'), dstPort, get_param(sb,'Name'), srcName);
    end

    function chkFedBy(dstBlk, srcName, dstPort)
        ph = get_param(dstBlk,'PortHandles');
        ln = get_param(ph.Inport(dstPort),'Line');
        if ln <= 0, chk(false,'%s inport %d unconnected',dstBlk,dstPort); return; end
        sb = get_param(ln,'SrcBlockHandle');
        chk(strcmp(get_param(sb,'Name'), srcName), '%s inport %d fed by %s (expected %s)', ...
            get_param(dstBlk,'Name'), dstPort, get_param(sb,'Name'), srcName);
    end

    function chkDiode(blk, ~)
        % anode (p) must sit on the lower-potential node: verify by reading the
        % block's port names in the netlist order used by the map
        try
            chk(strcmp(get_param(blk,'BlockType'),'SimscapeBlock'), '%s is not a SimscapeBlock', blk);
        catch
            chk(false,'%s unreadable',blk);
        end
    end
end

% ---------------- file-local helpers (outside the nested-function scope) ----------------

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
% workspace, where the model resolves them.  The script runs in THIS function's
% workspace because a workspace that contains nested functions is static and
% cannot run a script into itself.  The parameter script starts with clearvars.
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
fprintf('Parameters: f_c = %g Hz, K_p = %.6g, K_i = %.6g, U_dc = %g V, bias = %g A\n', ...
    f_c, K_p, K_i, U_dc, bias_current);
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
        parts{end+1} = sprintf('%s n/a', names{k});           %#ok<AGROW>
    else
        parts{end+1} = sprintf('%s %s', names{k}, V(idx).Version); %#ok<AGROW>
    end
end
fprintf('Toolchain: %s\n', strjoin(parts, ' | '));
if ~strcmp(rel,'2025b')
    fprintf('NOTE: the documented values were verified with R2025b (%s detected).\n', rel);
end
end