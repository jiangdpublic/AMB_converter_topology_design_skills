function verify_model_facts()
% Asserts every block path, library path, parameter name and value, port name,
% Goto tag and netlist entry quoted in references/block-and-parameter-map.md
% against the bundled model in model/.  Prints a PASS/FAIL summary and raises
% an error when any check fails.
%
% Relocatable: the model and its parameter script are located relative to this
% file, so the whole skill folder can be copied anywhere.  Companion script:
% scripts/run_case_study.m.
%
% Verified with MATLAB/Simulink R2025b (25.2), Simscape and Simscape Electrical 25.2.

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
tmp = fullfile(tempdir,'amb_rsb_facts');
if ~exist(tmp,'dir'), mkdir(tmp); end
m = 'RSB_facts_check';
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

%% ---------------- 1. model configuration ----------------
fprintf('\n[1] model configuration\n');
cfg = { 'StopTime','0.1'; 'Solver','VariableStepAuto'; 'SolverType','Variable-step';
        'MaxStep','1e-6'; 'RelTol','1e-4'; 'StartTime','0.0';
        'SimulationMode','accelerator'; 'ZeroCrossAlgorithm','Nonadaptive';
        'MaxConsecutiveZCs','1000'; 'SignalLogging','on'; 'SignalLoggingName','logsout';
        'SimscapeLogType','none'; 'SaveFormat','Dataset' };
for i=1:size(cfg,1)
    v = get_param(m,cfg{i,1});
    chk(strcmp(v,cfg{i,2}), 'root %s = %s (found %s)', cfg{i,1}, cfg{i,2}, v);
end
sc = [m '/Solver Configuration'];
scp = { 'UseLocalSolver','off'; 'LocalSolverChoice','NE_TRAPEZOIDAL_ADVANCER'; 'DoFixedCost','off'; 'AutomaticFiltering','on' };
for i=1:size(scp,1)
    v = get_param(sc,scp{i,1});
    chk(strcmp(v,scp{i,2}), 'Solver Configuration.%s = %s (found %s)', scp{i,1}, scp{i,2}, v);
end

%% ---------------- 2. root blocks and library links ----------------
fprintf('[2] root blocks and library links\n');
rootBlocks = { 'Voltage Source','SimscapeBlock','ee_lib/Sources/Voltage Source';
               'Solver Configuration','SubSystem','nesl_utility/Solver Configuration';
               'Electrical Reference','SimscapeBlock','fl_lib/Electrical/Electrical Elements/Electrical Reference';
               'Open Circuit','SimscapeBlock','fl_lib/Electrical/Electrical Elements/Open Circuit';
               'Open Circuit1','SimscapeBlock','fl_lib/Electrical/Electrical Elements/Open Circuit';
               'Open Circuit2','SimscapeBlock','fl_lib/Electrical/Electrical Elements/Open Circuit';
               'IGBT (Ideal, Switching)3','SimscapeBlock','ee_lib/Semiconductors & Converters/IGBT (Ideal, Switching)';
               'IGBT (Ideal, Switching)4','SimscapeBlock','ee_lib/Semiconductors & Converters/IGBT (Ideal, Switching)';
               'Diode3','SimscapeBlock','fl_lib/Electrical/Electrical Elements/Diode';
               'Diode4','SimscapeBlock','fl_lib/Electrical/Electrical Elements/Diode';
               'Current Sensor','SimscapeBlock','fl_lib/Electrical/Electrical Sensors/Current Sensor';
               'Current Sensor1','SimscapeBlock','fl_lib/Electrical/Electrical Sensors/Current Sensor';
               'On-Off Delay','SubSystem','eeGeneralControl/On-Off Delay';
               'On-Off Delay1','SubSystem','eeGeneralControl/On-Off Delay';
               'Mux','Mux',''; 'Demux','Demux',''; 'Product','Product','';
               'Add1','Sum',''; 'Add2','Sum',''; 'Add3','Sum',''; 'Add4','Sum','';
               'Gain1','Gain',''; 'winding_currents','Scope',''; 'shared_bridge_current','Scope','' };
for i=1:size(rootBlocks,1)
    b = [m '/' rootBlocks{i,1}];
    ok = ~isempty(find_system(m,'SearchDepth',1,'Name',rootBlocks{i,1}));
    chk(ok, 'root block %s exists', rootBlocks{i,1});
    if ~ok, continue; end
    chk(strcmp(get_param(b,'BlockType'),rootBlocks{i,2}), '%s BlockType = %s', rootBlocks{i,1}, rootBlocks{i,2});
    if ~isempty(rootBlocks{i,3})
        chk(strcmp(flat(get_param(b,'ReferenceBlock')),rootBlocks{i,3}), '%s ReferenceBlock = %s (found %s)', rootBlocks{i,1}, rootBlocks{i,3}, flat(get_param(b,'ReferenceBlock')));
    end
end
for k = 1:5
    nm = 'Bridge_normal_module'; if k>1, nm = sprintf('Bridge_normal_module%d',k-1); end
    chk(~isempty(find_system(m,'SearchDepth',1,'Name',nm)), 'T module %s exists', nm);
    nm = 'Bridge_reversed_module'; if k>1, nm = sprintf('Bridge_reversed_module%d',k-1); end
    chk(~isempty(find_system(m,'SearchDepth',1,'Name',nm)), 'B module %s exists', nm);
    nm = 'AMB_ref_gen'; if k>1, nm = sprintf('AMB_ref_gen%d',k-1); end
    chk(~isempty(find_system(m,'SearchDepth',1,'Name',nm)), 'reference generator %s exists', nm);
end
for k = 1:10
    chk(~isempty(find_system(m,'SearchDepth',1,'Name',sprintf('PI_ctrl%d',k))), 'PI_ctrl%d exists', k);
end
for k = 0:10
    nm = 'PWM_gen'; if k>0, nm = sprintf('PWM_gen%d',k); end
    chk(~isempty(find_system(m,'SearchDepth',1,'Name',nm)), 'modulator %s exists', nm);
end
chk(strcmp(get_param([m '/Mux'],'Inputs'),'10') && strcmp(get_param([m '/Demux'],'Outputs'),'10'), 'Mux/Demux are 10-wide');
chk(strcmp(get_param([m '/winding_currents'],'NumInputPorts'),'10'), 'winding_currents has 10 inputs');
chk(strcmp(get_param([m '/shared_bridge_current'],'NumInputPorts'),'2'), 'shared_bridge_current has 2 inputs');

%% ---------------- 3. device parameters ----------------
fprintf('[3] device parameters\n');
mods = {'Bridge_normal_module','Bridge_reversed_module'};
for i=1:numel(mods)
    ig = find_system([m '/' mods{i}],'SearchDepth',1,'ReferenceBlock','ee_lib/Semiconductors & Converters/IGBT (Ideal, Switching)');
    chk(numel(ig)==1, '%s has exactly one IGBT block', mods{i});
    chk(strcmp(get_param(ig{1},'Vf'),'0.8') && strcmp(get_param(ig{1},'Ron'),'0.001'), '%s IGBT Vf/Ron', mods{i});
    chk(strcmp(get_param(ig{1},'Vth'),'6'), '%s IGBT Vth = 6', mods{i});
    chk(strcmp(get_param(ig{1},'diode_param'),'ee.enum.semiconductors.protectionDiode.external'), '%s IGBT protection diode is external', mods{i});
    d = [m '/' mods{i} '/Diode5'];
    chk(strcmp(get_param(d,'Vf'),'0.6') && strcmp(get_param(d,'Ron'),'0.3') && strcmp(get_param(d,'Goff'),'1e-8'), '%s freewheel diode 0.6/0.3/1e-8', mods{i});
    chk(strcmp(get_param([m '/' mods{i} '/Resistor2'],'R'),'R_value'), '%s resistor R = R_value', mods{i});
    chk(strcmp(get_param([m '/' mods{i} '/Inductor2'],'l'),'L_value'), '%s inductor l = L_value', mods{i});
    chk(strcmp(get_param([m '/' mods{i} '/Inductor2'],'r'),'0') && strcmp(get_param([m '/' mods{i} '/Inductor2'],'g'),'1e-9'), '%s inductor r/g', mods{i});
end
for b = {'IGBT (Ideal, Switching)3','IGBT (Ideal, Switching)4'}
    chk(strcmp(get_param([m '/' b{1}],'Vf'),'0.8') && strcmp(get_param([m '/' b{1}],'Ron'),'0.001') && strcmp(get_param([m '/' b{1}],'Vth'),'6'), '%s parameters', b{1});
end
for b = {'Diode3','Diode4'}
    chk(strcmp(get_param([m '/' b{1}],'Vf'),'0.6') && strcmp(get_param([m '/' b{1}],'Ron'),'0.3'), '%s parameters', b{1});
end
chk(strcmp(get_param([m '/Voltage Source'],'dc_voltage'),'U_dc'), 'Voltage Source dc_voltage = U_dc');

%% ---------------- 4. dead time ----------------
fprintf('[4] shared-leg dead time\n');
for b = {'On-Off Delay','On-Off Delay1'}
    mv = get_param([m '/' b{1}],'MaskValues'); if ~ischar(mv), mv = strjoin(cellstr(string(mv)),' '); end
    mv = strtrim(regexprep(mv,'\s+',' '));
    chk(strcmp(mv,'1e-6 0 0 0'), '%s mask values are [1e-6 0 0 0] (found "%s")', b{1}, mv);
end

%% ---------------- 5. module interface and netlist ----------------
fprintf('[5] module interface and netlist\n');
ports = {'sw','i_fbk','Vcci','Shared_i','Vddi','Vcco','Shared_o','Vddo'};
for i=1:numel(mods)
    bl = find_system([m '/' mods{i}],'SearchDepth',1,'Type','Block');
    names = cellfun(@(x) regexprep(get_param(x,'Name'),'\s+',' '), bl, 'uni', 0);
    for p = 1:numel(ports)
        chk(any(strcmp(names,ports{p})), '%s has port/block %s', mods{i}, ports{p});
    end
    B = [m '/' mods{i}];
    if strcmp(mods{i},'Bridge_normal_module')
        chk(link(B,'IGBT (Ideal, Switching)5','C', 'Vcci',''), 'T module: IGBT collector linked to the Vcc port');
        chk(link(B,'IGBT (Ideal, Switching)5','C', 'Vcco',''), 'T module: IGBT collector linked to the Vcco port');
        chk(link(B,'Diode5','p', 'Vddo',''), 'T module: diode anode linked to the Vdd port');
        chk(link(B,'Diode5','n', 'IGBT (Ideal, Switching)5','E'), 'T module: diode cathode linked to the IGBT emitter (node X)');
        chk(link(B,'Current Sensor2','p', 'IGBT (Ideal, Switching)5','E') || link(B,'Current Sensor2','p', 'Diode5','n'), 'T module: sensor p on node X');
        chk(link(B,'Current Sensor2','n', 'Inductor2','n'), 'T module: sensor n linked to the inductor');
        chk(link(B,'Inductor2','p', 'Resistor2','n'), 'T module: inductor linked to the resistor');
        chk(link(B,'Resistor2','p', 'Shared_i','') || link(B,'Resistor2','p', 'Shared_o',''), 'T module: resistor linked to the Shared port');
        chk(link(B,'Shared_i','', 'Shared_o','') || link(B,'Shared_o','', 'Shared_i',''), 'T module: Shared_i and Shared_o are the same node');
    else
        chk(link(B,'Diode5','n', 'Vcci',''), 'B module: diode cathode linked to the Vcc port');
        chk(link(B,'IGBT (Ideal, Switching)5','E', 'Vddo',''), 'B module: IGBT emitter linked to the Vdd port');
        chk(link(B,'IGBT (Ideal, Switching)5','C', 'Diode5','p'), 'B module: IGBT collector linked to the diode anode (node Y)');
        chk(link(B,'Current Sensor2','n', 'Diode5','p') || link(B,'Current Sensor2','n', 'IGBT (Ideal, Switching)5','C'), 'B module: sensor n on node Y');
        chk(link(B,'Current Sensor2','p', 'Inductor2','n'), 'B module: sensor p linked to the inductor');
        chk(link(B,'Inductor2','p', 'Resistor2','n'), 'B module: inductor linked to the resistor');
        chk(link(B,'Resistor2','p', 'Shared_i','') || link(B,'Resistor2','p', 'Shared_o',''), 'B module: resistor linked to the Shared port');
        chk(link(B,'Shared_i','', 'Shared_o','') || link(B,'Shared_o','', 'Shared_i',''), 'B module: Shared_i and Shared_o are the same node');
    end
end

%% ---------------- 6. shared-leg wiring ----------------
fprintf('[6] shared-leg wiring\n');
chk(link(m,'IGBT (Ideal, Switching)3','E', 'Current Sensor','p'), 'shared leg: IGBT3 emitter linked to the upper sensor node');
chk(link(m,'Current Sensor','n', 'Bridge_normal_module','Shared_i'), 'shared leg: upper sensor n on the module Shared port (the shared node)');
chk(link(m,'Current Sensor1','p', 'Bridge_normal_module','Shared_i'), 'shared leg: lower sensor p on the shared node');
chk(link(m,'IGBT (Ideal, Switching)4','C', 'Current Sensor1','n'), 'shared leg: IGBT4 collector linked to the lower sensor node');
chk(link(m,'IGBT (Ideal, Switching)3','C', 'Voltage Source','p'), 'shared leg: IGBT3 collector on bus P');
chk(link(m,'IGBT (Ideal, Switching)4','E', 'Voltage Source','n'), 'shared leg: IGBT4 emitter on bus N');
chk(link(m,'Diode3','p', 'Current Sensor','p'), 'shared leg: Diode3 anode on the shared node');
chk(link(m,'Diode4','n', 'Current Sensor1','n'), 'shared leg: Diode4 cathode on the shared node');
chk(link(m,'Diode3','n', 'Voltage Source','p'), 'shared leg: Diode3 cathode on bus P');
chk(link(m,'Diode4','p', 'Voltage Source','n'), 'shared leg: Diode4 anode on bus N');

%% ---------------- 7. controller chain ----------------
fprintf('[7] controller chain\n');
b = [m '/PI_ctrl1'];
chk(strcmp(get_param([b '/Add2'],'Inputs'),'+-'), 'PI Add2 is +- (err = ref - fbk)');
chk(strcmp(get_param([b '/Add3'],'Inputs'),'++'), 'PI Add3 is ++');
chk(strcmp(get_param([b '/Gain4'],'Gain'),'K_p'), 'PI Gain4 = K_p');
chk(strcmp(get_param([b '/Saturation'],'UpperLimit'),'1') && strcmp(get_param([b '/Saturation'],'LowerLimit'),'-1'), 'PI saturation +/-1');
rt1 = [b '/Rate Transition1'];
chk(strcmp(get_param(rt1,'OutPortSampleTimeOpt'),'Specify') && strcmp(get_param(rt1,'OutPortSampleTime'),'SW_Ts/4'), 'oversampling Rate Transition uses SW_Ts/4');
chk(strcmp(get_param(rt1,'Integrity'),'on') && strcmp(get_param(rt1,'Deterministic'),'on'), 'oversampling Rate Transition integrity/determinism');
fir = [b '/Discrete FIR Filter'];
chk(strcmp(get_param(fir,'Coefficients'),'[0.25 0.25 0.25 0.25]'), 'FIR coefficients are four 0.25 taps');
chk(strcmp(get_param([b '/Memory'],'InitialCondition'),'0'), 'feedback Memory IC = 0');
noFilter = isempty(find_system(b,'LookUnderMasks','all','FollowLinks','on','Regexp','on','Name','Second-Order'));
chk(noFilter, 'no second-order filter in the controller feedback path');
aw = [b '/Ictrl_anti_windup'];
chk(strcmp(get_param([aw '/Gain1'],'Gain'),'K_i'), 'anti-windup Gain1 = K_i');
chk(strcmp(get_param([aw '/Integrator'],'LimitOutput'),'on') && strcmp(get_param([aw '/Integrator'],'UpperSaturationLimit'),'1'), 'integrator limited to +/-1');
chk(strcmp(get_param([aw '/AND'],'Operator'),'OR'), 'the block named AND is configured as OR');
chk(strcmp(get_param([aw '/AND1'],'Operator'),'AND'), 'AND1 is configured as AND');
chk(strcmp(get_param([aw '/Compare To Constant'],'const'),'-1') && strcmp(get_param([aw '/Compare To Constant1'],'const'),'1') && strcmp(get_param([aw '/Compare To Constant2'],'const'),'0'), 'anti-windup comparison constants');
chk(strcmp(get_param([aw '/Switch'],'Criteria'),'u2 ~= 0'), 'anti-windup switch criteria');

%% ---------------- 8. carrier, reference generator, modulation chain ----------------
fprintf('[8] carrier, reference generator, modulation chain\n');
for k = 0:10
    nm = 'PWM_gen'; if k>0, nm = sprintf('PWM_gen%d',k); end
    rs = [m '/' nm '/Repeating Sequence'];
    chk(strcmp(get_param(rs,'rep_seq_t'),'[0 half_SW_Ts SW_Ts]') && strcmp(get_param(rs,'rep_seq_y'),'[0 1 0]'), '%s carrier definition', nm);
    ro = [m '/' nm '/Relational Operator2'];
    chk(strcmp(get_param(ro,'Operator'),'>='), '%s comparator is >=', nm);
end
chk(strcmp(get_param([m '/AMB_ref_gen/Constant'],'Value'),'bias_current'), 'reference generator bias constant');
chk(strcmp(get_param([m '/AMB_ref_gen/Gain'],'Gain'),'0.5'), 'reference generator gain 0.5');
chk(strcmp(get_param([m '/AMB_ref_gen/Add'],'Inputs'),'++') && strcmp(get_param([m '/AMB_ref_gen/Add1'],'Inputs'),'-+'), 'reference generator sum signs');
chk(strcmp(get_param([m '/Add1'],'Inputs'),'++') && strcmp(get_param([m '/Add2'],'Inputs'),'+-') && ...
    strcmp(get_param([m '/Add3'],'Inputs'),'+-') && strcmp(get_param([m '/Add4'],'Inputs'),'-+'), 'modulation chain sum signs');
chk(strcmp(get_param([m '/Gain1'],'Gain'),'0.5'), 'modulation Gain1 = 0.5');
chk(strcmp(get_param([m '/Constant5'],'Value'),'0.5') && strcmp(get_param([m '/Constant8'],'Value'),'0'), 'modulation constants 0.5 / 0');
chk(strcmp(get_param([m '/Constant7'],'Value'),'[1 1 1 1 1 1 1 1 1 1]'), 'broadcast constant is ten ones');
rt = sfroot; ch = rt.find('-isa','Stateflow.EMChart','Path',[m '/MATLAB Function']);
chk(~isempty(ch), 'MATLAB Function block exists');
if ~isempty(ch)
    s = ch(1).Script;
    chk(contains(s,'sort(') && contains(s,'c_sorted(1)') && contains(s,'c_sorted(end)'), 'min/max is computed by sorting (first/last element)');
end

%% ---------------- 9. gate and current routing ----------------
fprintf('[9] gate and current routing\n');
gateMap = { 'PWM_gen','','Gain','sw1','Bridge_normal_module';
            'PWM_gen1','','Gain3','sw2','Bridge_normal_module1';
            'PWM_gen2','','Gain4','sw3','Bridge_normal_module2';
            'PWM_gen3','','Gain5','sw4','Bridge_normal_module3';
            'PWM_gen4','','Gain6','sw5','Bridge_normal_module4';
            'PWM_gen5','NOT4','Gain7','sw6','Bridge_reversed_module';
            'PWM_gen6','NOT','Gain8','sw7','Bridge_reversed_module1';
            'PWM_gen7','NOT1','Gain9','sw8','Bridge_reversed_module2';
            'PWM_gen8','NOT2','Gain10','sw9','Bridge_reversed_module3';
            'PWM_gen9','NOT3','Gain11','sw10','Bridge_reversed_module4' };
for i=1:size(gateMap,1)
    src = gateMap{i,1};
    if ~isempty(gateMap{i,2})
        chk(any(strcmp(bare(srcNames([m '/' gateMap{i,2}]),m), src)), '%s is fed by %s', gateMap{i,2}, src);
        src = gateMap{i,2};
    end
    amp = gateMap{i,3};
    chk(any(strcmp(bare(srcNames([m '/' amp]),m), src)), '%s is fed by %s', amp, src);
    chk(strcmp(get_param([m '/' amp],'Gain'),'15'), '%s gain is 15', amp);
    chk(strcmp(tagOf(dstNames([m '/' amp])), gateMap{i,4}), '%s drives tag %s', amp, gateMap{i,4});
    chk(strcmp(tagOf(srcNames([m '/' gateMap{i,5}])), gateMap{i,4}), '%s gate is driven by tag %s', gateMap{i,5}, gateMap{i,4});
end
chk(strcmp(tagOf(srcNames([m '/On-Off Delay'])),'sw0'), 'upper shared-leg delay is driven by sw0');
chk(strcmp(tagOf(srcNames([m '/NOT5'])),'sw0'), 'NOT5 is driven by sw0');
chk(any(strcmp(bare(dstNames([m '/On-Off Delay']),m),'Gain2')), 'upper shared-leg delay feeds Gain2');
chk(any(strcmp(bare(dstNames([m '/On-Off Delay1']),m),'Gain12')), 'lower shared-leg delay feeds Gain12');
chk(any(strcmp(bare(dstNames([m '/Gain2']),m),'Simulink-PS Converter')), 'Gain2 feeds the upper IGBT gate converter');
chk(any(strcmp(bare(dstNames([m '/Gain12']),m),'Simulink-PS Converter1')), 'Gain12 feeds the lower IGBT gate converter');
curMap = { 'Bridge_normal_module','Goto','i_a1','PI_ctrl1';
           'Bridge_normal_module1','Goto12','i_c1','PI_ctrl2';
           'Bridge_normal_module2','Goto13','i_a2','PI_ctrl3';
           'Bridge_normal_module3','Goto14','i_c2','PI_ctrl4';
           'Bridge_normal_module4','Goto15','i_a3','PI_ctrl5';
           'Bridge_reversed_module','Goto16','i_c3','PI_ctrl6';
           'Bridge_reversed_module1','Goto17','i_a4','PI_ctrl7';
           'Bridge_reversed_module2','Goto18','i_c4','PI_ctrl8';
           'Bridge_reversed_module3','Goto19','i_a5','PI_ctrl9';
           'Bridge_reversed_module4','Goto20','i_c5','PI_ctrl10' };
for i=1:size(curMap,1)
    G = [m '/' curMap{i,2}];
    chk(strcmp(get_param(G,'GotoTag'),curMap{i,3}), '%s carries tag %s', curMap{i,2}, curMap{i,3});
    chk(any(strcmp(bare(dstNames([m '/' curMap{i,1}]),m), curMap{i,2})), '%s i_fbk feeds %s', curMap{i,1}, curMap{i,2});
    fbk = srcNames([m '/' curMap{i,4}]);
    chk(~isempty(fbk) && strcmp(tagOf(fbk),curMap{i,3}), '%s.fbk is fed by tag %s', curMap{i,4}, curMap{i,3});
end
chk(strcmp(get_param([m '/Goto21'],'GotoTag'),'i_shared_h') && strcmp(get_param([m '/Goto22'],'GotoTag'),'i_shared_l'), 'shared-leg current tags exist');
chk(any(strcmp(bare(physDst([m '/Current Sensor'],'I'),m),'PS-Simulink Converter')), 'upper shared-leg sensor feeds its converter');
chk(any(strcmp(bare(physDst([m '/Current Sensor1'],'I'),m),'PS-Simulink Converter1')), 'lower shared-leg sensor feeds its converter');
for k=13:17
    chk(strcmp(get_param([m sprintf('/Gain%d',k)],'Gain'),'-1'), 'Gain%d negates a B-module command', k);
end
chk(any(strcmp(bare(dstNames([m '/Gain13']),m),'Mux')), 'negated commands feed the Mux');

%% ---------------- 10. workspace variables ----------------
fprintf('[10] workspace variables\n');
vars = {'R_value','L_value','U_dc','f_c','K_p','K_i','SW_Ts','half_SW_Ts','bias_current','SW_frequency'};
for i=1:numel(vars)
    chk(evalin('base',sprintf('exist(''%s'',''var'')',vars{i}))==1, 'workspace variable %s exists', vars{i});
end
chk(abs(evalin('base','K_p') - 2*pi*evalin('base','f_c')*evalin('base','L_value')/evalin('base','U_dc')) < 1e-12, 'K_p matches 2*pi*f_c*L/U_dc');
chk(abs(evalin('base','K_i') - 2*pi*evalin('base','f_c')*evalin('base','R_value')/evalin('base','U_dc')) < 1e-12, 'K_i matches 2*pi*f_c*R/U_dc');
chk(abs(evalin('base','SW_Ts') - 1/evalin('base','SW_frequency')) < 1e-15, 'SW_Ts = 1/SW_frequency');
chk(abs(evalin('base','half_SW_Ts') - 0.5*evalin('base','SW_Ts')) < 1e-15, 'half_SW_Ts = SW_Ts/2');

%% ---------------- summary ----------------
close_system(m,0);
fprintf('\nverify_model_facts: %d PASS, %d FAIL\n', npass, nfail);
if nfail > 0
    error('verify_model_facts: %d checks failed against references/block-and-parameter-map.md', nfail);
end
fprintf('ALL FACTS VERIFIED\n');
end

%% ---------------- local helpers ----------------
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
run(scriptPath);   % the script starts with clearvars and defines the eight variables
end

function ambToolchain()
v = ver('simscape');
assert(~isempty(v), 'Simscape is required to run the RSB verification scripts.');
end

function s = flat(s)
if ~ischar(s), s = ''; return; end
s = strtrim(regexprep(s,'\s+',' '));
end

function t = tagOf(names)
t = '';
if ischar(names), names = {names}; end
for i=1:numel(names)
    try
        t = get_param(names{i},'GotoTag');
        if ~isempty(t), return; end
    catch
    end
end
end

function s = bare(names, m)
% Strip the model prefix so that helper output can be compared with bare block names.
if ischar(names), names = {names}; end
s = names;
for i = 1:numel(names)
    s{i} = regexprep(regexprep(strrep(names{i},[m '/'],''),'\s+',' '),'^.*/','');
end
end

function names = srcNames(blk)
% Blocks driving the input ports of a block or subsystem (Simulink signals and
% Simscape physical connections alike, as reported by PortConnectivity).
names = {};
pc = get_param(blk,'PortConnectivity');
for i = 1:numel(pc)
    v = pc(i).SrcBlock; if iscell(v), v = [v{:}]; end
    for k = 1:numel(v)
        if v(k) > 0, names{end+1} = regexprep(getfullname(v(k)),'\s+',' '); end
    end
end
names = unique(names);
end

function names = dstNames(blk)
% Blocks driven by the output ports of a block or subsystem.
names = {};
pc = get_param(blk,'PortConnectivity');
for i = 1:numel(pc)
    v = pc(i).DstBlock; if iscell(v), v = [v{:}]; end
    for k = 1:numel(v)
        if v(k) > 0, names{end+1} = regexprep(getfullname(v(k)),'\s+',' '); end
    end
end
names = unique(names);
end

function names = physDst(blkPath, portName)
% Block names on the other end of the physical line leaving one Simscape port.
names = {};
ph = get_param(blkPath,'PortHandles');
pc = get_param(blkPath,'PortConnectivity');
idx = portIndex(blkPath, portName, ph.LConn, ph.RConn, pc);
for i = idx
    v = pc(i).DstBlock;
    if iscell(v), v = [v{:}]; end
    for k = 1:numel(v)
        if v(k) > 0
            names{end+1} = regexprep(getfullname(v(k)),'\s+',' ');
        end
    end
end
names = unique(names);
end

function tf = link(sys, nameA, portA, nameB, portB)
% True when the two Simscape ports are joined by one physical line segment of
% the same network (checked in both directions).
tf = oneWay(sys,nameA,portA,nameB,portB) || oneWay(sys,nameB,portB,nameA,portA);
end

function tf = oneWay(sys, nameA, portA, nameB, portB)
nb = neighbourNames(fullfile(sys,nameA), portA);
tf = any(strcmp(nb, nameB));
end

function names = neighbourNames(blkPath, portName)
% Block names that share a physical line segment with the given port.
names = {};
ph = get_param(blkPath,'PortHandles');
pc = get_param(blkPath,'PortConnectivity');
idx = portIndex(blkPath, portName, ph.LConn, ph.RConn, pc);
for i = idx
    for f = {'SrcBlock','DstBlock'}
        v = pc(i).(f{1});
        if iscell(v), v = [v{:}]; end
        for k = 1:numel(v)
            if v(k) > 0
                names{end+1} = regexprep(get_param(v(k),'Name'),'\s+',' ');
            end
        end
    end
end
names = unique(names);
end

function idx = portIndex(blkPath, portName, lconn, rconn, pc)
% Map a datasheet port label onto the PortConnectivity entry index.
% Layouts verified against the bundled model (R2025b library blocks):
%   IGBT: LConn1 = G, RConn1 = C, RConn2 = E
%   Diode / Resistor / Inductor / Voltage Source: LConn1 = p, RConn1 = n
%   Current Sensor: LConn1 = p, RConn1 = I, RConn2 = n
%   connection-port block (PMIOPort): a single port
rb = '';
try, rb = get_param(blkPath,'ReferenceBlock'); catch, end
if isempty(portName)
    idx = 1:numel(pc);
elseif contains(rb,'IGBT')
    switch portName
        case 'G', idx = 1; case 'C', idx = 2; case 'E', idx = 3;
        otherwise, error('portIndex: bad IGBT port %s', portName);
    end
elseif contains(rb,'Diode') || contains(rb,'Resistor') || contains(rb,'Inductor') || contains(rb,'Voltage Source')
    if strcmp(portName,'p'), idx = 1; else, idx = 2; end
elseif contains(rb,'Current Sensor')
    switch portName
        case 'p', idx = 1; case 'I', idx = 2; case 'n', idx = 3;
        otherwise, error('portIndex: bad sensor port %s', portName);
    end
else
    error('portIndex: unsupported block %s (%s)', blkPath, rb);
end
end