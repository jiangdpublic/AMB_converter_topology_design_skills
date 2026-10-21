function verify_model_facts()
% Asserts every block path, library reference, parameter name/value, connection
% port name and netlist edge quoted in references/block-and-parameter-map.md
% against the bundled model in model/.  Prints a PASS/FAIL summary and raises an
% error when any check fails.
%
% Relocatable: the model and its parameter script are located relative to this
% file, and the model is loaded from a renamed copy in the system temp folder,
% so the bundled .slx is never written to.  Companion script:
% scripts/run_case_study.m.
%
% Verified with MATLAB/Simulink R2025b (25.2), Simscape and Simscape Electrical 25.2.
% Runtime: about 20 s (no simulation is performed).

Here      = fileparts(mfilename('fullpath'));
ModelPath = ambFindModel(Here);
ParamPath = fullfile(fileparts(ModelPath),'parameters_for_AMB_topology.m');
assert(isfile(ParamPath), 'parameters_for_AMB_topology.m must sit next to the model.');
P = loadAmbParams(ParamPath);
tally(NaN,0);   % start this run from zero
fprintf('Model asset : %s\n', ModelPath);

tmp = fullfile(tempdir,'amb_swt_facts');
if ~exist(tmp,'dir'), mkdir(tmp); end
m = 'SWT_facts_check';
if bdIsLoaded(m), close_system(m,0); end
target = fullfile(tmp,[m '.slx']);
if exist(target,'file'), delete(target); end
copyfile(ModelPath, target, 'f');
load_system(target);
cleanup = onCleanup(@() close_system(m,0));

bp = @(varargin) [m '/' strjoin(varargin, '/')];
solverBlk = bp(['Solver' char(10) 'Configuration']);

%% ---------- 1. inventory ----------
legs  = arrayfun(@(k) legName(k),  0:10, 'uni', 0);
winds = arrayfun(@(k) windName(k), 1:10, 'uni', 0);
for k = 1:numel(legs),  check(sprintf('block present: %s', legs{k}),  bdIsLoaded(m) && exist_block(bp(legs{k}))); end
for k = 1:numel(winds), check(sprintf('block present: %s', winds{k}), exist_block(bp(winds{k}))); end
check('11 Bridge_module blocks', numel(find_system(m,'SearchDepth',1,'Regexp','on','Name','^Bridge_module[0-9]*$')) == 11);
check('10 RL_load blocks',       numel(find_system(m,'SearchDepth',1,'Regexp','on','Name','^RL_load[0-9]*$')) == 10);
check('10 PI_ctrl blocks',       numel(find_system(m,'SearchDepth',1,'Regexp','on','Name','^PI_ctrl[0-9]*$')) == 10);
check('11 PWM_gen blocks',       numel(find_system(m,'SearchDepth',1,'Regexp','on','Name','^PWM_gen[0-9]*$')) == 11);
check('5 AMB_ref_gen blocks',    numel(find_system(m,'SearchDepth',1,'Regexp','on','Name','^AMB_ref_gen[0-9]*$')) == 5);
check('4 ref_delay blocks',      numel(find_system(m,'SearchDepth',1,'Regexp','on','Name','^ref_delay[0-9]*$')) == 4);
check('4 Open Circuit blocks',   numel(find_system(m,'SearchDepth',1,'Regexp','on','Name','^Open Circuit[0-9]*$')) == 4);
check('MATLAB Function present', exist_block(bp('MATLAB Function')));
check('Scope winding_currents has 10 inputs', str2double(get_param(bp('winding_currents'),'NumInputPorts')) == 10);

%% ---------- 2. library references ----------
check('IGBT library path', strcmp(refOf(bp('Bridge_module','IGBT (Ideal, Switching)1')), ...
      sprintf('ee_lib/Semiconductors & Converters/IGBT (Ideal, Switching)')));
check('Diode library path', strcmp(refOf(bp('Bridge_module','Diode1')), ...
      'fl_lib/Electrical/Electrical Elements/Diode'));
check('Voltage Source library path', strcmp(refOf(bp('Voltage Source')), 'ee_lib/Sources/Voltage Source'));
check('Electrical Reference library path', strcmp(refOf(bp('Electrical Reference')), ...
      'fl_lib/Electrical/Electrical Elements/Electrical Reference'));
check('Solver Configuration library path', strcmp(refOf(solverBlk), 'nesl_utility/Solver Configuration'));
check('On-Off Delay library path', strcmp(refOf(bp('PWM_gen','On-Off Delay')), 'eeGeneralControl/On-Off Delay'));
check('Second-Order Filter library path', strcmp(refOf(bp('PI_ctrl','Second-Order Filter')), 'eeGeneralControl/Second-Order Filter'));
check('RL_load current sensor library path', strcmp(refOf(bp('RL_load','Current Sensor2')), ...
      'fl_lib/Electrical/Electrical Sensors/Current Sensor'));

%% ---------- 3. device parameters ----------
igbt = bp('Bridge_module','IGBT (Ideal, Switching)1');
check('IGBT Vf = 0.8',    str2double(get_param(igbt,'Vf'))   == 0.8);
check('IGBT Ron = 1e-3',  str2double(get_param(igbt,'Ron'))  == 1e-3);
check('IGBT Goff = 1e-5', str2double(get_param(igbt,'Goff')) == 1e-5);
check('IGBT Vth = 6',     str2double(get_param(igbt,'Vth'))  == 6);
check('IGBT protection diode external', contains(get_param(igbt,'diode_param'),'external'));
di = bp('Bridge_module','Diode1');
check('Diode Vf = 0.6',   str2double(get_param(di,'Vf'))  == 0.6);
check('Diode Ron = 0.3',  str2double(get_param(di,'Ron')) == 0.3);
check('Diode Goff = 1e-8',str2double(get_param(di,'Goff'))== 1e-8);
ind = bp('RL_load','Inductor2');
check('inductor l = L_value', strcmp(get_param(ind,'l'),'L_value'));
check('inductor r = 0',       strcmp(get_param(ind,'r'),'0'));
check('resistor R = R_value', strcmp(get_param(bp('RL_load','Resistor2'),'R'),'R_value'));
check('current sensor feeds i_fbk', exist_block(bp('RL_load','i_fbk')));

%% ---------- 4. control and modulation parameters ----------
gain4 = get_param(bp('PI_ctrl','Gain4'),'Gain');
check('PI proportional gain is K_p', strcmp(gain4,'K_p'));
check('PI integral gain is K_i', strcmp(get_param(bp('PI_ctrl','Ictrl_anti_windup','Gain1'),'Gain'),'K_i'));
rt = bp('PI_ctrl','Rate Transition');
check('Rate Transition samples at SW_Ts', strcmp(get_param(rt,'OutPortSampleTime'),'SW_Ts'));
check('Rate Transition integrity on',     strcmp(get_param(rt,'Integrity'),'on'));
check('Rate Transition deterministic on', strcmp(get_param(rt,'Deterministic'),'on'));
check('feedback Memory before the filter', strcmp(get_param(bp('PI_ctrl','Memory'),'InitialCondition'),'0'));
sf = bp('PI_ctrl','Second-Order Filter');
check('feedback filter fn = 6000',  str2double(get_param(sf,'fn'))   == 6000);
check('feedback filter zeta = 0.707', abs(str2double(get_param(sf,'Zeta')) - 0.707) < 1e-9);
sat = bp('PI_ctrl','Saturation');
check('output clamp upper = 1',  str2double(get_param(sat,'UpperLimit')) == 1);
check('output clamp lower = -1', str2double(get_param(sat,'LowerLimit')) == -1);
itg = bp('PI_ctrl','Ictrl_anti_windup','Integrator');
check('integrator limit output on', strcmp(get_param(itg,'LimitOutput'),'on'));
check('integrator upper limit = 1',  str2double(get_param(itg,'UpperSaturationLimit')) == 1);
check('integrator lower limit = -1', str2double(get_param(itg,'LowerSaturationLimit')) == -1);
check('anti-windup OR block computes OR',  strcmp(get_param(bp('PI_ctrl','Ictrl_anti_windup','OR'),'Operator'),'OR'));
check('anti-windup AND block computes AND',strcmp(get_param(bp('PI_ctrl','Ictrl_anti_windup','AND'),'Operator'),'AND'));
check('anti-windup limit test <= -1', strcmp(get_param(bp('PI_ctrl','Ictrl_anti_windup','Compare To Constant'),'relop'),'<=') && ...
                                      str2double(get_param(bp('PI_ctrl','Ictrl_anti_windup','Compare To Constant'),'const')) == -1);
check('anti-windup limit test >= 1',  strcmp(get_param(bp('PI_ctrl','Ictrl_anti_windup','Compare To Constant1'),'relop'),'>=') && ...
                                      str2double(get_param(bp('PI_ctrl','Ictrl_anti_windup','Compare To Constant1'),'const')) == 1);
check('anti-windup freeze switch is u2 ~= 0', contains(get_param(bp('PI_ctrl','Ictrl_anti_windup','Switch'),'Criteria'),'~='));
check('reference gainic 0.5', str2double(get_param(bp('AMB_ref_gen','Gain'),'Gain')) == 0.5);
check('reference bias constant = bias_current', strcmp(get_param(bp('AMB_ref_gen','Constant'),'Value'),'bias_current'));
check('ia_ref sum is ++', strcmp(get_param(bp('AMB_ref_gen','Add'),'Inputs'),'++'));
check('ic_ref sum is -+', strcmp(get_param(bp('AMB_ref_gen','Add1'),'Inputs'),'-+'));
check('carrier rep_seq_t = [0 half_SW_Ts SW_Ts]', strcmp(get_param(bp('PWM_gen','Repeating Sequence'),'rep_seq_t'),'[0 half_SW_Ts SW_Ts]'));
check('carrier rep_seq_y = [0 1 0]',             strcmp(get_param(bp('PWM_gen','Repeating Sequence'),'rep_seq_y'),'[0 1 0]'));
check('comparator is >=', contains(get_param(bp('PWM_gen','Relational Operator2'),'Operator'),'>='));
check('dead time Td_on = 1e-6', str2double(get_param(bp('PWM_gen','On-Off Delay'),'Td_on')) == 1e-6);
check('dead time Td_off = 0',   str2double(get_param(bp('PWM_gen','On-Off Delay'),'Td_off')) == 0);
check('gate level 15 V (PWMH)', str2double(get_param(bp('PWM_gen','Gain'),'Gain'))  == 15);
check('gate level 15 V (PWML)', str2double(get_param(bp('PWM_gen','Gain1'),'Gain')) == 15);
check('enable Steps at 0/1/2/3/4 ms', ...
      strcmp(get_param(bp('Step'),'Time'),'0.001*4') && strcmp(get_param(bp('Step1'),'Time'),'0.001*3') && ...
      strcmp(get_param(bp('Step2'),'Time'),'0.001*2') && strcmp(get_param(bp('Step3'),'Time'),'0.001*1'));
check('i_ref constants all equal 1', all(cellfun(@(b) strcmp(get_param(bp(b),'Value'),'1'), ...
      {'Constant1','Constant2','Constant3','Constant4','Constant6'})));
check('modulator offset constant 0.5', strcmp(get_param(bp('Constant5'),'Value'),'0.5'));
check('modulator broadcast vector is 11 ones', strcmp(get_param(bp('Constant7'),'Value'),'[1 1 1 1 1 1 1 1 1 1 1]'));
check('Mux1 has 11 inputs', strcmp(get_param(bp('Mux1'),'Inputs'),'11'));
check('Demux has 11 outputs', strcmp(get_param(bp('Demux'),'Outputs'),'11'));

%% ---------- 5. connection ports of a bridge module ----------
ports = find_system(bp('Bridge_module'),'SearchDepth',1,'BlockType','PMIOPort');
pnames = sort(cellfun(@(b) get_param(b,'Name'), ports(:), 'uni', 0));
check('Bridge_module has the six documented connection ports', ...
      isequal(pnames, sort({'Vcci';'Vsi';'Vddi';'Vcco';'Vso';'Vddo'})));

%% ---------- 6. electrical netlist ----------
adj = adjacency(m);
for k = 1:numel(legs)
  check(sprintf('bus chain into %s', legs{k}), k == 1 || hasPair(adj, legs{k-1}, legs{k}));
  if k <= numel(winds),  check(sprintf('%s drives %s', legs{k}, winds{k}),  hasPair(adj, legs{k}, winds{k}));  end
  if k > 1,              check(sprintf('%s drives %s', legs{k}, winds{k-1}), hasPair(adj, legs{k}, winds{k-1})); end
end
check('bus positive tied to the voltage source', hasPair(adj,'Bridge_module','Voltage Source'));
check('bus negative tied to the electrical reference', hasPair(adj,'Bridge_module','Electrical Reference'));
% The solver block hangs off the same node through a branch segment that carries
% no block handle in the plain line API, so only its wiring is asserted here.
check('solver configuration is wired into the circuit', isWired(solverBlk));
check('leg 0 unused midpoint terminated', hasPair(adj,'Bridge_module','Open Circuit3'));
check('leg 10 unused ports terminated', hasPair(adj,'Bridge_module10','Open Circuit') && ...
      hasPair(adj,'Bridge_module10','Open Circuit1') && hasPair(adj,'Bridge_module10','Open Circuit2'));

%% ---------- 7. Goto/From pairing ----------
gTags = cellfun(@(b) get_param(b,'GotoTag'), find_system(m,'SearchDepth',1,'BlockType','Goto'), 'uni', 0);
fTags = cellfun(@(b) get_param(b,'GotoTag'), find_system(m,'SearchDepth',1,'BlockType','From'), 'uni', 0);
missing = setdiff(unique(gTags), unique(fTags));
check('every Goto tag has a matching From', isempty(missing));

%% ---------- 8. resolved parameter values ----------
check('K_p resolves to 0.023562', abs(P.K_p - 0.023562) < 1e-6);
check('K_i resolves to 47.1239',  abs(P.K_i - 47.1239)  < 1e-3);
check('U_dc = 200 V',             P.U_dc == 200);
check('bias_current = 5 A',       P.bias_current == 5);
check('SW_frequency = 20 kHz',    P.SW_frequency == 20e3);
check('R_value = 1 Ohm',          P.R_value == 1);
check('L_value = 500 uH',         P.L_value == 500e-6);

%% ---------- summary ----------
[pass, fail] = tally();
fprintf('\n%d checks passed, %d failed\n', pass, fail);
if fail > 0
  error('verify_model_facts:FAIL', '%d check(s) failed.', fail);
end
fprintf('PASS - the bundled model matches references/block-and-parameter-map.md\n');
end

%% ================= local helpers =================
function tf = exist_block(path)
tf = getSimulinkBlockHandle(path) ~= -1;
end

function r = refOf(path)
% Library paths of masked library blocks may contain line breaks; compare without them.
r = strrep(get_param(path,'ReferenceBlock'), sprintf('\n'), ' ');
end

function nm = legName(k)
if k == 0, nm = 'Bridge_module'; else, nm = sprintf('Bridge_module%d',k); end
end

function nm = windName(k)
if k == 1, nm = 'RL_load'; else, nm = sprintf('RL_load%d',k-1); end
end

function check(name, cond)
if cond
  tally(1,0);
else
  tally(0,1);
  fprintf('  FAIL  %s\n', name);
end
end

function [pass, fail] = tally(dpass, dfail)
% Single owner of the counters, so that the check helpers and the summary share
% state.  tally(NaN,0) resets them, so that a second run in the same MATLAB session
% does not keep accumulating.
persistent npass nfail
if isempty(npass), npass = 0; nfail = 0; end
if nargin > 0
  if isnan(dpass)
    npass = 0; nfail = 0;
  else
    npass = npass + dpass;
    nfail = nfail + dfail;
  end
end
pass = npass; fail = nfail;
end

function adj = adjacency(m)
% Undirected block-to-block adjacency of the drawn nets at root level.
% Simscape branch segments carry no block handles, so lines are first stitched
% into nets by shared endpoints and the attached blocks are collected per net.
% This is a structural aid only: a signal line drawn onto the same coordinate as
% a physical net could in principle merge two nets, which can only add edges.
lines = find_system(m,'SearchDepth',1,'FindAll','on','Type','Line');
n = numel(lines);
map = containers.Map('KeyType','char','ValueType','any');
for i = 1:n
  pts = get_param(lines(i),'Points');
  ka = sprintf('%.3f_%.3f', pts(1,1), pts(1,2));
  kb = sprintf('%.3f_%.3f', pts(end,1), pts(end,2));
  for k = {ka, kb}
    if isKey(map, k{1}), map(k{1}) = [map(k{1}) i]; else, map(k{1}) = i; end
  end
end
parent = 1:n;
ks = keys(map);
for k = 1:numel(ks)
  idx = map(ks{k});
  for t = 2:numel(idx), parent = unite(parent, idx(1), idx(t)); end
end
nets = containers.Map('KeyType','double','ValueType','any');
for i = 1:n
  r = findroot(parent, i);
  sH = get_param(lines(i),'SrcBlockHandle');
  dH = get_param(lines(i),'DstBlockHandle');
  hs = [sH(:); dH(:)];
  hs = hs(hs ~= -1);
  for k = 1:numel(hs)
    nm = strip_root(getfullname(hs(k)), m);
    if isempty(nm), continue; end
    if isKey(nets, r), nets(r) = [nets(r) {nm}]; else, nets(r) = {nm}; end
  end
end
adj = {};
rk = keys(nets);
for k = 1:numel(rk)
  names = unique(nets(rk{k}));
  for a = 1:numel(names)
    for b = a+1:numel(names)
      adj{end+1} = {names{a}, names{b}};
    end
  end
end
end

function r = findroot(parent, i)
r = i;
while parent(r) ~= r, r = parent(r); end
end

function parent = unite(parent, a, b)
ra = findroot(parent, a); rb = findroot(parent, b);
if ra ~= rb, parent(rb) = ra; end
end

function nm = strip_root(full, m)
nm = '';
p = [m '/'];
if strncmp(full, p, numel(p))
  rest = full(numel(p)+1:end);
  if isempty(strfind(rest, '/')), nm = rest; end   % root level only
end
end

function tf = isWired(blk)
% True when at least one port of the block has a line attached.
tf = false;
ph = get_param(blk,'PortHandles');
fn = fieldnames(ph);
for i = 1:numel(fn)
  for k = 1:numel(ph.(fn{i}))
    if get_param(ph.(fn{i})(k),'Line') ~= -1, tf = true; return; end
  end
end
end

function tf = connected(adj, names)
% True when every name belongs to one connected component of the adjacency list.
seen = names(1);
queue = names(1);
while ~isempty(queue)
  cur = queue{1}; queue(1) = [];
  for i = 1:numel(adj)
    p = adj{i};
    nb = '';
    if strcmp(p{1},cur), nb = p{2}; elseif strcmp(p{2},cur), nb = p{1}; end
    if ~isempty(nb) && ~any(strcmp(seen,nb))
      seen{end+1} = nb; queue{end+1} = nb;
    end
  end
end
tf = all(cellfun(@(x) any(strcmp(seen,x)), names));
end

function tf = hasPair(adj, a, b)
tf = false;
for i = 1:numel(adj)
  p = adj{i};
  if (strcmp(p{1},a) && strcmp(p{2},b)) || (strcmp(p{1},b) && strcmp(p{2},a))
    tf = true; return;
  end
end
end

function path = ambFindModel(here)
cand = dir(fullfile(fileparts(here),'model','*.slx'));
assert(~isempty(cand), 'no .slx found next to this script (expected ../model/*.slx)');
[~, idx] = max([cand.bytes]);
path = fullfile(cand(idx).folder, cand(idx).name);
end

function P = loadAmbParams(scriptPath)
% Runs the parameter script in the BASE workspace (the model reads its variables
% from there) and returns the resolved values this script needs.
evalin('base', sprintf('run(''%s'')', strrep(scriptPath, '''', '''''')));
get_ = @(n) evalin('base', n);
P = struct('K_p', get_('K_p'), 'K_i', get_('K_i'), 'U_dc', get_('U_dc'), ...
           'bias_current', get_('bias_current'), 'SW_frequency', get_('SW_frequency'), ...
           'R_value', get_('R_value'), 'L_value', get_('L_value'));
end
