function b10optb_diag()
%B10OPTB_DIAG  Read-only diagnostics of the primary Simscape model.
%   Prints the solver settings, the bus source value, the device parameters of
%   one leg, the gate-drive settings, the model workspace contents and the port
%   geometry of the main blocks.  Nothing is modified.
mdl = 'SINGLE_STAR_TEN_LEG_AMB';
b10optb_load();
q = @(b,p) getp_(b,p);

fprintf('=== model config ===\n');
fprintf('Solver=%s MaxStep=%s RelTol=%s StopTime=%s\n', ...
  q(mdl,'Solver'), q(mdl,'MaxStep'), q(mdl,'RelTol'), q(mdl,'StopTime'));

fprintf('=== Udc source ===\n');
for p = {'dc_voltage','dc_voltage_conf','dc_voltage_unit','sample_time'}
  fprintf('  %-18s = %s\n', p{1}, q([mdl '/Udc'], p{1}));
end

fprintf('=== Leg1 upper IGBT / upper diode ===\n');
for p = {'Vth','Ron','Vf','Goff'}
  fprintf('  IGU %-13s = %s\n', p{1}, q([mdl '/Leg1/IGU'], p{1}));
end
for p = {'Vf','Ron','Goff'}
  fprintf('  DU  %-13s = %s\n', p{1}, q([mdl '/Leg1/DU'], p{1}));
end

fprintf('=== gate drive ===\n');
fprintf('  cu1  op=%s  out=%s\n', q([mdl '/cu1'],'Operator'), q([mdl '/cu1'],'OutDataTypeStr'));
fprintf('  cl1  op=%s  out=%s\n', q([mdl '/cl1'],'Operator'), q([mdl '/cl1'],'OutDataTypeStr'));
fprintf('  gu1  out=%s   gl1 out=%s\n', q([mdl '/gu1'],'OutDataTypeStr'), ...
        q([mdl '/gl1'],'OutDataTypeStr'));

fprintf('=== model workspace ===\n');
hws = get_param(mdl,'ModelWorkspace');
try
  w = hws.whos; fprintf('  count=%d\n', numel(w));
  for k=1:numel(w), fprintf('    %s\n', w(k).name); end
catch e
  fprintf('  whos failed: %s\n', e.message);
end
for nm = {'U_dc','R_w','L_w','i_bias','i_ctrl'}
  try
    v = hws.getVariable(nm{1}); fprintf('  %-8s = %s\n', nm{1}, mat2str(v));
  catch
    fprintf('  %-8s : ABSENT\n', nm{1});
  end
end

fprintf('=== port geometry ===\n');
show_ports_(mdl, {'Udc','Leg1','W1','GND','Solver'});
end

function show_ports_(mdl, names)
for k = 1:numel(names)
  b = [mdl '/' names{k}];
  ph = get_param(b,'PortHandles');
  fprintf('  --- %s (%s)\n', names{k}, get_param(b,'BlockType'));
  allp = [ph.LConn ph.RConn ph.Inport ph.Outport];
  n = numel(allp);
  for j = 1:n
    pos = get_param(allp(j),'Position');
    typ = '?'; nm = '';
    try, typ = get_param(allp(j),'PortType'); catch, end
    try, nm  = get_param(allp(j),'Name'); catch, end
    if isempty(nm), nm = '<none>'; end
    fprintf('      [%d] pos=[%4d %4d] type=%-6s name=%s\n', j, pos(1), pos(2), typ, nm);
  end
end
end

function s = getp_(b,p)
try
  v = get_param(b,p);
  if ischar(v), s = v;
  elseif isnumeric(v) || islogical(v), s = mat2str(v);
  else, s = ['<' class(v) '>'];
  end
catch e
  s = ['ERR:' e.message];
end
end
