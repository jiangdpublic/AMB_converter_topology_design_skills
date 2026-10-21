function b10optb_porttest()
%B10OPTB_PORTTEST  Empirical orientation probes for the ten-leg model rebuild.
%   These probes establish the device port roles used by b10optb_build_all.
load_system('ee_lib'); load_system('fl_lib'); load_system('nesl_utility');
IG = blk('ee_lib','IGBT (Ideal, Switching)',4);
DI = blk('fl_lib','Diode',5);
CS = blk('fl_lib','Current Sensor',5);
RE = blk('fl_lib','Resistor',5);
SP = blk('nesl_utility','Simulink-PS Converter',2);
PS = blk('nesl_utility','PS-Simulink Converter',2);
GND= 'fl_lib/Electrical/Electrical Elements/Electrical Reference';

fprintf('--- test 1: pure resistor (source polarity x current-sensor polarity)\n');
i = run_case('t1', IG, DI, CS, RE, SP, PS, GND, 'V.L1 -> R -> CS -> V.R1', 0);
fprintf('--- test 2: IGBT RConn1 top / RConn2 bottom, gate 0 and 15 V\n');
i0 = run_case('t2', IG, DI, CS, RE, SP, PS, GND, 'V.L1 -> IG.R1 ; IG.R2 -> R -> CS -> V.R1', 0);
i1 = run_case('t2', IG, DI, CS, RE, SP, PS, GND, 'V.L1 -> IG.R1 ; IG.R2 -> R -> CS -> V.R1', 15);
fprintf('--- test 3: IGBT RConn2 top / RConn1 bottom, gate 15 V\n');
i2 = run_case('t3', IG, DI, CS, RE, SP, PS, GND, 'V.L1 -> IG.R2 ; IG.R1 -> R -> CS -> V.R1', 15);
fprintf('--- test 4: diode LConn1 -> RConn1 forward, gate n/a\n');
i3 = run_case('t4', IG, DI, CS, RE, SP, PS, GND, 'V.L1 -> D.L1 ; D.R1 -> R -> CS -> V.R1', 0);

fprintf('\n--- branch-line test ---\n');
mdl2 = 'b10optb_probe2';
if bdIsLoaded(mdl2), close_system(mdl2,0); end
new_system(mdl2); load_system(mdl2);          % never saved -> nothing to delete
add_block('simulink/Sources/Constant',[mdl2 '/A'],'Position',[100 100 140 120]);
add_block('simulink/Sinks/Display',[mdl2 '/B'],'Position',[500 80 540 100]);
add_block('simulink/Sinks/Display',[mdl2 '/C'],'Position',[500 200 540 220]);
LB = @(b) get_param([mdl2 '/' b],'PortHandles');
h1 = add_line(mdl2, LB('A').Outport(1), LB('B').Inport(1));
p1 = get_param(h1,'Points'); fprintf('  trunk points: %s\n', mat2str(p1));
drawnow;
ok = false;
try
  h2 = add_line(mdl2, [p1(1,1)+20 p1(1,2)], LB('C').Inport(1));
  ok = true;
catch e
  fprintf('  branch via point FAILED: %s\n', e.message);
end
if ok
  fprintf('  branch OK: SrcPort branch=%g trunk=%g ; SrcBlock branch=%g trunk=%g\n', ...
    get_param(h2,'SrcPortHandle'), get_param(h1,'SrcPortHandle'), ...
    get_param(h2,'SrcBlockHandle'), get_param(h1,'SrcBlockHandle'));
  fprintf('  lines now: %d\n', numel(find_system(mdl2,'FindAll','on','Type','line')));
  fprintf('  branch points: %s\n', mat2str(get_param(h2,'Points')));
end
close_system(mdl2,0);
end

function ilast = run_case(tag, IG, DI, CS, RE, SP, PS, GND, spec, vg)
mdl = ['b10optb_' tag];
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl); load_system(mdl);            % never saved -> nothing to delete
add_block('ee_lib/Sources/Voltage Source',[mdl '/V'],'Position',[40 150 90 250]);
set_param([mdl '/V'],'dc_voltage','10');
add_block(GND,[mdl '/GND'],'Position',[40 330 90 360]);
add_block('nesl_utility/Solver Configuration',[mdl '/Solver'],'Position',[40 400 90 440]);
add_block(RE,[mdl '/R'],'Position',[300 200 360 240]); set_param([mdl '/R'],'R','1');
add_block(CS,[mdl '/CS'],'Position',[400 200 460 240]);
add_block(PS,[mdl '/PS'],'Position',[400 100 460 130]);
add_block('simulink/Sinks/To Workspace',[mdl '/log'],'Position',[520 100 600 130]);
set_param([mdl '/log'],'VariableName','ip','SaveFormat','Structure With Time');
L = @(b) get_param([mdl '/' b],'PortHandles');
switch tag
  case 't1'
    add_line(mdl, L('V').LConn(1), L('R').LConn(1));
    add_line(mdl, L('R').RConn(1), L('CS').LConn(1));
  case 't2'
    add_block(IG,[mdl '/IG'],'Position',[160 160 230 240]);
    add_block(SP,[mdl '/SP'],'Position',[160 60 220 90]);
    add_block('simulink/Sources/Constant',[mdl '/Vg'],'Position',[40 65 90 85]);
    set_param([mdl '/Vg'],'Value',num2str(vg));
    add_line(mdl, L('V').LConn(1), L('IG').RConn(1));
    add_line(mdl, L('IG').RConn(2), L('R').LConn(1));
    add_line(mdl, L('R').RConn(1), L('CS').LConn(1));
    add_line(mdl, L('Vg').Outport(1), L('SP').Inport(1));
    add_line(mdl, L('SP').RConn(1), L('IG').LConn(1));
  case 't3'
    add_block(IG,[mdl '/IG'],'Position',[160 160 230 240]);
    add_block(SP,[mdl '/SP'],'Position',[160 60 220 90]);
    add_block('simulink/Sources/Constant',[mdl '/Vg'],'Position',[40 65 90 85]);
    set_param([mdl '/Vg'],'Value',num2str(vg));
    add_line(mdl, L('V').LConn(1), L('IG').RConn(2));
    add_line(mdl, L('IG').RConn(1), L('R').LConn(1));
    add_line(mdl, L('R').RConn(1), L('CS').LConn(1));
    add_line(mdl, L('Vg').Outport(1), L('SP').Inport(1));
    add_line(mdl, L('SP').RConn(1), L('IG').LConn(1));
  case 't4'
    add_block(DI,[mdl '/D'],'Position',[200 200 260 240]);
    add_line(mdl, L('V').LConn(1), L('D').LConn(1));
    add_line(mdl, L('D').RConn(1), L('R').LConn(1));
    add_line(mdl, L('R').RConn(1), L('CS').LConn(1));
end
add_line(mdl, L('CS').RConn(2), L('V').RConn(1));
add_line(mdl, L('V').RConn(1), L('GND').LConn(1));
add_line(mdl, L('Solver').RConn(1), L('GND').LConn(1));
add_line(mdl, L('CS').RConn(1), L('PS').LConn(1));
add_line(mdl, L('PS').Outport(1), L('log').Inport(1));
set_param(mdl,'StopTime','1e-3','Solver','ode23t','MaxStep','1e-6','RelTol','1e-6');
out = sim(mdl); y = out.ip; d = y.signals.values;
ilast = d(end);
fprintf('  %-34s vg=%2d -> i = %+9.4f A\n', spec, vg, ilast);
close_system(mdl,0);
end

function p = blk(lib,nm,d)
c = find_system(lib,'LookUnderMasks','all','FollowLinks','on','SearchDepth',d,'Name',nm);
p = strrep(c{1},char(10),' ');
end
