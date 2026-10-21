function b10optb_devtest()
%B10OPTB_DEVTEST  Swap the near-ideal devices for realistic ones on the CLOSED
%   loop and check that the winding currents are still on their references.
mdl = 'SINGLE_STAR_TEN_LEG_AMB';
if bdIsLoaded(mdl), close_system(mdl,0); end    % reload from the bundled file
b10optb_load();
for k = 1:10
  set_param(sprintf('%s/Leg%d/IGU',mdl,k),'Vf','0.8','Ron','0.001');
  set_param(sprintf('%s/Leg%d/IGL',mdl,k),'Vf','0.8','Ron','0.001');
  set_param(sprintf('%s/Leg%d/DU',mdl,k), 'Vf','0.6','Ron','0.3');
  set_param(sprintf('%s/Leg%d/DL',mdl,k), 'Vf','0.6','Ron','0.3');
end
fprintf('devices: IGBT Vf=0.8 Ron=1m, diode Vf=0.6 Ron=0.3 ohm\n');
out = sim(mdl);
y = out.iw_log; t = y.time; d = y.signals.values;
S = b10optb_params('w9x1');
colOfW = 1:10;      % the Mux/Scope channels are already winding w1..w10
i0 = find(t >= 0.8*t(end),1);
dt = diff(t(i0:end)); xx = d(i0:end,:);
mw = (sum(xx(1:end-1,:).*dt,1)/sum(dt)); mw = mw(colOfW);
fprintf('closed loop + realistic devices, currents w1..w10 [A]: '); fprintf('%.3f ', mw); fprintf('\n');
fprintf('reference                                    [A]: '); fprintf('%.3f ', S.i_ref); fprintf('\n');
fprintf('max error = %.4f A (%.2f %%)\n', max(abs(mw-S.i_ref)), 100*max(abs(mw-S.i_ref)./S.i_ref));
close_system(mdl,0);
end
