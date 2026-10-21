function m = b10optb_simcheck()
%B10OPTB_SIMCHECK  Simulate the primary single-star-point ten-leg model and
%   report winding currents.  The packaged model is built with variant 'w9x1',
%   the internal id of the single-star-point ten-leg topology.
%   The logged Mux (and the Scope) has one channel per WINDING, in the order
%   w1..w10: the meter publishes each winding under its own tag iw<winding>.
%   Use the column index (= winding number) directly.
mdl = b10optb_load();
out = sim(mdl);
y = out.iw_log; t = y.time; d = y.signals.values;
S = b10optb_params();
colOfW = 1:10;     % the Mux/Scope channels are already winding w1..w10 (see README)
fprintf('sim: %d samples, t = %.4g .. %.4g s\n', numel(t), t(1), t(end));
i0 = find(t >= 0.8*t(end), 1);
dt = diff(t(i0:end)); x = d(i0:end,:);
m  = sum(x(1:end-1,:).*dt, 1)/sum(dt);     % time-weighted mean, per Mux column
mw = m(colOfW);                            % re-ordered to w1..w10
fprintf('mean winding currents [A] : '); fprintf('%.3f ', mw); fprintf('\n');
fprintf('reference   w1..w10 [A]   : '); fprintf('%.3f ', S.i_ref); fprintf('\n');
fprintf('max abs error            : %.4f A  (%.2f %%)\n', max(abs(mw-S.i_ref)), ...
        100*max(abs(mw-S.i_ref)./S.i_ref));
fprintf('ripple p-p [A]           : '); fprintf('%.3f ', max(x)-min(x)); fprintf('\n');
dof = zeros(size(x,1),5);
for j = 1:5
  a = colOfW(2*j-1); c = colOfW(2*j);
  dof(:,j) = d(i0:end,a) - d(i0:end,c);
end
fprintf('differential i_a-i_c [A] : '); fprintf('%.4f ', mean(dof(1:end-1,:),1)); fprintf('\n');
fprintf('  (reference             : '); fprintf('%.4f ', S.i_ctrl_by_dof); fprintf(')\n');
fprintf('differential ripple p-p  : '); fprintf('%.3f ', max(dof)-min(dof)); fprintf('\n');
wir = max(x) - min(x);
fprintf('winding ripple p-p w1..w10: '); fprintf('%.3f ', wir(colOfW)); fprintf('\n');
save(fullfile(fileparts(mfilename('fullpath')),'results_sstl.mat'), ...
     't','d','m','mw','colOfW');
end
