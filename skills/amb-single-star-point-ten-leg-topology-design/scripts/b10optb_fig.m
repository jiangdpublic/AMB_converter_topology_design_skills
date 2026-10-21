function b10optb_fig()
%B10OPTB_FIG  Plot the simulated winding currents of the primary topology
%   (single-star-point ten-leg, internal id w9x1).
here = fileparts(mfilename('fullpath'));
rfile = fullfile(here,'results_sstl.mat');
S = [];
if exist(rfile,'file'), S = load(rfile); end
if isempty(S) || ~isfield(S,'colOfW')
  b10optb_simcheck();
  S = load(rfile);
end
t = S.t; d = S.d; colOfW = S.colOfW;
P = b10optb_params();
sel = t >= t(end) - 1e-3;                 % last millisecond
f = figure('Position',[80 80 1150 720],'Color','w');
subplot(2,1,1); hold on; grid on; box on;
h = zeros(1,10);
for k = 1:10
  h(k) = plot(1e3*t(sel), d(sel,colOfW(k)), 'LineWidth', 0.9);
end
xlabel('t [ms]'); ylabel('i_w [A]');
title(sprintf('Single-star-point ten-leg (w9x1) winding currents (U_{dc}=%.0f V, bias %.1f A, i_{ctrl} %.2f A/DOF)', ...
      P.U_dc, P.i_bias, P.i_ctrl));
legend(h, arrayfun(@(k) sprintf('w%d',k), 1:10, 'UniformOutput', false), ...
       'Location','eastoutside','NumColumns',2);
subplot(2,1,2); hold on; grid on; box on;
g = zeros(1,5);
for j = 1:5
  a = colOfW(2*j-1); c = colOfW(2*j);
  g(j) = plot(1e3*t(sel), d(sel,a)-d(sel,c), 'LineWidth', 1.1);
end
xlabel('t [ms]'); ylabel('i_a - i_c [A]');
title('differential (control) current of the five DOF');
legend(g, arrayfun(@(j) sprintf('DOF%d',j), 1:5, 'UniformOutput', false), ...
       'Location','eastoutside');
outp = fullfile(b10optb_outdir('assets', mfilename('fullpath')),'fig4_winding_currents.png');
exportgraphics(f,outp,'Resolution',110);
close(f);
fprintf('wrote %s\n', outp);
end
