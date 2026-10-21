function b10optb_fig_legs()
%B10OPTB_FIG_LEGS  Leg currents of the primary topology and the two references.
%   primary  'w9x1'  single-star-point ten-leg: all five pairs clean, 5 A class
%   ref 'B'          B10-OPT-B: lowest bus, DOF5 opposite pair on L3 (0.5 A)
%   ref 'd569'       direction_569 = -1: all clean, but L3 carries 10 A
%   The leg currents are reconstructed from the ten winding currents with the
%   winding-node incidence matrix, so the figure shows exactly the trade-off
%   the topology is built around: ripple versus leg current.
modes = {'w9x1','B','d569'};
ttl   = {'primary: single-star-point ten-leg (w9x1): all five pairs clean, 5 A class', ...
         'reference B10-OPT-B: lowest bus, DOF5 opposite pair on L3 (0.5 A ripple)', ...
         'reference direction\_569 = -1: all clean, but the shared legs carry 10 A'};
nd = {'X','L1','L2','L3','L4','L5','L6','L7','L8','L9','L10'};
res = cell(1,numel(modes));
for q = 1:numel(modes)
  b10optb_build_all(modes{q},'nosim');
  out = sim('SINGLE_STAR_TEN_LEG_AMB');
  y = out.iw_log; t = y.time; iw = y.signals.values;      % columns = w1..w10
  E = b10optb_netlist(modes{q});
  wOfRow = zeros(1,10);
  for k = 1:10, wOfRow(k) = sscanf(E{k,1},'w%d'); end
  A = zeros(10,11);
  for k = 1:10
    a = find(strcmp(nd,E{k,2}),1); b = find(strcmp(nd,E{k,3}),1);
    A(k,a) = 1; A(k,b) = -1;
  end
  iE = iw(:,wOfRow);                 % reorder to netlist-row order
  IL = iE * A(:,2:11);               % leg currents L1..L10
  % differential currents i_a - i_c, in winding order
  id = iw(:,1:2:9) - iw(:,2:2:10);
  res{q} = struct('t',t,'IL',IL,'id',id,'iw',iw);
  i0 = find(t >= 0.8*t(end),1);
  fprintf('%-6s leg |I| max %.2f A (bias)  |  differential ripple p-p ', modes{q}, ...
          max(abs(mean(IL(i0:end,:),1))));
  fprintf('%.3f ', max(id(i0:end,:))-min(id(i0:end,:))); fprintf('\n');
end
f = figure('Position',[60 60 1150 1000],'Color','w');
for q = 1:numel(modes)
  subplot(numel(modes),1,q); hold on; grid on; box on;
  sel = res{q}.t >= res{q}.t(end) - 1e-3;
  hs = zeros(1,10);
  for j = 1:10
    hs(j) = plot(1e3*res{q}.t(sel), res{q}.IL(sel,j), 'LineWidth', 0.9);
  end
  ylabel('leg current [A]');
  title(ttl{q},'Interpreter','tex');
  if q == numel(modes), xlabel('t [ms]'); end
  legend(hs, arrayfun(@(j) sprintf('L%d',j), 1:10, 'UniformOutput', false), ...
         'Location','eastoutside','NumColumns',2);
end
outdir = b10optb_outdir('assets', mfilename('fullpath'));
outp = fullfile(outdir,'fig8_leg_currents.png');
exportgraphics(f,outp,'Resolution',110); close(f);
fprintf('wrote %s\n', outp);
plot_leg_zoom(res{1}, outdir);          % supplementary zoom of the primary L6
b10optb_build_all('w9x1','nosim');      % leave the primary model in place
end

function plot_leg_zoom(R, outdir)
%PLOT_LEG_ZOOM  Supplementary figure for fig8: show that L6 is a LEG current
%   built from w5 + w6 - w9, and that its narrow notch is a finite-width PWM
%   pulse, not a one-sample solver spike.
t = R.t; iw = R.iw; IL = R.IL;
i0 = find(t >= t(end)-1e-3, 1);
[~,ix] = max(abs(diff(IL(i0:end,6))));
te = t(i0+ix-1);
sel = t >= te-4e-5 & t <= te+4e-5;      % +/-40 us around the L6 event
f = figure('Position',[80 80 1100 760],'Color','w');
subplot(2,1,1); hold on; grid on; box on;
h1 = plot(1e3*t(sel), iw(sel,5), '.-', 'LineWidth',1.0);
h2 = plot(1e3*t(sel), iw(sel,6), '.-', 'LineWidth',1.0);
h3 = plot(1e3*t(sel), iw(sel,9), '.-', 'LineWidth',1.0);
ylabel('winding current [A]');
title('Zoom of fig8: w5, w6 and w9 are smooth winding currents (20 kHz sawtooth)');
legend([h1 h2 h3], {'w5','w6','w9'}, 'Location','best');
subplot(2,1,2); hold on; grid on; box on;
plot(1e3*t(sel), IL(sel,6), '.-', 'LineWidth',1.2, 'Color',[0 0.45 0.74]);
xlabel('t [ms]'); ylabel('leg current [A]');
title('L6 = w5 + w6 - w9: the narrow notch is the summed differential PWM pulse, not a solver spike');
outp = fullfile(outdir,'fig8b_leg_currents_zoom.png');
exportgraphics(f,outp,'Resolution',130); close(f);
fprintf('wrote %s\n', outp);
end
