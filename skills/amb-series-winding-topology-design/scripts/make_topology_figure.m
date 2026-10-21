function make_topology_figure(outPath)
% Regenerates assets/fig1_topology.png, the schematic of the series-winding chain.
% Purely parametric: it draws from the documented geometry, so no simulation and no
% model access are required.
%
% Verified with MATLAB/Simulink R2025b (25.2), Simscape and Simscape Electrical 25.2.

if nargin < 1
  Here = fileparts(mfilename('fullpath'));
  outPath = fullfile(fileparts(Here),'assets','fig1_topology.png');
end

nLeg   = 11;          % legs
nWind  = nLeg - 1;    % windings
yTop = 3; yBot = 1; yMid = 2;

set(0,'defaultAxesFontSize',9);
f = figure('Visible','off','Position',[0 0 1100 420]);
hold on; axis equal; axis off;
plot([-0.5 nLeg-0.5],[yTop yTop],'k','LineWidth',1.5);
plot([-0.5 nLeg-0.5],[yBot yBot],'k','LineWidth',1.5);
text(nLeg-0.3, yTop, '+U_{dc}','FontSize',9);
text(nLeg-0.3, yBot, '0','FontSize',9);
for k = 0:nLeg-1
  x = k;
  plot([x x],[yBot yTop],'Color',[0.4 0.4 0.4],'LineWidth',0.8);
  rectangle('Position',[x-0.16 yMid-0.28 0.32 0.56],'FaceColor',[0.92 0.92 0.92], ...
            'EdgeColor','k','LineWidth',0.8);
  text(x, yMid, sprintf('L%d',k),'HorizontalAlignment','center','FontSize',8);
  if k < nWind
    xa = k+0.16; xb = k+0.84;
    xx = linspace(xa,xb,60);
    yy = yMid + 0.55 + 0.12*sin((xx-xa)/(xb-xa)*6*pi);
    plot(xx,yy,'Color',[0 0.45 0.2],'LineWidth',1.3);
    text((xa+xb)/2, yMid+0.95, sprintf('w%d',k+1),'HorizontalAlignment','center', ...
         'FontSize',8,'Color',[0 0.45 0.2]);
  end
end
title('Series-winding chain: 11 half-bridge legs, 10 windings in series (single DC bus)');
xlabel('leg index');
exportgraphics(f, outPath, 'Resolution', 140);
close(f);
fprintf('wrote %s\n', outPath);
end