function make_topology_figure()
% Regenerates assets/fig1_topology.png: the winding/leg graph of the
% ten-phase-nine-leg topology as it is wired in model/Ten_phase_nine_leg_with_RL_load.slx.
%
% Relocatable: the output folder is located relative to this file. No model is
% loaded and no simulation is performed.
%
% Verified with MATLAB R2025b (25.2), Simulink 25.2, Simscape 25.2 and
% Simscape Electrical 25.2 (Linux glnxa64).

Here   = fileparts(mfilename('fullpath'));
Assets = fullfile(fileparts(Here),'assets');
if ~exist(Assets,'dir'), mkdir(Assets); end

N = struct('L1',[0.5 3.5],'L2',[0.5 6.5],'L3',[2.0 7.8],'L4',[3.5 5.0],'L5',[5.0 5.0], ...
           'L6',[6.5 5.0],'L7',[8.0 7.8],'L8',[9.5 6.5],'L9',[9.5 3.5],'X1',[2.0 5.0],'X2',[8.0 5.0]);
E = { 'L1','X1','w1'; 'L2','X1','w2'; 'X1','L3','w3'; 'X1','L4','w4'; ...
      'L4','L5','w5'; 'L5','L6','w6'; 'L6','X2','w7'; 'L7','X2','w8'; ...
      'X2','L8','w9'; 'X2','L9','w10' };

f = figure('Visible','off','Position',[60 60 1150 620]); hold on; axis off; axis equal;
plot([0.2 10.2],[9.6 9.6],'k-','LineWidth',1.6);
text(5.2,9.85,'common DC bus  U_{dc} = 200 V','HorizontalAlignment','center','FontSize',10);
plot([0.2 10.2],[2.4 2.4],'k-','LineWidth',1.6);
text(5.2,2.05,'DC- rail (0 V, Electrical Reference)','HorizontalAlignment','center','FontSize',10);
for i = 1:size(E,1)
    a = N.(E{i,1}); b = N.(E{i,2});
    plot([a(1) b(1)],[a(2) b(2)],'-','Color',[0.15 0.35 0.75],'LineWidth',1.6);
    mx = 0.5*(a(1)+b(1)); my = 0.5*(a(2)+b(2));
    ang = atan2(b(2)-a(2), b(1)-a(1));
    R = 0.16;
    th = linspace(0,4*pi,80);
    px = mx + (0.34*cos(th))*cos(ang) - R*sin(th)*sin(ang);
    py = my + (0.34*cos(th))*sin(ang) + R*sin(th)*cos(ang);
    plot(px,py,'-','Color',[0.75 0.15 0.15],'LineWidth',1.3);
    text(mx, my-0.42, E{i,3}, 'HorizontalAlignment','center','FontSize',10, ...
         'Color',[0.75 0.15 0.15],'FontWeight','bold');
end
fn = fieldnames(N);
for i = 1:numel(fn)
    p = N.(fn{i});
    if fn{i}(1) == 'X'
        plot(p(1),p(2),'ks','MarkerSize',11,'MarkerFaceColor',[0.95 0.85 0.2]);
        text(p(1), p(2)-0.62, fn{i}, 'HorizontalAlignment','center','FontSize',12,'FontWeight','bold');
        text(p(1), p(2)-0.95,'(hard-wired star node)','HorizontalAlignment','center','FontSize',8);
    else
        plot(p(1),p(2),'s','MarkerSize',9,'MarkerFaceColor',[0.55 0.75 0.95]);
        lab = sprintf('%s\nhalf-bridge', fn{i});
        if strcmp(fn{i},'L5'), lab = sprintf('%s\nextra leg', fn{i}); end
        text(p(1), p(2)+0.30, lab, 'HorizontalAlignment','center','FontSize',9);
        plot([p(1) p(1)],[p(2)+0.16 9.6],':','Color',[0.6 0.6 0.6]);
        plot([p(1) p(1)],[2.4 p(2)-0.16],':','Color',[0.6 0.6 0.6]);
    end
end
text(5.2, 1.2, 'control currents:  i_{a1}-i_{c1} ... i_{a5}-i_{c5} ;  every pair sum i_a+i_c = 2 i_{bias} = 10 A', ...
     'HorizontalAlignment','center','FontSize',10);
text(0.5, 10.6, 'AMB 9-leg / 10-winding (ten-phase nine-leg) topology as wired in the bundled model', ...
     'FontSize',13,'FontWeight','bold');
print(f,'-dpng','-r130',fullfile(Assets,'fig1_topology.png'));
close(f);
fprintf('make_topology_figure: wrote %s\n', fullfile('assets','fig1_topology.png'));
end