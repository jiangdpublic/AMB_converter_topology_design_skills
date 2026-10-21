% Draws assets/fig1_topology.png: the device-level structure of the reversed
% shared bridge (one shared leg, one T module, one B module, one winding
% pair).  Pure MATLAB graphics - no toolbox and no model needed.
%
% Relocatable: the figure is written next to this script, into ../assets.
%
% Verified with MATLAB R2025b (25.2).

Here = fileparts(mfilename('fullpath'));
outDir = fullfile(Here,'..','assets');
if ~exist(outDir,'dir'), mkdir(outDir); end
outFile = fullfile(outDir,'fig1_topology.png');

f = figure('Visible','off','Position',[100 100 1280 760],'Color','w');
ax = axes('Parent',f,'Position',[0.02 0.02 0.96 0.96]);
hold(ax,'on'); axis(ax,[0 12.9 0 8]); axis(ax,'off');

cSwitch = [0.85 0.92 1.00];
cDiode  = [1.00 0.93 0.85];
cWind   = [0.90 0.97 0.90];
cRail   = [0.15 0.15 0.15];
yP = 7.40; yN = 1.10; ySh = 4.20;

% ---------- bus rails ----------
line([0.7 11.5],[yP yP],'Color',cRail,'LineWidth',2.5);
text(0.55,yP,'P  (+U_{dc})','HorizontalAlignment','right','FontSize',11,'FontWeight','bold');
line([0.7 11.5],[yN yN],'Color',cRail,'LineWidth',2.5);
text(0.55,yN,'N  (0 V)','HorizontalAlignment','right','FontSize',11,'FontWeight','bold');

% ---------- shared node ----------
line([1.3 10.4],[ySh ySh],'Color',[0.75 0.1 0.1],'LineWidth',2.5);
text(10.95,ySh-0.05,'  Shared node','Color',[0.75 0.1 0.1],'FontSize',11,'FontWeight','bold', ...
     'HorizontalAlignment','left');

% ---------- shared leg (x = 2.0) ----------
xs = 2.0; xd = 3.05;
line([xs xs],[yP 6.75],'Color',cRail,'LineWidth',1.4);
drawBox(xs-0.42,5.95,0.84,0.80,'S_{up}',cSwitch);
line([xs xs],[5.95 ySh],'Color',cRail,'LineWidth',1.4);
line([xs xs],[ySh 2.70],'Color',cRail,'LineWidth',1.4);
drawBox(xs-0.42,1.90,0.84,0.80,'S_{low}',cSwitch);
line([xs xs],[1.90 yN],'Color',cRail,'LineWidth',1.4);
line([xd xd],[yP 6.75],'Color',cRail,'LineWidth',1.4);
drawBox(xd-0.42,5.95,0.84,0.80,'D_{up}',cDiode);
line([xd xd],[5.95 ySh],'Color',cRail,'LineWidth',1.4);
line([xd xd],[ySh 2.70],'Color',cRail,'LineWidth',1.4);
drawBox(xd-0.42,1.90,0.84,0.80,'D_{low}',cDiode);
line([xd xd],[1.90 yN],'Color',cRail,'LineWidth',1.4);
text(3.55,6.85,'shared leg','FontSize',11,'FontWeight','bold','Color',[0.1 0.3 0.6]);

% ---------- T module (x = 6.2) ----------
xt = 6.2;
line([xt xt],[yP 6.75],'Color',cRail,'LineWidth',1.4);
drawBox(xt-0.52,5.95,1.04,0.80,'S  (upper)',cSwitch);
line([xt xt],[5.95 5.05],'Color',cRail,'LineWidth',1.4);
drawBox(xt-1.10,4.55,2.20,0.50,'winding   R, L, sensor',cWind);
line([xt xt],[4.55 ySh],'Color',cRail,'LineWidth',1.4);
line([xt xt],[ySh 2.70],'Color',cRail,'LineWidth',1.4);
drawBox(xt-0.52,1.90,1.04,0.80,'D  (lower)',cDiode);
line([xt xt],[1.90 yN],'Color',cRail,'LineWidth',1.4);
text(xt-0.06,5.05,'X','FontSize',12,'FontWeight','bold','HorizontalAlignment','right');
quiver(xt-0.75,4.15,0,-0.55,0,'Color',[0 0.45 0],'LineWidth',1.6,'MaxHeadSize',2.5);
text(xt-0.85,4.05,'i_a   into the shared node','FontSize',10,'Color',[0 0.45 0],'HorizontalAlignment','right');
text(xt,6.90,'T module','FontSize',12,'FontWeight','bold','HorizontalAlignment','center','Color',[0.1 0.3 0.6]);

% ---------- B module (x = 9.8) ----------
xb = 9.8;
line([xb xb],[yP 6.75],'Color',cRail,'LineWidth',1.4);
drawBox(xb-0.52,5.95,1.04,0.80,'D  (upper)',cDiode);
line([xb xb],[5.95 5.05],'Color',cRail,'LineWidth',1.4);
drawBox(xb-1.10,4.55,2.20,0.50,'winding   R, L, sensor',cWind);
line([xb xb],[4.55 ySh],'Color',cRail,'LineWidth',1.4);
line([xb xb],[ySh 2.70],'Color',cRail,'LineWidth',1.4);
drawBox(xb-0.52,1.90,1.04,0.80,'S  (lower)',cSwitch);
line([xb xb],[1.90 yN],'Color',cRail,'LineWidth',1.4);
text(xb-0.06,5.05,'Y','FontSize',12,'FontWeight','bold','HorizontalAlignment','left');
quiver(xb-0.75,4.05,0,0.55,0,'Color',[0.6 0.1 0.1],'LineWidth',1.6,'MaxHeadSize',2.5);
text(xb-0.85,4.15,'i_c   out of the shared node','FontSize',10,'Color',[0.6 0.1 0.1],'HorizontalAlignment','right');
text(xb,6.90,'B module (reversed)','FontSize',12,'FontWeight','bold','HorizontalAlignment','center','Color',[0.7 0.3 0.1]);

% ---------- notes ----------
text(6.1,3.55,'I_{shared}  =  \Sigma i (T modules)  -  \Sigma i (B modules)  =  2{\cdot}bias{\cdot}(p - q)  +  \Sigma_{split pairs} \pmi_{ref}', ...
     'HorizontalAlignment','center','FontSize',11,'FontWeight','bold');
text(6.1,3.10,'with 5 winding pairs and 5 T + 5 B modules  (p,q,s) = (2,2,1):   the bias terms cancel and only the control currents remain in the shared leg', ...
     'HorizontalAlignment','center','FontSize',10);
text(6.1,0.82,'Bias + differential references:   i_{a,ref} = bias + 0.5{\cdot}i_{ref} ,    i_{c,ref} = bias - 0.5{\cdot}i_{ref}', ...
     'HorizontalAlignment','center','FontSize',10,'Color',[0.25 0.25 0.25]);
text(6.1,0.52,'Winding current is unidirectional by construction: a T module pushes current into the shared node, a B module pulls it out of it.', ...
     'HorizontalAlignment','center','FontSize',10,'Color',[0.25 0.25 0.25]);
text(6.1,0.22,'Each module leg is one switch + one freewheel diode and needs no dead time; only the bidirectional shared leg does.', ...
     'HorizontalAlignment','center','FontSize',10,'Color',[0.25 0.25 0.25]);

exportgraphics(f,outFile,'Resolution',130);
close(f);
fprintf('wrote %s\n', outFile);

function drawBox(x0,y0,w,h,label,color)
rectangle('Position',[x0 y0 w h],'Curvature',0.18,'FaceColor',color,'EdgeColor',[0.25 0.25 0.25],'LineWidth',1.1);
text(x0+w/2,y0+h/2,label,'HorizontalAlignment','center','FontSize',10);
end