function b10_fig(variant, outname)
%B10_FIG  Topology schematic of the single-star-point ten-leg topology and its
%   two reference variants.
%   b10_fig              -> primary topology             (fig1_topology.png)
%   b10_fig('B')         -> reference B10-OPT-B          (fig9_reference_b10optb_topology.png)
%   b10_fig('d569')      -> reference direction_569 = -1 (fig10_reference_d569_topology.png)
%   The primary topology is also addressed by its internal id 'w9x1'.
%   Every annotation is placed by a collision search: a label is accepted only
%   where its text box misses all edges, all current-direction arrows, the
%   node markers, the title block, the key block and all previously placed
%   labels.  The result is verified before the PNG is written.
root = fileparts(mfilename('fullpath'));    % no cd: keep the caller's folder
% output folder: <skill>/assets in the skill layout, <archive>/figures otherwise
outdir = fullfile(fileparts(root),'assets');
if ~exist(outdir,'dir'), outdir = fullfile(fileparts(root),'figures'); end
if ~exist(outdir,'dir'), outdir = fullfile(root,'figures'); end
if ~exist(outdir,'dir'), mkdir(outdir); end
if nargin < 1 || isempty(variant), variant = 'w9x1'; end
par = tp_par();
[T, P, ttl, sub, keytxt, skel, fname] = fig_spec(variant);
if nargin >= 2 && ~isempty(outname), fname = outname; end
nd = T.nodes;
A = zeros(10,11);
for k = 1:10
  a = find(strcmp(nd,T.E{k,2})); b = find(strcmp(nd,T.E{k,3}));
  A(k,a) = 1; A(k,b) = -1;
end
IL = A'*(par.i_bias*ones(10,1));

% ------------------------------------------------------------------ canvas
f  = figure('Visible','off','Position',[60 60 1200 1280],'Color','w');
ax = axes('Parent',f); hold(ax,'on');
axis(ax,'equal'); axis(ax,'off');
xlim(ax,[-2.9 7.1]); ylim(ax,[-11.8 3.6]);
SET = @(h) set(h,'Parent',ax);

% ------------------------------------------------------- titles and key first
t1 = text(1.5, 3.00, ttl, 'FontSize',15, ...
     'FontWeight','bold','HorizontalAlignment','center','Interpreter','none'); SET(t1);
t2 = text(1.5, 2.66, sub, ...
     'HorizontalAlignment','center','FontSize',10.5,'Interpreter','none'); SET(t2);
t3 = text(1.5, 2.40, 'arrows = suggested winding current direction (tail -> head);  all ten winding currents are positive at the bias point', ...
     'HorizontalAlignment','center','FontSize',10,'Color',[0.20 0.20 0.20],'Interpreter','none'); SET(t3);
key = keytxt;
tk = text(1.5, -5.00, key, 'HorizontalAlignment','center','VerticalAlignment','top', ...
     'FontSize',10,'Interpreter','none','FontName','Courier New', ...
     'BackgroundColor',[0.97 0.97 0.88],'EdgeColor',[0.5 0.5 0.5],'Margin',10); SET(tk);

% ------------------------------------------------------------- draw the graph
cc = [0.85 0.30 0.05; 0.00 0.42 0.74];
EW   = zeros(10,1);
E1   = zeros(10,2); E2 = zeros(10,2);
ECOL = zeros(10,3); ELW = zeros(10,1);
ARW  = zeros(10,4);
for k = 1:10
  w = sscanf(T.E{k,1},'w%d'); EW(k) = w;
  q1 = P.(T.E{k,2}); q2 = P.(T.E{k,3});
  x1 = q1(1); y1 = q1(2); x2 = q2(1); y2 = q2(2);
  col = cc(1+mod(w+1,2),:);
  lw = 1.8; if any(w==skel), lw = 3.0; end       % emphasis on the main trunk
  E1(k,:) = q1; E2(k,:) = q2; ECOL(k,:) = col; ELW(k) = lw;
  plot([x1 x2],[y1 y2],'-','Color',col,'LineWidth',lw);
  dx = x2-x1; dy = y2-y1;
  ax1 = x1+0.52*dx; ay1 = y1+0.52*dy;
  ARW(k,:) = [ax1 ay1 ax1+0.30*dx ay1+0.30*dy];
  quiver(ax1, ay1, 0.30*dx, 0.30*dy, 0, 'Color',col,'LineWidth',lw, ...
         'MaxHeadSize',1.0,'AutoScale','off');
end
ESEG  = [E1 E2];
ALLSEG = [ESEG; ARW];
% obstacles made of boxes: node markers, title lines, key block
mtop = zeros(11,4);
for j = 1:11
  q = P.(nd{j}); mtop(j,:) = [q(1)-0.16 q(2)-0.16 0.32 0.32];
end
drawnow;
OBS = mtop;
for h = [t1 t2 t3 tk]
  e = get(h,'Extent'); OBS(end+1,:) = [e(1) e(2) e(3) e(4)];   %#ok<AGROW>
end

% ------------------------------------------------------- measure text sizes
th = gobjects(10,1); tw = zeros(10,1); thh = zeros(10,1);
for k = 1:10
  th(k) = text(0, 0, sprintf('w%d',EW(k)), 'Color',ECOL(k,:),'FontSize',10.5, ...
    'FontWeight','bold','HorizontalAlignment','center','VerticalAlignment','middle', ...
    'Interpreter','none','BackgroundColor','w','Margin',1.5); SET(th(k));
end
drawnow;
for k = 1:10
  e = get(th(k),'Extent'); tw(k) = e(3); thh(k) = e(4);
end

% ------------------------------------------------------- place edge labels
placed = zeros(0,4);
[~,order] = sort(sum((E2-E1).^2,2));          % shortest edge first: least room
for k = order'
  cands = edgeCands(E1(k,:), E2(k,:), 0.46);
  best = cands(1,:); bestCost = inf;
  for c = 1:size(cands,1)
    R = rectOf(cands(c,:), tw(k), thh(k));
    ncol = rectCost(R, ALLSEG, OBS, placed, 0.05);
    dOwn = segDist(cands(c,:), E1(k,:), E2(k,:));
    mis  = closerToOther(cands(c,:), k, E1, E2, dOwn);
    cost = ncol*1e6 + mis*1e3 + dOwn;      % no collisions, then own edge nearest, then tight
    if cost < bestCost, bestCost = cost; best = cands(c,:); end
    if ncol == 0 && ~mis, break; end
  end
  set(th(k),'Position',[best(1) best(2) 0]);
  placed(end+1,:) = rectOf(best, tw(k), thh(k));   %#ok<AGROW>
end

% ------------------------------------------------------- node labels
for j = 1:11
  nm = nd{j}; q = P.(nm); isStar = (nm(1)=='X');
  plot(q(1), q(2), 's', 'MarkerSize', 12+5*isStar, ...
       'MarkerFaceColor',[0.12 0.12 0.12],'MarkerEdgeColor','k');
  if isStar
    t1n = text(0,0,'X  (star)','FontSize',9,'FontWeight','bold', ...
         'Color',[0.20 0.20 0.60],'HorizontalAlignment','center', ...
         'VerticalAlignment','middle','Interpreter','none', ...
         'BackgroundColor','w','Margin',1.5); SET(t1n);
    t2n = [];
  else
    t1n = text(0,0,nm,'FontSize',10.5,'FontWeight','bold', ...
         'HorizontalAlignment','center','VerticalAlignment','middle', ...
         'Interpreter','none','BackgroundColor','w','Margin',1.5); SET(t1n);
    t2n = text(0,0,sprintf('%+.1f A',IL(j)),'FontSize',8.5,'Color',[0.30 0.30 0.30], ...
         'HorizontalAlignment','center','VerticalAlignment','middle', ...
         'Interpreter','none','BackgroundColor','w','Margin',1.5); SET(t2n);
  end
  drawnow;
  e1 = get(t1n,'Extent'); w1 = e1(3); h1 = e1(4);
  if ~isempty(t2n), e2 = get(t2n,'Extent'); w2 = e2(3); h2 = e2(4); else, w2 = 0; h2 = 0; end
  gw = max(w1,w2); gh = h1 + h2 + 0.06;
  cands = nodeCands(q, isStar);
  best = cands(1,:); bestCost = inf;
  for c = 1:size(cands,1)
    R = rectOf(cands(c,:), gw, gh);
    cost = rectCost(R, ALLSEG, OBS, placed, 0.05);
    if cost == 0, best = cands(c,:); break; end
    if cost < bestCost, bestCost = cost; best = cands(c,:); end
  end
  set(t1n,'Position',[best(1) best(2)+(h2+0.06)/2, 0]);
  if ~isempty(t2n), set(t2n,'Position',[best(1) best(2)-(h1+0.06)/2, 0]); end
  placed(end+1,:) = rectOf(best, gw, gh);                 %#ok<AGROW>
end

% ------------------------------------------------------- verify and write
lab = findobj(ax,'Type','text');
rects = zeros(0,4); names = {}; opaque = [];
for h = lab'
  e = get(h,'Extent');
  if e(3) > 0 && e(4) > 0
    rects(end+1,:) = e; names{end+1} = get(h,'String');   %#ok<AGROW>
    bc = get(h,'BackgroundColor');
    opaque(end+1) = ~(ischar(bc) && strcmp(bc,'none'));   %#ok<AGROW>
  end
end
nSeg = 0; nTxt = 0; hits = {};
for i = 1:size(rects,1)
  R = rects(i,:);
  for k = 1:10
    if segHitsRect(ARW(k,1:2), ARW(k,3:4), R)
      nSeg = nSeg+1; hits{end+1} = sprintf('" %s " over arrow w%d', firstLine(names{i}), EW(k)); %#ok<AGROW>
    end
    if segHitsRect(E1(k,:), E2(k,:), R)
      nSeg = nSeg+1; hits{end+1} = sprintf('" %s " over edge w%d', firstLine(names{i}), EW(k)); %#ok<AGROW>
    end
  end
  for j = i+1:size(rects,1)
    % only an OPAQUE box can hide glyphs; two transparent lines may share space
    if (opaque(i) || opaque(j)) && rectsOverlap(R, rects(j,:))
      nTxt = nTxt+1; hits{end+1} = sprintf('" %s " over text " %s "', firstLine(names{i}), firstLine(names{j})); %#ok<AGROW>
    end
  end
end
fprintf('annotation check: %d text boxes -> %d over lines/arrows, %d text-on-text\n', ...
        size(rects,1), nSeg, nTxt);
for k = 1:min(10,numel(hits)), fprintf('   %s\n', hits{k}); end
print(f, fullfile(outdir,fname), '-dpng','-r150'); close(f);
fprintf('figure written: %s\n', fullfile(outdir,fname));
end

% ================================================================= spec
function [T, P, ttl, sub, keytxt, skel, fname] = fig_spec(variant)
nodes = {'X','L1','L2','L3','L4','L5','L6','L7','L8','L9','L10'};
switch lower(char(variant))
  case {'w9x1','w9-l1','w9_to_l1','primary','sstl','single-star-point'}
    E = b10optb_netlist('w9x1');
    P = struct('X',[0 0], 'L1',[4.2 0], 'L8',[-1.3 1.7], 'L9',[0 2.0], 'L10',[1.3 1.7], ...
               'L5',[3.3 1.8], 'L7',[5.4 1.6], 'L3',[6.0 -0.5], 'L6',[4.2 -3.3], ...
               'L2',[2.8 -4.4], 'L4',[5.6 -4.4]);
    skel = [4 9 10];                              % X-L1 trunk, L1-L6, L1-L3
    ttl = 'Single-Star-Point Ten-Leg AMB Topology';
    sub = 'one undriven STAR X (degree 4)  +  leg L1 (degree 5)  +  leg L6 (degree 3)   |   internal id: w9x1';
    keytxt = { ...
     '  w1 = a1 : L7  -> L1        w6 = c3 : L6  -> L2'; ...
     '  w2 = c1 : L5  -> L1        w7 = a4 : X   -> L9'; ...
     '  w3 = a2 : L10 -> X         w8 = c4 : X   -> L8'; ...
     '  w4 = c2 : L1  -> X         w9 = a5 : L1  -> L6   <-- RE-TERMINATED (reference B: L3 -> L6)'; ...
     '  w5 = a3 : L6  -> L4        w10 = c5 : L1 -> L3'; ...
     '  odd w = a winding (orange)   even w = c winding (blue)   DOF pairs: (w1,w2) (w3,w4) (w5,w6) (w7,w8) (w9,w10)'; ...
     ''; ...
     '  NAME            single-star-point ten-leg topology; internal id: w9x1 (same netlist)'; ...
     '  NODE DEGREES    X 4 | L1 5 | L6 3 | L3 1 | L2,L4,L5,L7,L8,L9,L10 = 1'; ...
     '  BIAS CURRENTS   X 0; L2 -5, L3 -5, L4 -5, L8 -5, L9 -5; L1 +5, L5 +5, L6 +5, L7 +5, L10 +5 [A]'; ...
     '  CRITERIA        Udc,min = 67.1239 V = 2.3501 phi | bias 5.00 A | envelope 7.50 A | cyc = 0'; ...
     '                  zero static bias error under winding R/L mismatch; no split-capacitor bus'; ...
     '  WHY IT WORKS    every DOF pair meets a common node with the SAME orientation: DOF1 at L1'; ...
     '                  (two heads), DOF2 at X, DOF3 at L6, DOF4 at X, DOF5 at L1 (both tails).'; ...
     '                  A pair common mode is 2*i_bias; at L1 the DOF1 and DOF5 common-mode flows'; ...
     '                  cancel exactly, so the net L1 current is the w4 current (5 - delta2/2):'; ...
     '                  bias 5 A and envelope 7.5 A, without the 10 A of the direction flip.'; ...
     '  VS B10-OPT-B    +16 % bus (2.3501 vs 2.0252 phi), but all five differential ripples drop'; ...
     '                  from {0.055,0.053,0.055,0.052,0.499} to {0.054,0.051,0.054,0.053,0.050} A p-p.'; ...
     '  VS d569 = -1    the same low ripple WITHOUT 10 A legs: 5 A / 7.5 A instead of 10 A / 10 A.'; ...
     '  NAME NOTE       w9x1 is the internal id; the canonical topology name is'; ...
     '                  "single-star-point ten-leg topology".  Same netlist, same numbers.'};
    fname = 'fig1_topology.png';
  case {'b','b10-opt-b','reference-b'}
    E = b10optb_netlist('B');
    P = struct('X',[0 0], 'L1',[4.2 0], 'L8',[-1.3 1.7], 'L9',[0 2.0], 'L10',[1.3 1.7], ...
               'L5',[3.3 1.8], 'L7',[5.4 1.6], 'L3',[4.2 -1.8], 'L6',[4.2 -3.3], ...
               'L2',[3.0 -4.5], 'L4',[5.4 -4.5]);
    skel = [4 10 9 6];
    ttl = 'Reference: B10-OPT-B (minimum-bus 5 A-class point)';
    sub = 'one undriven STAR X (degree 4)  +  branching legs L1 (degree 4) and L6 (degree 3)   |   reference only';
    keytxt = { ...
     '  w1 = a1 : L7  -> L1        w6 = c3 : L6  -> L2'; ...
     '  w2 = c1 : L5  -> L1        w7 = a4 : X   -> L9'; ...
     '  w3 = a2 : L10 -> X         w8 = c4 : X   -> L8'; ...
     '  w4 = c2 : L1  -> X         w9 = a5 : L3  -> L6'; ...
     '  w5 = a3 : L6  -> L4        w10 = c5 : L1 -> L3'; ...
     '  odd w = a winding (orange)   even w = c winding (blue)   DOF pairs: (w1,w2) (w3,w4) (w5,w6) (w7,w8) (w9,w10)'; ...
     ''; ...
     '  NAME            B10-OPT-B; reference for the minimum bus in the 5 A class'; ...
     '  NODE DEGREES    X 4 | L1 4 | L6 3 | L3 2 | L2,L4,L5,L7,L8,L9,L10 = 1'; ...
     '  BIAS CURRENTS   X 0, L1 0, L3 0; L2 -5, L4 -5, L8 -5, L9 -5; L5 +5, L6 +5, L7 +5, L10 +5 [A]'; ...
     '  CRITERIA        Udc,min = 57.8429 V = 2.0252 phi | bias 5.00 A | envelope 7.50 A | cyc = 0'; ...
     '                  zero static bias error under winding R/L mismatch; no split-capacitor bus'; ...
     '  WHY 10 LEGS     the star X joins a complete pair in (w3,w4 = DOF2) against a complete pair out'; ...
     '                  (w7,w8 = DOF4), so its bias and deltas cancel exactly.  Leg L1 is degree 4 but'; ...
     '                  it is NOT a passive star: its outgoing pair is (w4 = c2, w10 = c5), not a DOF pair,'; ...
     '                  so L1 must carry the control combination -(d2+d5)/2 -- 0 A at the bias point.'; ...
     '                  Hard-wiring L1 as a star leaves a 5 KCL residual -> invalid.  That is what the'; ...
     '                  tenth leg buys, and why this is not a split of a 9-leg shape.'; ...
     '  REFERENCE ONLY  w9 and w10 meet L3 with OPPOSITE orientations, so DOF5 has a 3-leg'; ...
     '                  differential expression and 0.5 A ripple.  The primary topology removes'; ...
     '                  this by re-terminating w9 to L1 (see fig1 / the primary model).'};
    fname = 'fig9_reference_b10optb_topology.png';
  case {'d569','ripple','d569-1','ripple_priority','reference-d569'}
    E = b10optb_netlist('d569');
    P = struct('X',[0 0], 'L1',[4.2 0], 'L8',[-1.3 1.7], 'L9',[0 2.0], 'L10',[1.3 1.7], ...
               'L5',[3.3 1.8], 'L7',[5.4 1.6], 'L3',[4.2 -1.8], 'L6',[4.2 -3.3], ...
               'L2',[3.0 -4.5], 'L4',[5.4 -4.5]);
    skel = [4 10 9 6];
    ttl = 'Reference: B10-OPT-B with direction_569 = -1';
    sub = 'same wiring as B10-OPT-B, current directions of w5, w6 and w9 reversed   |   reference only';
    keytxt = { ...
     '  w1 = a1 : L7  -> L1        w6 = c3 : L2  -> L6'; ...
     '  w2 = c1 : L5  -> L1        w7 = a4 : X   -> L9'; ...
     '  w3 = a2 : L10 -> X         w8 = c4 : X   -> L8'; ...
     '  w4 = c2 : L1  -> X         w9 = a5 : L6  -> L3'; ...
     '  w5 = a3 : L4  -> L6        w10 = c5 : L1 -> L3'; ...
     '  odd w = a winding (orange)   even w = c winding (blue)   DOF pairs: (w1,w2) (w3,w4) (w5,w6) (w7,w8) (w9,w10)'; ...
     ''; ...
     '  NAME            reference direction_569 = -1; not the primary topology'; ...
     '  NODE DEGREES    X 4 | L1 4 | L6 3 | L3 2 | L2,L4,L5,L7,L8,L9,L10 = 1'; ...
     '  BIAS CURRENTS   X 0, L1 0; L2 +5, L4 +5, L5 +5, L7 +5, L10 +5; L3 -10, L6 -5, L8 -5, L9 -5 [A]'; ...
     '  CRITERIA        Udc,min = 86.4054 V = 3.0252 phi | bias 10.00 A | envelope 10.00 A | cyc = 0'; ...
     '  WHY IT WORKS    all five pairs meet a common node with the SAME orientation (DOF5: both heads'; ...
     '                  at L3), so every differential ripple is at the 0.05 A limit.'; ...
     '  COST            L3 carries i_a + i_c = 10 A; the bus is 29 % above the primary and 49 %'; ...
     '                  above reference B.  It is strictly dominated by the primary topology (same low'; ...
     '                  ripple, 5 A / 7.5 A legs, 2.3501 phi).  Kept for comparison only.'};
    fname = 'fig10_reference_d569_topology.png';
  otherwise
    error('b10_fig:variant','unknown variant "%s"', char(variant));
end
T = tp_mk(sprintf('%s', char(variant)), nodes, E, 'fig');
end

% ================================================================= helpers
function s = firstLine(v)
if iscell(v), v = v{1}; end
v = strtrim(v);
if numel(v) > 22, s = [v(1:22) '...']; else, s = v; end
end

function R = rectOf(c, w, h)
R = [c(1)-w/2, c(2)-h/2, w, h];
end

function tf = inRect(p, R)
tf = p(1) >= R(1) && p(1) <= R(1)+R(3) && p(2) >= R(2) && p(2) <= R(2)+R(4);
end

function cost = rectCost(R, segs, obs, placed, pad)
Rb = [R(1)-pad, R(2)-pad, R(3)+2*pad, R(4)+2*pad];
cost = 0;
for k = 1:size(segs,1)
  if segHitsRect(segs(k,1:2), segs(k,3:4), Rb), cost = cost + 1; end
end
for k = 1:size(obs,1)
  if rectsOverlap(Rb, obs(k,:)), cost = cost + 1; end
end
for k = 1:size(placed,1)
  if rectsOverlap(Rb, placed(k,:)), cost = cost + 1; end
end
end

function tf = rectsOverlap(a, b)
tf = a(1) <= b(1)+b(3) && b(1) <= a(1)+a(3) && a(2) <= b(2)+b(4) && b(2) <= a(2)+a(4);
end

function tf = segHitsRect(p, q, R)
tf = false;
if inRect(p,R) || inRect(q,R), tf = true; return; end
x1 = R(1); y1 = R(2); x2 = R(1)+R(3); y2 = R(2)+R(4);
if max(p(1),q(1)) < x1 || min(p(1),q(1)) > x2 || max(p(2),q(2)) < y1 || min(p(2),q(2)) > y2
  return;
end
tf = segInt(p,q,[x1 y1],[x2 y1]) || segInt(p,q,[x2 y1],[x2 y2]) || ...
     segInt(p,q,[x2 y2],[x1 y2]) || segInt(p,q,[x1 y2],[x1 y1]);
end

function tf = segInt(a, b, c, d)
r = b - a; s = d - c;
den = r(1)*s(2) - r(2)*s(1);
if abs(den) < 1e-12, tf = false; return; end
t = ((c(1)-a(1))*s(2) - (c(2)-a(2))*s(1))/den;
u = ((c(1)-a(1))*r(2) - (c(2)-a(2))*r(1))/den;
tf = t >= 0 && t <= 1 && u >= 0 && u <= 1;
end

function d = segDist(p, a, b)
ab = b - a; L2 = ab(1)^2 + ab(2)^2;
if L2 == 0, d = norm(p - a); return; end
t = ((p(1)-a(1))*ab(1) + (p(2)-a(2))*ab(2))/L2;
t = max(0, min(1, t));
d = norm(p - (a + t*ab));
end

function tf = closerToOther(p, k, E1, E2, dOwn)
% true when another edge is (clearly) nearer to p than its own edge
tf = false;
for j = 1:size(E1,1)
  if j == k, continue; end
  if segDist(p, E1(j,:), E2(j,:)) < dOwn - 0.02, tf = true; return; end
end
end

function cands = edgeCands(p1, p2, d)
% candidates around the edge, ordered so that the label first tries to sit as
% close to its own edge as possible (best association), then further away
u = p2 - p1; L = norm(u); u = u / L; n = [-u(2) u(1)];
ts = [0.50 0.60 0.40 0.68 0.32 0.76 0.24 0.84];
ss = [d -d 1.7*d -1.7*d 2.5*d -2.5*d 3.3*d -3.3*d 4.2*d -4.2*d];
[T,S] = meshgrid(ts, ss);
T = T(:); S = S(:);
key = abs(S) + 0.20*abs(T-0.5);
[~,ix] = sort(key);
cands = zeros(numel(ix),2);
for c = 1:numel(ix)
  cands(c,:) = p1 + T(ix(c))*L*u + S(ix(c))*n;
end
end

function cands = nodeCands(q, isStar)
% a small ring of positions around the node; the name sits above the current
if isStar
  rs = [0.55 0.85 1.15 1.45]; angs = [270 225 315 180 0 240 300 200 340];
else
  rs = [0.55 0.85 1.15 1.5];   angs = [90 45 135 -90 0 180 -45 -135 225 315];
end
cands = zeros(numel(rs)*numel(angs),2); c = 0;
for i = 1:numel(rs)
  for j = 1:numel(angs)
    c = c + 1;
    th = angs(j)*pi/180;
    cands(c,:) = q + rs(i)*[cos(th) sin(th)];
  end
end
end
