function R = b10optb_nets(mdl)
%B10OPTB_NETS  Nets of a Simulink/Simscape model, from real connectivity.
%   R = b10optb_nets(mdl) returns
%     R.line, R.seg, R.segNet, R.segSys, R.segSysName,
%     R.netOfLine, R.netOfPort (map: port handle -> net id), R.nNet
%
%   A net is grown three ways:
%     1. lines attached to the same block port (get_param(port,'Line')),
%     2. a subsystem connection port (PMIOPort) and the matching port of its
%        parent block (PMIOPort parameters Port and Side),
%     3. lines that touch geometrically inside the same canvas - Simulink
%        splits a polyline where a branch leaves it, and such a piece reports
%        SrcPortHandle = -1.
%   Physical connection lines report no source port at all, so (1)/(2) alone
%   are not sufficient.
%   Coordinates are grouped by canvas: segments of different subsystems are
%   never compared with each other.
TOL = 1.0;
lh  = find_system(mdl,'FindAll','on','Type','line');
n   = numel(lh);
lineIdx = containers.Map('KeyType','double','ValueType','double');
for i = 1:n, lineIdx(double(lh(i))) = i; end

lineSys = cell(n,1); seg = zeros(0,4); segLine = zeros(0,1);
P1 = zeros(n,2); P2 = zeros(n,2);
for i = 1:n
  lineSys{i} = get_param(lh(i),'Parent');
  p = get_param(lh(i),'Points');
  if isempty(p)
    P1(i,:) = [NaN NaN]; P2(i,:) = [NaN NaN]; continue;
  end
  P1(i,:) = p(1,:); P2(i,:) = p(end,:);
  for j = 1:size(p,1)-1
    seg(end+1,:) = [p(j,1) p(j,2) p(j+1,1) p(j+1,2)];   %#ok<AGROW>
    segLine(end+1,1) = i;                                 %#ok<AGROW>
  end
end
[sysU,~,sysIdx] = unique(lineSys);
segSys = sysIdx(segLine);
parent = 1:n;

% ---- 1. lines meeting at the same block port ----
blks = find_system(mdl,'LookUnderMasks','all','FollowLinks','on','Type','Block');
portIdx = containers.Map('KeyType','double','ValueType','double');
pu = zeros(0,1); portLines = {};
for b = 1:numel(blks)
  ph = get_param(blks{b},'PortHandles');
  fn = fieldnames(ph);
  for q = 1:numel(fn)
    hs = ph.(fn{q});
    for j = 1:numel(hs)
      ls = get_param(hs(j),'Line');
      ls = ls(ls > 0);
      li = [];
      for t = 1:numel(ls)
        if isKey(lineIdx, double(ls(t))), li(end+1) = lineIdx(double(ls(t))); end %#ok<AGROW>
      end
      [portIdx, pu, id] = pid_(portIdx, pu, hs(j));
      if numel(portLines) < id, portLines{id} = []; end
      if ~isempty(li), portLines{id} = unique([portLines{id} li]); end
    end
  end
end
% ---- 2. subsystem connection port <-> internal PMIOPort ----
subs = find_system(mdl,'LookUnderMasks','all','FollowLinks','on','BlockType','SubSystem');
for b = 1:numel(subs)
  pmi = find_system(subs{b},'LookUnderMasks','all','SearchDepth',1,'BlockType','PMIOPort');
  if isempty(pmi), continue; end
  ph = get_param(subs{b},'PortHandles');
  for k = 1:numel(pmi)
    side = get_param(pmi{k},'Side');
    idx  = str2double(get_param(pmi{k},'Port'));
    if strcmpi(side,'Left') && numel(ph.LConn) >= idx
      hPar = ph.LConn(idx);
    elseif strcmpi(side,'Right') && numel(ph.RConn) >= idx
      hPar = ph.RConn(idx);
    else
      continue;
    end
    ih = get_param(pmi{k},'PortHandles');
    cand = [ih.LConn ih.RConn];
    if isempty(cand), continue; end
    [portIdx, pu, idA] = pid_(portIdx, pu, hPar);
    [portIdx, pu, idB] = pid_(portIdx, pu, cand(1));
    pu = punite_(pu, idA, idB);
  end
end
groups = containers.Map('KeyType','double','ValueType','any');
for id = 1:numel(portLines)
  if isempty(portLines{id}), continue; end
  c = pfind_(pu, id);
  if isKey(groups, c), groups(c) = [groups(c) portLines{id}];
  else,                groups(c) = portLines{id}; end
end
gk = groups.keys;
for t = 1:numel(gk)
  ls = unique(groups(gk{t}));
  for u = 2:numel(ls), parent = unite_(parent, ls(1), ls(u)); end
end
% ---- 3. lines that touch geometrically inside one canvas ----
for c = 1:numel(sysU)
  li = find(sysIdx == c);
  if numel(li) < 2, continue; end
  PT = [P1(li,:); P2(li,:)];
  own = [li(:); li(:)];
  ok = ~any(isnan(PT),2); PT = PT(ok,:); own = own(ok);
  D = (PT(:,1)-PT(:,1)').^2 + (PT(:,2)-PT(:,2)').^2;
  [r,cc] = find(triu(D <= TOL^2, 1));
  for k = 1:numel(r), parent = unite_(parent, own(r(k)), own(cc(k))); end
  sm = (segSys == c);
  S = seg(sm,:); SL = segLine(sm);
  for k = 1:size(S,1)
    ax = S(k,1); ay = S(k,2); bx = S(k,3); by = S(k,4);
    dx = bx-ax; dy = by-ay; L2 = dx*dx + dy*dy;
    if L2 == 0, continue; end
    tt = ((PT(:,1)-ax)*dx + (PT(:,2)-ay)*dy)/L2;
    tt = max(0,min(1,tt));
    d2 = (PT(:,1)-(ax+tt*dx)).^2 + (PT(:,2)-(ay+tt*dy)).^2;
    hit = find(d2 <= TOL^2);
    for u = 1:numel(hit)
      if own(hit(u)) ~= SL(k), parent = unite_(parent, own(hit(u)), SL(k)); end
    end
  end
end
root = zeros(n,1);
for i = 1:n, root(i) = find_(parent, i); end
[~,~,root] = unique(root);

netOfPort = containers.Map('KeyType','double','ValueType','double');
pk = portIdx.keys;
for t = 1:numel(pk)
  id = portIdx(pk{t});
  if id > numel(portLines) || isempty(portLines{id}), continue; end
  netOfPort(pk{t}) = root(portLines{id}(1));
end
R = struct();                     % assign fields one by one: struct() would
R.line       = lh;                % expand the cell arrays into a struct array
R.seg        = seg;
R.segNet     = root(segLine);
R.segSys     = segSys;
R.segSysName = sysU(segSys);
R.netOfLine  = root;
R.netOfPort  = netOfPort;
R.sysOfLine  = lineSys;
R.nNet       = max(root);
end

function [portIdx, pu, id] = pid_(portIdx, pu, h)
h = double(h);
if isKey(portIdx, h)
  id = portIdx(h);
else
  pu(end+1,1) = numel(pu)+1;
  id = numel(pu);
  portIdx(h) = id;
end
end

function p = pfind_(pu, i)
while pu(i) ~= i, i = pu(i); end
p = i;
end

function pu = punite_(pu, a, b)
ra = pfind_(pu,a); rb = pfind_(pu,b);
if ra ~= rb, pu(rb) = ra; end
end

function p = find_(parent, i)
while parent(i) ~= i
  parent(i) = parent(parent(i));
  i = parent(i);
end
p = i;
end

function parent = unite_(parent, a, b)
ra = find_(parent,a); rb = find_(parent,b);
if ra ~= rb, parent(rb) = ra; end
end
