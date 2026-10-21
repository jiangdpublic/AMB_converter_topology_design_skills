function n = b10optb_line_overlap(mdl, verbose)
%B10OPTB_LINE_OVERLAP  Count collinear overlapping line segments that belong to
%   DIFFERENT nets.  Crossings are allowed, and two wires of the same net (for
%   instance the two halves of a bus, or a trunk with taps) may coincide: they
%   are one node and are drawn as a single line, which keeps the picture
%   readable.  This is exactly the user requirement "wires that are not
%   connected together must not lie on top of each other".
%   Segments are only compared inside the same system (canvas), because the
%   coordinates of different subsystems are unrelated.
%
%   n = b10optb_line_overlap(mdl)         % print the first offenders
%   n = b10optb_line_overlap(mdl, false)  % silent count
if nargin < 2, verbose = true; end
if ~bdIsLoaded(mdl), b10optb_load(); end
R = b10optb_nets(mdl);
seg = R.seg; net = R.segNet; sys = R.segSys;
n = 0; bad = {};
for orient = 1:2
  if orient == 1                      % vertical segments
    m = seg(:,1) == seg(:,3);
    c = seg(:,1); lo = min(seg(:,2),seg(:,4)); hi = max(seg(:,2),seg(:,4));
  else                                % horizontal segments
    m = seg(:,2) == seg(:,4);
    c = seg(:,2); lo = min(seg(:,1),seg(:,3)); hi = max(seg(:,1),seg(:,3));
  end
  idx = find(m);
  key = sys(idx)*1e6 + c(idx);
  [~,~,g] = unique(key);
  for q = 1:max(g)
    ii = idx(g == q);
    if numel(ii) < 2, continue; end
    LO = max(lo(ii), lo(ii)');
    HI = min(hi(ii), hi(ii)');
    ov = HI - LO;
    mask = triu(true(numel(ii)),1) & (ov > 2) & (net(ii) ~= net(ii)');
    [r,cc] = find(mask);
    for k = 1:numel(r)
      n = n + 1;
      if numel(bad) < 12
        a = ii(r(k)); b = ii(cc(k));
        if orient == 1, s1 = 'V'; s2 = 'x'; s3 = 'y'; else, s1 = 'H'; s2 = 'y'; s3 = 'x'; end
        bad{end+1} = sprintf('%s %s=%d %s=%d..%d (nets %d/%d) in %s', s1, s2, c(a), s3, ...
          round(LO(r(k),cc(k))), round(HI(r(k),cc(k))), net(a), net(b), R.segSysName{a}); %#ok<AGROW>
      end
    end
  end
end
if verbose
  fprintf('overlap check: %d segments in %d nets -> %d overlapping different-net pairs\n', ...
          size(seg,1), R.nNet, n);
  for k = 1:numel(bad), fprintf('   %s\n', bad{k}); end
end
end
