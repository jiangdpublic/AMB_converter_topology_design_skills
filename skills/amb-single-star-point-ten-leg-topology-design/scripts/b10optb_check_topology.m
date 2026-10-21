function ok = b10optb_check_topology(mdl, variant, verbose)
%B10OPTB_CHECK_TOPOLOGY  Verify the built model against the winding netlist.
%   Reads back the connectivity of the assembled model and checks that
%     - every leg midpoint M1..M10 and the star node X is one single net,
%     - those 11 nets are pairwise different,
%     - winding wk really connects tail(wk) to head(wk) of the netlist.
%   ok = b10optb_check_topology('SINGLE_STAR_TEN_LEG_AMB')
%   The default netlist is 'w9x1', the internal id of the primary
%   single-star-point ten-leg topology (same netlist).
if nargin < 2 || isempty(variant), variant = 'w9x1'; end
if nargin < 3, verbose = true; end
if ~bdIsLoaded(mdl), b10optb_load(); end
E = b10optb_netlist(variant);
R = b10optb_nets(mdl);
nNet = zeros(11,1); nm = cell(11,1);
for k = 1:10
  nm{k} = sprintf('L%d',k);
  nNet(k) = netof_(R, portof_(mdl, sprintf('Leg%d',k), 'LConn', 2));
end
nm{11} = 'X';
kx = find(strcmp(E(:,3),'X'), 1);          % winding subsystem W<kx> has its head on X
if isempty(kx), kx = find(strcmp(E(:,2),'X'), 1); end
nNet(11) = netof_(R, portof_(mdl, sprintf('W%d',kx), 'LConn', 2));
ok = true;
if numel(unique(nNet)) ~= 11
  ok = false;
  if verbose, fprintf('  !! the 11 nodes are not 11 distinct nets: %s\n', mat2str(nNet')); end
end
for k = 1:10
  it = find(strcmp(nm, E{k,2})); ih = find(strcmp(nm, E{k,3}));
  t = netof_(R, portof_(mdl, sprintf('W%d',k), 'LConn', 1));
  h = netof_(R, portof_(mdl, sprintf('W%d',k), 'LConn', 2));
  good = (t == nNet(it)) && (h == nNet(ih)) && (t ~= h);
  ok = ok && good;
  if verbose
    if good, st = 'ok'; else, st = 'MISMATCH'; end
    fprintf('  %-4s %-4s -> %-4s : %s\n', E{k,1}, E{k,2}, E{k,3}, st);
  end
end
if verbose, fprintf('topology check: %d\n', ok); end
end

function h = portof_(mdl, blk, grp, idx)
ph = get_param([mdl '/' blk],'PortHandles');
h = ph.(grp)(idx);
end

function n = netof_(R, h)
m = R.netOfPort;                    % pull the map out first (struct field of a map)
if isKey(m, double(h)), n = m(double(h)); else, n = NaN; end
end
