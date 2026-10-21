function out = b10_is_split(E, nodes)
%B10_IS_SPLIT  True when the 10-leg tree is just the 9-leg shape with ONE of its
%   two star nodes driven (i.e. not essentially different).
%   Signature of the 9-leg shape: degree sequence {4,4,2,2,2,1,1,1,1,1,1}, the two
%   degree-4 nodes joined by a 3-edge path whose internal nodes have degree 2, and
%   each degree-4 node carrying exactly 3 leaf neighbours.
N = numel(nodes); deg = zeros(1,N); adj = zeros(N);
for k = 1:size(E,1)
  a = find(strcmp(nodes,E{k,2})); b = find(strcmp(nodes,E{k,3}));
  deg(a)=deg(a)+1; deg(b)=deg(b)+1; adj(a,b)=1; adj(b,a)=1;
end
out = false;
if ~isequal(sort(deg), [1 1 1 1 1 1 2 2 2 4 4]), return; end
hi = find(deg==4);
for i = 1:2
  v = hi(i);
  if sum(deg(adj(v,:)>0)==1) ~= 3, return; end
end
% path between the two degree-4 nodes
a = hi(1); b = hi(2); prev = 0; cur = a; len = 0;
while cur ~= b
  nb = find(adj(cur,:) > 0); nb = nb(nb ~= prev);
  if numel(nb) ~= 1, return; end
  prev = cur; cur = nb; len = len + 1;
  if len > 4, return; end
end
if len ~= 3, return; end
out = true;
end