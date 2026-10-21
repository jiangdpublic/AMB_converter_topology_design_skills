function isLeg = b9_isleg(T)
%B9_ISLEG  Logical mask of the DRIVEN nodes: every node NOT named X* is a leg.
isLeg = true(1, numel(T.nodes));
for k = 1:numel(T.nodes)
  if ~isempty(T.nodes{k}) && (T.nodes{k}(1) == 'X'), isLeg(k) = false; end
end
end