function T = tp_mk(name, nodes, E, note)
%TP_MK  Descriptor constructor.
T = struct('name',name,'nodes',{nodes},'E',{E},'note',note);
end
