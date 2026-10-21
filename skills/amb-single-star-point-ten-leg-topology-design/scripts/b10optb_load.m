function mdl = b10optb_load()
%B10OPTB_LOAD  Make sure the bundled SINGLE_STAR_TEN_LEG_AMB model is loaded.
%   Looks for <skill>/model/SINGLE_STAR_TEN_LEG_AMB.slx first, then falls back to the
%   MATLAB path, so the scripts work after the skill folder is moved anywhere.
mdl = 'SINGLE_STAR_TEN_LEG_AMB';
if bdIsLoaded(mdl), return; end
here = fileparts(mfilename('fullpath'));
skr  = fileparts(here);
mf   = fullfile(skr,'model',[mdl '.slx']);
if ~exist(mf,'file'), mf = fullfile(here,[mdl '.slx']); end
if exist(mf,'file'), load_system(mf); else, load_system(mdl); end
end
