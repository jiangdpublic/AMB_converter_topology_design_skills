function d = b10optb_outdir(kind, caller)
%B10OPTB_OUTDIR  Where this script family writes its outputs.
%   d = b10optb_outdir('model')            % uses this file's own location
%   d = b10optb_outdir('model', mfilename('fullpath'))
%   Pass the CALLER's path (as shown) so the answer stays correct even when a
%   second copy of these scripts (e.g. the development workspace) is also on
%   the MATLAB path.  Layouts supported:
%     <skill>/scripts/...  ->  <skill>/<kind>          (model/, assets/)
%     <work>/ ...          ->  <work>/<kind> if it exists, else the script folder
if nargin < 2 || isempty(caller), caller = mfilename('fullpath'); end
here = fileparts(caller);
skr  = fileparts(here);
d = fullfile(skr,kind);
if ~exist(d,'dir'), d = fullfile(here,kind); end
if ~exist(d,'dir'), d = here; end
end
