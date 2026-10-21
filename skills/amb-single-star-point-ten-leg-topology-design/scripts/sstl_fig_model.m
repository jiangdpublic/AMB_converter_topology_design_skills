function sstl_fig_model()
%SSTL_FIG_MODEL  Export the model-overview and subsystem screenshots used by
%   the skill documentation.  The model must be the packed primary build.
mdl = 'SINGLE_STAR_TEN_LEG_AMB';
if ~bdIsLoaded(mdl), b10optb_load(); end
out = b10optb_outdir('assets', mfilename('fullpath'));
shots = { mdl,                    'fig2_model_layout.png'; ...
          [mdl '/Ctrl'],          'fig3_controller.png'; ...
          [mdl '/Leg1'],          'fig5_leg_internals.png'; ...
          [mdl '/W1'],            'fig6_winding_internals.png'; ...
          [mdl '/Ctrl/aw1'],      'fig7_antiwindup.png' };
for k = 1:size(shots,1)
  f = fullfile(out, shots{k,2});
  print(['-s' shots{k,1}], '-dpng', '-r150', f);
  fprintf('wrote %s\n', f);
end
end
