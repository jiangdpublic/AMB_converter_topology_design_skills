function par = tp_par()
%TP_PAR  Parameters shared by every topology comparison in this study.
%   Identical to the sibling AMB topology studies so the tables are comparable.
par.R = 1.0; par.L = 500e-6; par.i_bias = 5; par.rating = 10;
par.delta_max = 5; par.f_c = 1500; par.f_sw = 20e3;
par.sigma = 2*pi*par.f_c*par.delta_max;      % slew proxy  = 47123.9 A/s
par.phi   = par.R*par.delta_max + par.L*par.sigma;   % 28.5619 V
par.u_max = par.R*par.i_bias + par.phi/2;            % 19.2810 V
par.U_dc  = 30;                              % design bus used for windows
end
