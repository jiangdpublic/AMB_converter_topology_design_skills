clearvars;

% RL load parameters
R_value = 1;
L_value = 500e-6;




% DC bus voltage
U_dc = 200;


% current loop parameters
f_c = 1500;
K_p = 2*pi*f_c*L_value/U_dc;
K_i = 2*pi*f_c*R_value/U_dc;


% switching parameters
SW_frequency = 20e3;
SW_Ts = 1/(SW_frequency);
half_SW_Ts = SW_Ts*0.5;


% AMB specified current parameters
bias_current = 5;



