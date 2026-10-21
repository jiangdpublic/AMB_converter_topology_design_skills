function run_case_study(mode, irefArg, udcArg)
% Reproduces and regresses the numbers documented in
%   references/case-ten-phase-nine-leg-amb.md                   (baseline)
%   references/case-ten-phase-nine-leg-modulation-boundary.md   (boundary)
%   references/case-ten-phase-nine-leg-relocating-the-extra-leg.md (relocation)
% from the models bundled in model/.
%
% Relocatable: models and the parameter script are located relative to this
% file, and every simulation runs on an INSTRUMENTED COPY in the system temp
% folder, so the bundled .slx files are never modified.
%
% Verified with MATLAB R2025b (25.2), Simulink 25.2, Simscape 25.2 and
% Simscape Electrical 25.2 (Linux glnxa64).
%
% Usage (from the scripts/ folder of this skill):
%   run_case_study                 baseline, 20 ms                 (about 1 min)
%   run_case_study('point', iref)  one operating point at i_ref     (about 15 s)
%   run_case_study('point', iref, udc)  same, at a given bus voltage
%   run_case_study('boundary')     sweep endpoints, regenerates fig5/fig6 (about 5 min)
%   run_case_study('relocation')   the two relocated variants, fig4  (about 1 min)
%   run_case_study('deadtime')     dead-time overhead evidence       (about 1 min)

if nargin < 1 || isempty(mode), mode = 'baseline'; end
if nargin < 2, irefArg = 1.0; end
if nargin < 3, udcArg = []; end

Here      = fileparts(mfilename('fullpath'));
SkillRoot = fileparts(Here);
ModelDir  = fullfile(SkillRoot,'model');
Assets    = fullfile(SkillRoot,'assets');
P = loadAmbParams(fullfile(ModelDir,'parameters_for_AMB_topology.m'));
tally(NaN,0);
% Work from a scratch folder so that compiler artefacts (.slxc, slprj) never
% land in the skill folder.
work = fullfile(tempdir,'amb_b9p10_work');
if ~exist(work,'dir'), mkdir(work); end
oldDir = cd(work); restoreDir = onCleanup(@() cd(oldDir));   %#ok<NASGU>
fprintf('run_case_study(%s)\n', mode);

switch lower(mode)
    case 'baseline',   caseBaseline(ModelDir, P, Assets)
    case 'point',      casePoint(ModelDir, P, irefArg, udcArg)
    case 'boundary',   caseBoundary(ModelDir, P, Assets)
    case 'relocation', caseRelocation(ModelDir, P, Assets)
    case 'deadtime',   caseDeadtime(ModelDir, P)
    otherwise, error('run_case_study: unknown mode ''%s''', mode);
end
tally(NaN,1);
end

%% ===================== baseline ==========================================
function caseBaseline(ModelDir, P, Assets)
m = instrumentCopy(fullfile(ModelDir,'Ten_phase_nine_leg_with_RL_load.slx'),'amb_b9p10_base',1);
R = runPoint(m, P, ones(1,5), P.U_dc, 0.02, 'keep', false);   % keep the delivered start-up stagger
pair = [1 2;3 4;5 6;7 8;9 10];
ok = @(c) c;
fprintf('\nA. steady state (18-20 ms), i_ref = 1 A, U_dc = 200 V\n');
fprintf('   winding currents [A] : %s\n', mat2str(round(R.imean,3)));
fprintf('   control currents [A] : %s\n', mat2str(round(R.ctrl,5)));
fprintf('   pair sums [A]        : %s\n', mat2str(round(R.sum,4)));
fprintf('   switching ripple [A] : %s\n', mat2str(round(R.ripple,3)));
tally(max(abs(R.ctrl - 1)) < 0.02,            'control currents within 0.02 A of 1 A');
tally(max(abs(R.sum - 2*P.bias_current)) < 0.05, 'pair sums within 0.05 A of 2*i_bias');
tally(max(abs(R.imean - [5.495 4.496 5.494 4.497 5.494 4.497 5.494 4.494 5.493 4.495])) < 0.05, ...
      'winding currents match the documented values');
tally(max(R.ripple) < 0.5 && min(R.ripple) > 0.05, 'winding ripple between 0.05 and 0.5 A pp');

fprintf('\nB. topology identities\n');
kclX1 = max(abs((R.I(:,1)+R.I(:,2)) - (R.I(:,3)+R.I(:,4))));
kclX2 = max(abs((R.I(:,7)+R.I(:,8)) - (R.I(:,9)+R.I(:,10))));
sumLeg = max(abs(R.sumLeg));
tel = max(abs(R.Pleg - R.Pw));
fprintf('   star X1 KCL residual : %.3e A\n', kclX1);
fprintf('   star X2 KCL residual : %.3e A\n', kclX2);
fprintf('   sum of 9 leg currents: %.3e A\n', sumLeg);
fprintf('   Tellegen residual    : %.3e W\n', tel);
fprintf('   leg currents [A]     : %s\n', mat2str(round(mean(R.Ileg(R.ss,:)),3)));
tally(kclX1 < 1e-9 && kclX2 < 1e-9, 'star-node KCL holds to 1e-9 A');
tally(sumLeg < 1e-9,                'the nine leg currents sum to zero');
tally(tel < 1e-6,                   'Tellegen identity P_leg = P_winding');
tally(abs(mean(R.Ileg(R.ss,4)) - 1.0) < 0.1 && abs(mean(R.Ileg(R.ss,5)) + 1.0) < 0.1, ...
      'legs 4 and 5 carry about +/-1 A (control current only)');
tally(abs(mean(R.Ileg(R.ss,1)) - 5.5) < 0.1 && abs(mean(R.Ileg(R.ss,3)) + 5.5) < 0.1, ...
      'legs 1 and 3 carry the bias current');

fprintf('\nC. modulation\n');
fprintf('   spread S (18-20 ms)  : %.4f\n', R.spread);
fprintf('   duty range (whole run) : %.4f .. %.4f\n', R.dallmin, R.dallmax);
fprintf('   duty range (18-20 ms)  : %.4f .. %.4f\n', R.dssmin, R.dssmax);
tally(abs(R.spread - 0.209) < 0.02, 'spread matches the documented 0.209');
tally(R.dallmin >= 0 && R.dallmax <= 1, 'duty never leaves [0 1]');
tally(R.dssmin > 0.25 && R.dssmax < 0.75, 'steady-state duty keeps headroom');

fprintf('\nD. losses\n');
fprintf('   copper loss          : %.1f W\n', R.Pcu);
tally(abs(R.Pcu - 252) < 3, 'copper loss = 252 W +/- 3 W');

% baseline figures
f1 = figure('Visible','off','Position',[60 60 1000 800]);
for d = 1:5
    subplot(5,1,d); hold on; grid on;
    plot(R.t*1e3, R.I(:,pair(d,1)),'b','LineWidth',0.8);
    plot(R.t*1e3, R.I(:,pair(d,2)),'r','LineWidth',0.8);
    ylabel(sprintf('DOF%d [A]',d)); ylim([-0.5 6.5]);
    if d == 1, title('winding currents (blue = a, red = c)'); end
end
xlabel('time [ms]');
print(f1,'-dpng','-r130',fullfile(Assets,'fig2_winding_currents.png'));
f2 = figure('Visible','off','Position',[60 60 900 600]);
for d = 1:5
    subplot(5,1,d); hold on; grid on;
    plot(R.t*1e3, R.I(:,pair(d,1))-R.I(:,pair(d,2)),'k','LineWidth',0.8);
    yline(1,'g--'); ylabel(sprintf('\\Delta i_%d [A]',d)); ylim([-0.8 1.6]);
    if d == 1, title('control (differential) currents vs the 1 A reference'); end
end
xlabel('time [ms]');
print(f2,'-dpng','-r130',fullfile(Assets,'fig3_control_currents.png'));
f3 = figure('Visible','off','Position',[60 60 1000 700]);
subplot(2,1,1); hold on; grid on;
plot(R.t*1e3, R.Ileg,'LineWidth',1.0);
ylabel('leg current [A]'); title('nine leg currents (from winding currents + KCL)');
legend(arrayfun(@(k) sprintf('L%d',k),1:9,'uni',0),'Location','eastoutside');
subplot(2,1,2); hold on; grid on;
plot(R.t*1e3, R.sumLeg,'k','LineWidth',1.2);
ylabel('\Sigma I_{leg} [A]'); title('sum of the nine leg currents (identically zero)');
xlabel('time [ms]');
print(f3,'-dpng','-r130',fullfile(Assets,'fig4_leg_currents.png'));
fprintf('   figures written: fig2_winding_currents.png, fig3_control_currents.png, fig4_leg_currents.png\n');
end

%% ===================== single point ======================================
function casePoint(ModelDir, P, iref, udc)
if isempty(udc), udc = P.U_dc; end
m = instrumentCopy(fullfile(ModelDir,'Ten_phase_nine_leg_with_RL_load.slx'),'amb_b9p10_point',1);
R = runPoint(m, P, iref*ones(1,5), udc, 0.006, 'keep', true);
fprintf('  i_ref=%g A  U_dc=%g V : ctrl=%s  spread=%.4f  duty=[%.3f %.3f]  maxerr=%.4f\n', ...
        iref, udc, mat2str(round(R.ctrl,4)), R.spread, R.dssmin, R.dssmax, max(abs(R.ctrl-iref)));
tally(max(abs(R.ctrl - iref)) < 0.05 || R.spread > 1, 'operating point either tracks or is documented as saturated');
end

%% ===================== boundary sweeps ====================================
function caseBoundary(ModelDir, P, Assets)
m = instrumentCopy(fullfile(ModelDir,'Ten_phase_nine_leg_with_RL_load.slx'),'amb_b9p10_bnd',1);
fprintf('\nA. spread vs control current at U_dc = 200 V (all five DOFs equal)\n');
Dv = [0 2 4 6 8 10]; Sv = zeros(size(Dv)); Ev = zeros(size(Dv));
for k = 1:numel(Dv)
    R = runPoint(m, P, Dv(k)*ones(1,5), 200, 0.006, 'keep', true);
    Sv(k) = R.spread; Ev(k) = max(abs(R.ctrl - Dv(k)));
    fprintf('   i_ref=%4.1f A : S=%.4f  duty=[%.3f %.3f]  maxerr=%.4f\n', Dv(k), Sv(k), R.dssmin, R.dssmax, Ev(k));
end
pf = polyfit(Dv, Sv, 1);
fprintf('   fitted dS/di_ref = %.5f /A (theory R/U_dc = %.5f)\n', pf(1), P.R_value/200);
fprintf('   constant overhead = %.4f pu = %.1f V\n', pf(2)-P.R_value*6*P.bias_current/200, ...
        (pf(2)-P.R_value*6*P.bias_current/200)*200);
tally(abs(pf(1) - P.R_value/200) < 0.0008, 'slope of S vs control current = R/U_dc');
tally(abs((pf(2)-0.15) - 0.0534) < 0.012,  'dead-time + device-drop overhead = 0.053 pu');
tally(max(abs(Sv(2:end) - (0.15 + 0.005*Dv(2:end)) - 0.053)) < 0.02, 'spread model S = R*(6ib+d)/Udc + sigma0');
tally(max(Ev) < 0.15, 'all five control currents track at U_dc = 200 V');

fprintf('\nB. bus voltage sweep, symmetric i_ref = +10 A (= 2*i_bias)\n');
Us = [200 100 60 50 45 40]; Ss = zeros(size(Us));
for k = 1:numel(Us)
    R = runPoint(m, P, 10*ones(1,5), Us(k), 0.006, 'keep', true);
    Ss(k) = R.spread;
    fprintf('   U_dc=%5.0f V : S=%.4f  duty=[%7.3f %7.3f]  maxerr=%.4f\n', Us(k), Ss(k), R.dssmin, R.dssmax, max(abs(R.ctrl-10)));
end
tally(Ss(4) < 0.95, 'full range reachable at 50 V');
tally(Ss(5) > 0.90, 'near the boundary at 45 V');
tally(Ss(6) > 1.5,  'deep saturation at 40 V');
bnd = min(Us(Ss < 1));
tally(bnd > 42 && bnd < 48, 'symmetric boundary between 42 V and 48 V');

fprintf('\nC. near-worst-case distribution delta = [9.5 -9.5 5 9.5 9.5]\n');
Un = [200 70 60 55 50]; Sn = zeros(size(Un));
for k = 1:numel(Un)
    R = runPoint(m, P, [9.5 -9.5 5 9.5 9.5], Un(k), 0.006, 'keep', true);
    Sn(k) = R.spread;
    fprintf('   U_dc=%5.0f V : S=%.4f  duty=[%7.3f %7.3f]  maxerr=%.4f\n', Un(k), Sn(k), R.dssmin, R.dssmax, ...
            max(abs(R.ctrl - [9.5 -9.5 5 9.5 9.5])));
end
tally(Sn(3) < 0.95, 'worst-case distribution reachable at 60 V');
tally(Sn(5) > 1.2,  'saturation at 50 V');
bndN = min(Un(Sn < 1));
tally(bndN > 51 && bndN < 60, 'worst-case boundary between 51 V and 60 V');

fprintf('\nD. zero control current (bias circulation only)\n');
for U = [200 35]
    R = runPoint(m, P, zeros(1,5), U, 0.006, 'keep', true);
    fprintf('   U_dc=%5.0f V : S=%.4f  maxerr=%.4f\n', U, R.spread, max(abs(R.ctrl)));
    if U == 200
        tally(abs(R.spread - 0.2033) < 0.01, 'spread at zero control current matches 0.2033');
    else
        tally(R.spread > 0.90 && R.spread < 0.99, 'bias circulation alone needs about 35 V');
    end
end

% figures for the boundary study
f1 = figure('Visible','off','Position',[60 60 1150 800]);
subplot(2,2,1); hold on; grid on;
Uf = linspace(30,220,400);
plot(Uf, (P.R_value*(6*P.bias_current+10) + 1.9)./Uf + 0.04, 'b-');
plot(Uf, (P.R_value*(6*P.bias_current+19) + 1.9)./Uf + 0.04, 'r-');
plot(Uf, (P.R_value*6*P.bias_current + 1.9)./Uf + 0.04, 'g-');
plot([30 220],[1 1],'k-','LineWidth',1.4);
plot(Us, Ss, 'bo','MarkerFaceColor','b'); plot(Un, Sn, 'rs','MarkerFaceColor','r');
xlabel('U_{dc} [V]'); ylabel('spread S [pu]'); ylim([0 1.6]); xlim([30 220]);
title('spread vs bus voltage'); legend('sym \Delta=10 A','worst case','\Delta=0','S=1','measured sym','measured worst','Location','northeast');
subplot(2,2,2); hold on; grid on;
plot(Dv, Sv, 'bo-','MarkerFaceColor','b'); plot(Dv, 0.15+0.005*Dv, 'k--'); plot(Dv, pf(2)+pf(1)*Dv, 'r-');
xlabel('control current \Delta [A]'); ylabel('spread S [pu]'); title('spread vs control current at 200 V');
legend('measured','ideal','linear fit','Location','northwest');
subplot(2,2,3); hold on; grid on;
bar([200 100 60 50 45 40], [10 10 10 10 9.5 4], 0.5, 'FaceColor',[0.4 0.7 0.4]);
xlabel('U_{dc} [V]'); ylabel('achievable |\Delta| [A]'); ylim([0 11]); title('achievable control current');
subplot(2,2,4); hold on; grid on;
plot(Us, Ss, 'bo-'); plot(Un, Sn, 'rs-'); xlabel('U_{dc} [V]'); ylabel('S [pu]'); title('S vs U_{dc}');
legend('symmetric','worst case','Location','northeast');
print(f1,'-dpng','-r130',fullfile(Assets,'fig6_modulation_boundary.png'));
f2 = figure('Visible','off','Position',[60 60 900 400]); hold on; grid on;
plot(Uf, (P.R_value*(6*P.bias_current+10) + 1.9)./Uf + 0.04,'b-');
plot(Uf, (P.R_value*(6*P.bias_current+19) + 1.9)./Uf + 0.04,'r-');
plot([30 220],[1 1],'k-'); plot(Us,Ss,'bo','MarkerFaceColor','b'); plot(Un,Sn,'rs','MarkerFaceColor','r');
xlabel('U_{dc} [V]'); ylabel('S [pu]'); xlim([30 220]); ylim([0 2]);
title('analytic boundary vs measurement');
legend('sym (k=1)','worst (k=2)','S=1','measured','Location','northeast');
print(f2,'-dpng','-r130',fullfile(Assets,'fig7_analytic_limit.png'));
fprintf('   figures written: fig6_modulation_boundary.png, fig7_analytic_limit.png\n');
end

%% ===================== relocation ========================================
function caseRelocation(ModelDir, P, Assets)
fprintf('\nA. baseline, variant A (legs 3-5-7) and variant B (legs 2-5-8)\n');
files = {'Ten_phase_nine_leg_with_RL_load.slx','Ten_phase_nine_leg_relocated_L3_L7.slx','Ten_phase_nine_leg_relocated_L2_L8.slx'};
names = {'amb_b9p10_rel0','amb_b9p10_relA','amb_b9p10_relB'};
Ileg = cell(1,3); C = cell(1,3);
for i = 1:3
    m = instrumentCopy(fullfile(ModelDir,files{i}), names{i}, i);
    R = runPoint(m, P, ones(1,5), P.U_dc, 0.02, 'keep', false, i);
    Ileg{i} = mean(R.Ileg(R.ss,:)); C{i} = R.ctrl;
    fprintf('   %-34s legs = %s\n', files{i}, mat2str(round(Ileg{i},2)));
    fprintf('   %-34s ctrl = %s   sum(Ileg) = %.2e A\n', '', mat2str(round(C{i},4)), max(abs(R.sumLeg)));
    tally(max(abs(R.sumLeg)) < 1e-9, sprintf('model %d: nine leg currents sum to zero', i));
    if i == 2
        tally(max(abs(C{i} - [0.9992 1.0603 0.9057 1.0303 0.9988])) < 0.05, ...
              'variant A: documented dead-zone offsets on DOF2..DOF4');
    else
        tally(max(abs(C{i} - 1)) < 0.03, sprintf('model %d: control currents near 1 A', i));
    end
end
tally(max(abs(Ileg{2} - [5.51 4.51 -0.11 -4.48 -0.91 5.50 -0.06 -5.49 -4.49])) < 0.3, ...
      'variant A leg currents match the documented values');
tally(max(abs(Ileg{3} - [5.49 9.99 -5.49 -4.50 -1.00 5.49 4.50 -9.99 -4.50])) < 0.5, ...
      'variant B legs 2 and 8 carry 2*i_bias');
tally(abs(Ileg{3}(2)) > 9 && abs(Ileg{3}(8)) > 9, 'variant B doubles the device current on two legs');
tally(abs(Ileg{1}(4)) < 1.5 && abs(Ileg{1}(5)) < 1.5 && abs(Ileg{1}(6)) < 1.5, ...
      'baseline: legs 4-5-6 carry control current only');
tally(abs(Ileg{2}(3)) < 1.5 && abs(Ileg{2}(5)) < 1.5 && abs(Ileg{2}(7)) < 1.5, ...
      'variant A: legs 3-5-7 carry control current only');
f = figure('Visible','off','Position',[60 60 1000 700]);
ttl = {'baseline: axial pair on legs 4-5-6','variant A: axial pair on legs 3-5-7','variant B: axial pair on legs 2-5-8'};
for i = 1:3
    subplot(3,1,i); bar(1:9, Ileg{i}); grid on; ylim([-12 12]);
    ylabel('I_{leg} [A]'); title(ttl{i});
    if i == 3, xlabel('leg index'); end
end
print(f,'-dpng','-r130',fullfile(Assets,'fig5_relocated_leg_currents.png'));
fprintf('   figure written: fig5_relocated_leg_currents.png\n');
end

%% ===================== dead time =========================================
function caseDeadtime(ModelDir, P)
m = instrumentCopy(fullfile(ModelDir,'Ten_phase_nine_leg_with_RL_load.slx'),'amb_b9p10_dt',1);
fprintf('\nA. spread at zero control current (bias circulation only), U_dc = 200 V\n');
S = zeros(1,2);
for k = 1:2
    Td = [1e-6 0];  Td = Td(k);
    R = runPoint(m, P, zeros(1,5), 200, 0.006, Td, true);
    S(k) = R.spread;
    fprintf('   Td = %4.1f us : S = %.4f  duty=[%.3f %.3f]  maxerr=%.4f\n', Td*1e6, R.spread, R.dssmin, R.dssmax, max(abs(R.ctrl)));
end
tally(abs(S(1) - 0.2033) < 0.010, 'with 1 us dead time the spread is 0.2033');
tally(abs(S(2) - 0.1635) < 0.010, 'with no dead time the spread falls to 0.1635 (device drops only)');
tally(abs((S(1) - S(2)) - 2*1e-6/50e-6) < 0.006, 'spread penalty equals 2*Td/Ts = 0.04 pu');
fprintf('   dead-time penalty = %.4f pu (2*Td/Ts = %.4f pu)\n', S(1)-S(2), 2e-6/50e-6);

fprintf('\nB. tracking at i_ref = 1 A\n');
E = zeros(1,2);
for k = 1:2
    Td = [1e-6 0];  Td = Td(k);
    R = runPoint(m, P, ones(1,5), 200, 0.006, Td, true);
    E(k) = max(abs(R.ctrl-1));
    fprintf('   Td = %4.1f us : max|ctrl-1| = %.5f A  spread = %.4f\n', Td*1e6, E(k), R.spread);
end
tally(E(2) < E(1)/3, 'removing dead time cuts the steady-state error by more than 3x');
end

%% ===================== helpers ===========================================
function P = loadAmbParams(f)
run(f);
P = struct('R_value',R_value,'L_value',L_value,'U_dc',U_dc,'bias_current',bias_current, ...
           'f_c',f_c,'K_p',K_p,'K_i',K_i,'SW_frequency',SW_frequency,'SW_Ts',SW_Ts,'half_SW_Ts',half_SW_Ts);
fn = fieldnames(P);
for i = 1:numel(fn), assignin('base', fn{i}, P.(fn{i})); end   % the model reads them from the base workspace
end

function m = instrumentCopy(modelPath, name, variant)
scratch = fullfile(tempdir,'amb_b9p10_case_scratch');
if ~exist(scratch,'dir'), mkdir(scratch); end
target = fullfile(scratch,[name '.slx']);
if bdIsLoaded(name), close_system(name,0); end
if isfile(target), delete(target); end
copyfile(modelPath, target, 'f');
load_system(target);
prev = find_system(name,'SearchDepth',1,'Regexp','on','Name','^LOG_');
for i = 1:numel(prev), delete_block(prev{i}); end
y0 = 1600;
tags = {'i_a1','i_c1','i_a2','i_c2','i_a3','i_c3','i_a4','i_c4','i_a5','i_c5'};
add_block('simulink/Signal Routing/Mux',[name '/LOG_mux_i'],'Inputs','10','Position',[100 y0 105 y0+220]);
add_block('simulink/Sinks/To Workspace',[name '/LOG_i'],'VariableName','amb_i', ...
          'SaveFormat','Structure With Time','SampleTime','1e-6','Position',[160 y0 220 y0+40]);
for k = 1:10
    b = sprintf('LOG_from%d',k);
    add_block('simulink/Signal Routing/From',[name '/' b],'GotoTag',tags{k},'Position',[20 y0+(k-1)*22 60 y0+(k-1)*22+16]);
    add_line(name,[b '/1'],['LOG_mux_i/' num2str(k)],'autorouting','on');
end
add_line(name,'LOG_mux_i/1','LOG_i/1','autorouting','on');
pi = {'PI_ctrl','PI_ctrl1','PI_ctrl2','PI_ctrl3','PI_ctrl4','PI_ctrl5','PI_ctrl6','PI_ctrl7','PI_ctrl8','PI_ctrl9'};
add_block('simulink/Signal Routing/Mux',[name '/LOG_mux_u'],'Inputs','10','Position',[100 y0+300 105 y0+520]);
add_block('simulink/Sinks/To Workspace',[name '/LOG_u'],'VariableName','amb_u', ...
          'SaveFormat','Structure With Time','SampleTime','1e-6','Position',[160 y0+300 220 y0+340]);
for k = 1:10
    add_line(name,[pi{k} '/1'],['LOG_mux_u/' num2str(k)],'autorouting','on');
end
add_line(name,'LOG_mux_u/1','LOG_u/1','autorouting','on');
add_block('simulink/Signal Routing/Mux',[name '/LOG_mux_d'],'Inputs','9','Position',[100 y0+600 105 y0+800]);
add_block('simulink/Sinks/To Workspace',[name '/LOG_d'],'VariableName','amb_d', ...
          'SaveFormat','Structure With Time','SampleTime','1e-6','Position',[160 y0+600 220 y0+640]);
for k = 1:9
    add_line(name,['Demux/' num2str(k)],['LOG_mux_d/' num2str(k)],'autorouting','on');
end
add_line(name,'LOG_mux_d/1','LOG_d/1','autorouting','on');
save_system(name);
m = name;
fprintf('   instrumented copy staged (variant %d)\n', variant);
end

function R = runPoint(m, P, irefs, Udc, Tstop, Td, zeroRefs, variant)
% Td       = 'keep' keeps the delivered dead time, otherwise Td is in seconds
% zeroRefs = true applies all five references from t = 0 (short, fast settling)
% variant  = 1 original placement, 2 relocated to legs 3-5-7, 3 relocated to legs 2-5-8
if nargin < 6 || isempty(Td), Td = 'keep'; end
if nargin < 7, zeroRefs = true; end
if nargin < 8, variant = 1; end
keepTd = ischar(Td) && strcmp(Td,'keep');
in = Simulink.SimulationInput(m);
in = in.setModelParameter('StopTime', num2str(Tstop));
in = in.setVariable('U_dc', Udc);
in = in.setVariable('bias_current', P.bias_current);
in = in.setVariable('K_p', 2*pi*P.f_c*P.L_value/Udc);
in = in.setVariable('K_i', 2*pi*P.f_c*P.R_value/Udc);
names = {'Constant6','Constant1','Constant2','Constant3','Constant4'};   % DOF1..DOF5 i_ref
for k = 1:5
    in = in.setBlockParameter([m '/' names{k}],'Value',num2str(irefs(k)));
end
if ~keepTd
    for k = 0:8
        if k == 0, pwm = 'PWM_gen'; else, pwm = sprintf('PWM_gen%d',k); end
        for j = {'On-Off Delay','On-Off Delay1'}
            in = in.setBlockParameter([m '/' pwm '/' j{1}],'Td_on',num2str(Td));
        end
    end
end
if zeroRefs
    for s = {'Step','Step1','Step2'}
        in = in.setBlockParameter([m '/' s{1}],'Time','0');    % apply every reference from t = 0
    end
end
so = sim(in);
t  = so.get('amb_i').time;
I  = so.get('amb_i').signals.values;
U  = so.get('amb_u').signals.values;
D  = so.get('amb_d').signals.values;
ss = t > (Tstop - 0.001);
pair = [1 2;3 4;5 6;7 8;9 10];
R.t = t; R.I = I; R.U = U; R.D = D; R.ss = ss;
R.ctrl  = arrayfun(@(d) mean(I(ss,pair(d,1)) - I(ss,pair(d,2))), 1:5);
R.sum   = arrayfun(@(d) mean(I(ss,pair(d,1)) + I(ss,pair(d,2))), 1:5);
R.imean = mean(I(ss,:));
R.ripple = max(I(ss,:)) - min(I(ss,:));
switch variant
    case 2, R.Ileg = [ I(:,1), I(:,2), (-I(:,3)+I(:,5)), -I(:,4), (I(:,6)-I(:,5)), I(:,7), (I(:,8)-I(:,6)), -I(:,9), -I(:,10) ];
    case 3, R.Ileg = [ I(:,1), (I(:,2)+I(:,5)), -I(:,3), -I(:,4), (I(:,6)-I(:,5)), I(:,7), I(:,8), (-I(:,9)-I(:,6)), -I(:,10) ];
    otherwise, R.Ileg = [ I(:,1), I(:,2), -I(:,3), (I(:,5)-I(:,4)), (I(:,6)-I(:,5)), (I(:,7)-I(:,6)), I(:,8), -I(:,9), -I(:,10) ];
end
R.sumLeg = sum(R.Ileg,2);
R.dssmin = min(min(D(ss,:))); R.dssmax = max(max(D(ss,:)));
R.dallmin = min(D(:)); R.dallmax = max(D(:));
ub = bridgeVoltages(U, variant);
R.ub = ub;
R.spread = mean(max(ub(ss,:),[],2) - min(ub(ss,:),[],2));
R.Pcu = P.R_value*sum(mean(I(ss,:).^2));
R.Pw  = mean(sum(U(ss,:).*I(ss,:),2))*Udc;
R.Pleg = mean(sum(D(ss,:).*R.Ileg(ss,:),2))*Udc;
end

function ub = bridgeVoltages(U, variant)
switch variant
    case 2   % axial pair between legs 3 - 5 - 7
        ub = [ U(:,1)+U(:,3)+U(:,5), U(:,2)+U(:,3)+U(:,5), U(:,5)+U(:,3), -U(:,4)+U(:,3)+U(:,5), ...
               zeros(size(U,1),1), U(:,7)+U(:,8)-U(:,6), U(:,8)-U(:,6), -U(:,9)-U(:,8)+U(:,6), -U(:,10)-U(:,8)+U(:,6) ];
    case 3   % axial pair between legs 2 - 5 - 8
        ub = [ U(:,1)-U(:,2)+U(:,5), U(:,5), -U(:,3)-U(:,2)+U(:,5), -U(:,4)-U(:,2)+U(:,5), zeros(size(U,1),1), ...
               U(:,7)+U(:,9)-U(:,6), U(:,8)+U(:,9)-U(:,6), -U(:,6), U(:,9)-U(:,10)-U(:,6) ];
    otherwise
        ub = [ U(:,1)+U(:,4)+U(:,5), U(:,2)+U(:,4)+U(:,5), -U(:,3)+U(:,4)+U(:,5), U(:,5), zeros(size(U,1),1), ...
              -U(:,6), -U(:,7)-U(:,6)+U(:,8), -U(:,9)-U(:,7)-U(:,6), -U(:,10)-U(:,7)-U(:,6) ];
end
end

function tally(ok, label)
persistent nPass nFail
if isnan(ok)
    if label == 0
        nPass = 0; nFail = 0;
    else
        fprintf('\n%d checks passed, %d failed\n', nPass, nFail);
        assert(nFail == 0, '%d case-study checks FAILED', nFail);
        fprintf('run_case_study: ALL PASS\n');
    end
    return
end
if ok
    nPass = nPass + 1;
else
    nFail = nFail + 1;
    fprintf('FAIL  %s\n', label);
end
end