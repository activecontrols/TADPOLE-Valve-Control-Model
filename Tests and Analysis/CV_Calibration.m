%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Code takes in a .csv or .xlsx data log from a TADPOLE test and plots out
% the data and respective analysis for the test. It also makes use of a
% rolling window mean filter to kill some of the noise in the data.
%
% Made by Pablo Plata - 04/18/25
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Curve fit V30 and V60 Valves
addpath('.DATA\')
V30 = [0:9:90;
       0.00 0.05 0.118 0.236 0.405 0.624 0.880 1.200 1.550 1.954 2.380];
V30C = polyfit(V30(1,:), V30(2,:), 3);


V60 = [0:9:90;
       0.00 0.07 0.161 0.378 0.670 1.000 1.450 2.050 2.780 3.710 4.960];
V60C = polyfit(V60(1,:), V60(2,:), 3);

TADPOLECv = [-2.3765e-07   3.7436e-05  -1.3222e-3   0.015629   0];
%% Settings

Cv_ox_plots = false;
Cv_ipa_plots = false;
Mdot_ox_plots = false;
Mdot_ipa_plots = false;
Cv_CMD = false;
controller_plots = false;

%% Initialize data and filter
dataWf = readmatrix("IPA FF");

rows = size(dataWf, 1);
cols = size(dataWf, 2);
t = dataWf(:, 1);
dataF = movingavg(dataWf);
dataF = dataF(1:rows, :);

%% Extract data
angle_Mox = dataWf(:, 6) * 360;
angle_Cox = dataWf(:, 4) * 360;
angle_Mipa = dataWf(:, 10) * 360;
angle_Cipa = dataWf(:, 5) * 360;

mdot_ox = dataF(:, 29);
mdot_ipa = dataF(:, 30);
mdot_trg_ox = dataWf(:, 31);
mdot_trg_ipa = dataWf(:, 32);

P_up_ox = dataF(:, 15);
P_dw_ox = dataF(:, 16);
P_diff_ox = dataF(:, 17);
P_up_ipa = dataF(:, 20);
P_dw_ipa = dataF(:, 21);
P_diff_ipa = dataF(:, 22);
Pc = dataF(:, 14);

OX_Integral = dataF(:, 26);
IPA_Integral = dataF(:, 28);
OX_FF = dataF(:, 33);
IPA_FF = dataF(:, 34);

% Constants
g = 32.17 * 12;
rhoFluid = 0.036;       
rhoWat = 0.036;

% Pressure estimations
DPValveOX = max((P_up_ox - P_dw_ox), 1e-3);
DPValveIPA = max((P_up_ipa - P_dw_ipa), 1e-3);
rhoOX = 0.0455;
rhoIPA = 0.02836; %49.06838 / 1728;

cv2OX = mdot_ox / 231 * 60 .* sqrt(1 ./ (DPValveOX * rhoOX * rhoWat));
cv2IPA = mdot_ipa / 231 * 60 .* sqrt(1 ./ (DPValveIPA  * rhoIPA * rhoWat));

% Feeforward Controller
C_d_IPA = 0.69;
C_d_OX = 0.35;
A_if = 0.04031;       % 0.0498 OX || 0.04031 IPA;
A_io = 0.0498;

DP_if = 1 / (2 * rhoIPA * g *(C_d_IPA * A_if)^2);
DP_io = 1 / (2 * rhoOX * g *(C_d_OX * A_io)^2); 

% APPROX CV MAPPING 
alpha_OX = 2.50;
beta_OX = 58;
gamma_OX = 11;

alpha_IPA = 2.95;
beta_IPA = 63;
gamma_IPA = 10;

% Feedforward
DP = max(P_up_ox - Pc - (DP_io) * mdot_trg_ox .^2, 0);
FF_ox = 60/231 * mdot_trg_ox .* sqrt(1 ./ (rhoOX * rhoWat * DP));
valve_ox = min(max(-gamma_OX * log(alpha_OX ./ FF_ox - 1) + beta_OX, 15), 90);

DP = max(P_up_ipa - Pc - (DP_if) * mdot_trg_ipa .^2, 0);
FF_ipa = 60/231 * mdot_trg_ipa .* sqrt(1 ./ (rhoIPA * rhoWat * DP));
valve_ipa = min(max(-gamma_IPA * log(alpha_IPA ./ FF_ipa - 1) + beta_IPA, 15), 90);

%% Plots
if Mdot_ox_plots == true
    figure;
    dt = t(end) / size(t, 1);
    tpast = [mdot_trg_ox(1:floor(0.5 / dt)); mdot_trg_ox];
    tpast = tpast(1:end-floor(0.5 / dt));
    tfut = [mdot_trg_ox; mdot_trg_ox(end-floor(0.5 / dt):end)];
    tfut = tfut(floor(0.5 / dt):end);
    high_bound = max(tpast, tfut(1:size(t, 1))) + max(mdot_trg_ox) * 0.05;
    low_bound = min(tpast, tfut(1:size(t, 1))) - max(mdot_trg_ox) * 0.05;
    plot(t, mdot_ox, 'b', 'LineWidth', 1); grid on; hold on;
    plot(t, mdot_trg_ox, 'r', 'LineWidth', 1)
    % plot(t, mdot_ox_EST, 'g', 'LineWidth', 1)
    plot(t, high_bound, 'LineWidth', 1);
    plot(t, low_bound, 'LineWidth', 1);
    xlabel('Time [s]');
    ylabel('Mass Flow [lbm/s]');
    title('Ox Mass Flow vs. Time');
    legend('Mass Flow', 'Target', 'High Bound', 'Low Bound');
end
if Mdot_ipa_plots == true
    figure;
    dt = t(end) / size(t, 1);
    tpast = [mdot_trg_ipa(1:floor(0.5 / dt)); mdot_trg_ipa];
    tpast = tpast(1:end-floor(0.5 / dt));
    tfut = [mdot_trg_ipa; mdot_trg_ipa(end-floor(0.5 / dt):end)];
    tfut = tfut(floor(0.5 / dt):end);
    high_bound = max(tpast, tfut(1:size(t, 1))) + max(mdot_trg_ipa) * 0.05;
    low_bound = min(tpast, tfut(1:size(t, 1))) - max(mdot_trg_ipa) * 0.05;
    plot(t, mdot_ipa, 'b', 'LineWidth', 1); grid on; hold on;
    plot(t, mdot_trg_ipa, 'r', 'LineWidth', 1)
    plot(t, high_bound, 'LineWidth', 1);
    plot(t, low_bound, 'LineWidth', 1);
    xlabel('Time [s]');
    ylabel('Mass Flow [lbm/s]');
    title('IPA Mass Flow vs. Time');
    legend('Mass Flow', 'Target', 'High Bound', 'Low Bound');
end
if Cv_ox_plots == true
    figure;
    plot(angle_Mox, cv2OX, 'b-x', 'MarkerSize', 4);
    xlim([0 90]);
    ylim([0 4]);
    xlabel('Valve Angle OX [deg]');
    ylabel('Cv');
    hold on; grid on;
    
    % Add a curve fit to local Cv to account for phase shift
    CVcoef = polyfit(angle_Mox, cv2OX, 4);
    angles = 0:1:90;
    CVMODEL = polyval(CVcoef, angles);

    % Make sure polynomial fit is always increasing
    % for i = angles
    %     upperbound = min(i + 2, length(angles));
    %     CVMODEL(upperbound) = max(CVMODEL(i + 1), CVMODEL(upperbound));
    % end

    %CVMODEL2 = 2.50 ./ (1 + exp(-(angles - 58) / 11));
    plot(angles, CVMODEL, 'm','LineWidth',1.5);
    %plot(angles, CVMODEL2, 'r','LineWidth', 0.9);
    legend('Estimated Cv', 'Angle to Cv Mapping');
    title('Cv Comparaison OX')
    hold off;
    fprintf("Interpolation Table for LOX Cv: \n")
    disp([0:6:90; CVMODEL(1:6:91)]);
       
    % Plot CMD angle vs Measured Angle
    figure;
    plot(t, angle_Cox, 'r', 'LineWidth', 1); hold on; grid on;
    plot(t, angle_Mox, 'b', 'LineWidth',1);
    xlabel('Time [s]');
    ylabel('Valve Angle [deg]');
    title('OX Angle vs. Time');
    legend('Commanded Angle', 'Measured Angle');
end
if Cv_ipa_plots == true
    figure;
    plot(angle_Mipa, cv2IPA, 'b-x', 'MarkerSize',5);
    xlim([0 90]);
    ylim([0 4]);
    xlabel('Valve Angle IPA [deg]');
    ylabel('Cv');
    hold on; grid on;
    
    % Add a curve fit to local Cv to account for phase shift
    CVcoef = polyfit(angle_Mipa, cv2IPA, 5);
    angles = 0:1:90;
    CVMODEL = polyval(CVcoef, 0:1:90);

    plot(0:1:90, CVMODEL, 'm','LineWidth',1.5);
    legend('Estimated Cv', 'Angle to Cv Mapping');
    title('Cv Comparaison IPA')
    hold off
    fprintf("Interpolation Table for IPA Cv: \n")
    disp([0:6:90; CVMODEL(1:6:91)]);
 
    %% Plot CMD angle vs Measured Angle
    figure;
    plot(t, angle_Cipa, 'r', 'LineWidth', 1); hold on; grid on;
    plot(t, angle_Mipa, 'g', 'LineWidth',1);
    xlabel('Time [s]');
    ylabel('Valve Angle [deg]');
    title('IPA Angle vs. Time');
    legend('Commanded Angle', 'Measured Angle');
end
if Cv_CMD == true
    figure;
    CvCMD = polyval(TADPOLECv, angle_Cox);
    plot(t, CvCMD, 'r', 'LineWidth', 1); hold on; grid on;
    plot(t, FF_ox, 'g', 'LineWidth',1);
    xlabel('Time [s]');
    ylabel('Cv');
    title('Cv vs. Time');
    legend('Commanded Cv Online', 'FF v4');
    ylim([0 3]);
end
if controller_plots == true
    figure;
    plot(t, OX_FF ./ (OX_FF + abs(OX_Integral)) * 100, 'r', 'LineWidth', 2); hold on; grid on;
    plot(t, abs(OX_Integral) ./ (OX_FF + abs(OX_Integral)) * 100, 'b', 'LineWidth', 2);
    xlabel('Time [s]');
    ylabel('Percent of Control Action [%]');
    title('Control Action');
    legend('Feedforward', 'Feedback Trim');
    hold off
end