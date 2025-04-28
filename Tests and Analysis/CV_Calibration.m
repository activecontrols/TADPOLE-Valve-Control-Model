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

Cv_ox_plots = true;
Cv_ipa_plots = false;
Mdot_ox_plots = false;
Mdot_ipa_plots = false;
Cv_CMD = false;
controller_plots = false;

%% Initialize data and filter
dataWf = readmatrix("loxcv9");

rows = size(dataWf, 1);
cols = size(dataWf, 2);
t = dataWf(:, 1);
dataF = movingavg(dataWf);
dataF = dataF(1:rows, :);

%% Extract data
angle_Mox = dataWf(:, 6) * 360;
angle_Cox = dataWf(:, 4) * 360;
angle_Mipa = dataWf(:, 11) * 360;
angle_Cipa = dataWf(:, 5) * 360;

mdot_ox = dataF(:, 30);
mdot_ipa = dataF(:, 31);
mdot_trg_ox = dataWf(:, 32);
mdot_trg_ipa = dataWf(:, 33);

P_up_ox = dataF(:, 16);
P_dw_ox = dataF(:, 17);
P_out_ox = dataF(:, 18);
P_up_ipa = dataF(:, 21);
P_dw_ipa = dataF(:, 22);
P_out_ipa = dataF(:, 23);

OX_Integral = dataF(:, 27);
IPA_Integral = dataF(:, 29);
OX_FF = dataF(:, 34);
IPA_FF = dataF(:, 35);

% Venturi dimensions
A_th_ox = 0.0203;
A_in = 0.127;
rhoFluid = 0.036;
rhoWat = 0.036;

% Constants
g = 32.17 * 12;

% Cv and Mass Flow Estimation
cvV60OX = polyval(V60C, angle_Mox);
cvV30OX = polyval(V30C, angle_Mox);
cvV60IPA = polyval(V60C, angle_Mipa);
cvV30IPA = polyval(V30C, angle_Mipa);

% Estimate one massflow locally to make sure code estimation is fine. Plot
% manually if needed. Code uses the massflow estimated in the loop, not
% here.
DPValveOX = max((P_up_ox - P_dw_ox), 1e-3);
DPValveIPA = max((P_up_ipa - P_dw_ipa), 1e-3);
DPVenturiOX = max(P_dw_ox - P_out_ox, 0);
cor1 = mean(DPVenturiOX(1:30), 1);
DPVenturiOX = max(DPVenturiOX - cor1, 0);
mdot_ox_EST = A_th_ox .* sqrt(2 * rhoFluid * DPVenturiOX * g ./ (1 - (A_th_ox / A_in)^2));

% Modify when using a diffrent fluid. Change density and can't assume SG of 1.
Kf = 1.355;        % Tunable parameter (In the 1.33-1.37 range).
P_atm = 14.3;        % Tecnically just the backpressure on the valve.
                     % this becomes Pc in a real test.

cv2OX = mdot_ox_EST / 231 * 60 .* sqrt(1 ./ (DPValveOX * rhoFluid * rhoWat));
cv2IPA = mdot_ipa / 231 * 60 .* sqrt(1 ./ (DPValveIPA  * rhoFluid * rhoWat));

% Old feedforward command
FF_ox = mdot_trg_ox / rhoFluid / 231 * 60 .* sqrt(1 ./ DPValveOX);

% Retuned feedforward command
FF_ox_v3 = 60/231 * mdot_trg_ox .* sqrt(1 ./ (rhoFluid * rhoWat * (P_up_ox - P_atm - ...
            mdot_trg_ox.^2 / (2 * rhoFluid * (Kf * A_in)^2))));

% Feeforward V4
C_fric = 0.03;
d = 0.35;
C_d = 0.084;
l = 10;

DP_i = 1 / (2 * rhoFluid * g *(C_d * A_in)^2);
%DP_l = C_fric * l / (2 * d * g * rhoFluid * A_in^2);
%DPVenturi = (1 - (A_th_ox / A_in)^2) / (2 * A_th_ox^2 * rhoFluid * g);

FF_ox_v4 = 60/231 * mdot_trg_ox .* sqrt(1 ./ (rhoFluid * rhoWat * (P_up_ox - P_atm - ...
            (DP_i) * mdot_trg_ox .^2)));

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
    plot(t, mdot_ipa, 'b', 'LineWidth', 1); grid on; hold on;
    plot(t, mdot_trg_ipa, 'r', 'LineWidth', 1)
    xlabel('Time [s]');
    ylabel('Mass Flow [lbm/s]');
    title('IPA Mass Flow vs. Time');
    legend('Mass Flow', 'Target');
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
    for i = angles
        upperbound = min(i + 2, length(angles));
        CVMODEL(upperbound) = max(CVMODEL(i + 1), CVMODEL(upperbound));
    end

    CVMODEL2 = 2.50 ./ (1 + exp(-(angles - 58) / 11));
    plot(angles, CVMODEL, 'm','LineWidth',1.5);
    plot(angles, CVMODEL2, 'r','LineWidth', 0.9);
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

    % Make sure polynomial fit is always increasing
    for i = angles
        upperbound = min(i + 2, length(angles));
        CVMODEL(upperbound) = max(CVMODEL(i + 1), CVMODEL(upperbound));
    end
    CVMODEL2 = 2.95 ./ (1 + exp(-(angles - 63) / 10));
    plot(0:1:90, CVMODEL, 'm','LineWidth',1.5);
    plot(angles, CVMODEL2, 'r','LineWidth', 0.9);
    legend('Estimated Cv', 'Angle to Cv Mapping');
    title('Cv Comparaison IPA')
    hold off
    fprintf("Interpolation Table for IPA Cv: \n")
    disp([0:6:90; CVMODEL(1:6:91)]);
 
    % Plot CMD angle vs Measured Angle
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
    plot(t, FF_ox_v4, 'g', 'LineWidth',1);
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