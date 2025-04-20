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
Mdot_ox_plots = true;
Mdot_ipa_plots = false;
Cv_CMD = true;

%% Initialize data and filter
dataWf = readmatrix("CL 4_19.xlsx");

rows = size(dataWf, 1);
cols = size(dataWf, 2);
t = dataWf(:, 1);
dataF = movingavg(dataWf);
dataF = dataF(1:rows, :);

%% Extract data
angle_Mox = dataWf(:, 7) * 360;
angle_Cox = dataWf(:, 4) * 360;
angle_Mipa = dataWf(:, 12) * 360;
angle_Cipa = dataWf(:, 6) * 360;

mdot_ox = dataF(:, 31);
mdot_ipa = dataF(:, 32);
mdot_trg_ox = dataWf(:, 33);
mdot_trg_ipa = dataWf(:, 34);

P_up_ox = dataF(:, 17);
P_dw_ox = dataF(:, 18);
P_out_ox = dataF(:, 19);
P_up_ipa = dataF(:, 22);
P_dw_ipa = dataF(:, 23);
P_out_ipa = dataF(:, 24);

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
mdot_ox_EST = A_th_ox .* sqrt(2 * rhoFluid * DPVenturiOX * g ./ (1 - (A_th_ox / A_in)^2));

% Modify when using a diffrent fluid. Change density and can't assume SG of 1.
CdFF = 1.355;        % Tunable parameter (In the 1.33-1.37 range).
P_atm = 14.3;        % Tecnically just the backpressure on the valve.
                     % this becomes Pc in a real test.

cv2OX = mdot_ox_EST / 231 * 60 .* sqrt(1 ./ (DPValveOX * rhoFluid * rhoWat));
cv2IPA = mdot_ipa / 231 * 60 .* sqrt(1 ./ (DPValveIPA  * rhoFluid * rhoWat));

% Old feedforward command
FF_ox = mdot_trg_ox / rhoFluid / 231 * 60 .* sqrt(1 ./ DPValveOX);

% Retuned feedforward command
FF_ox_v3 = 60/231 * mdot_trg_ox .* sqrt(1 ./ (rhoFluid * rhoWat * (P_up_ox - P_atm - ...
            mdot_trg_ox.^2 / (2 * rhoFluid * (CdFF * A_in)^2))));

%% Plots
if Mdot_ox_plots == true
    figure;
    plot(t, mdot_ox, 'b', 'LineWidth', 1); grid on; hold on;
    plot(t, mdot_trg_ox, 'r', 'LineWidth', 1)
    xlabel('Time [s]');
    ylabel('Mass Flow [lbm/s]');
    title('Ox Mass Flow vs. Time');
    legend('Mass Flow', 'Target');
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

    subplot(1,2,1);
    plot(angle_Mox, cvV60OX, 'r-x','MarkerSize',5); grid on; hold on;
    plot(angle_Mox, cvV30OX, 'b-x','MarkerSize',5); grid on; hold on;
    plot(angle_Mox, cv2OX, 'g-x', 'MarkerSize',5);
    xlim([0 90]);
    ylim([0 6]);
    xlabel('Valve Angle OX [deg]');
    ylabel('Cv');
    
    % Add a curve fit to local Cv to account for phase shift
    CVcoef = polyfit(angle_Mox, cv2OX, 4);
    CVMODEL = polyval(CVcoef, 0:1:90);
    plot(0:1:90, CVMODEL, 'm','LineWidth',0.9);
    legend('V60 Cv', 'V30 Cv', 'Estimated Cv', 'Curve Fit for Data');
    title('Cv Comparaison OX')
    
    % Plot Cvs over time
    subplot(1,2,2);
    plot(t, cvV60OX, 'r', 'LineWidth', 1); hold on; grid on;
    plot(t, cvV30OX, 'b', 'LineWidth', 1); hold on; grid on;
    plot(t, cv2OX, 'g', 'LineWidth',1);
    xlabel('Time [s]');
    ylabel('Cv');
    title('OX Valve Cv vs. Time');
    legend('V60 Cv','V30 Cv', 'Estimated Cv');
    sgtitle('OX Cv Estimation Data');
    
    % Plot CMD angle vs Measured Angle
    figure;
    plot(t, angle_Cox, 'r', 'LineWidth', 1); hold on; grid on;
    plot(t, angle_Mox, 'g', 'LineWidth',1);
    xlabel('Time [s]');
    ylabel('Valve Angle [deg]');
    title('OX Angle vs. Time');
    legend('Commanded Angle', 'Measured Angle');
end
if Cv_ipa_plots == true
    figure;

    subplot(1,2,1);
    plot(angle_Mipa, cvV60IPA, 'r-x','MarkerSize',5); grid on; hold on;
    plot(angle_Mipa, cvV30IPA, 'b-x','MarkerSize',5); grid on; hold on;
    plot(angle_Mipa, cv2IPA, 'g-x', 'MarkerSize',5);
    xlim([0 90]);
    ylim([0 6]);
    xlabel('Valve Angle IPA [deg]');
    ylabel('Cv');
    
    % Add a curve fit to local Cv to account for phase shift
    CVcoef = polyfit(angle_Mipa, cv2IPA, 4);
    CVMODEL = polyval(CVcoef, 0:1:90);
    plot(0:1:90, CVMODEL, 'm','LineWidth',0.9);
    legend('V60 Cv', 'V30 Cv', 'Estimated Cv', 'Curve Fit for Data');
    title('Cv Comparaison IPA')
    
    % Plot Cvs over time
    subplot(1,2,2);
    plot(t, cvV60IPA, 'r', 'LineWidth', 1); hold on; grid on;
    plot(t, cvV30IPA, 'b', 'LineWidth', 1); hold on; grid on;
    plot(t, cv2IPA, 'g', 'LineWidth',1);
    xlabel('Time [s]');
    ylabel('Cv');
    title('IPA Valve Cv vs. Time');
    legend('V60 Cv','V30 Cv', 'Estimated Cv');
    sgtitle('IPA Cv Estimation Data');
    
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
    plot(t, FF_ox_v3, 'b', 'LineWidth',1);
    xlabel('Time [s]');
    ylabel('Cv');
    title('Cv vs. Time');
    legend('Commanded Cv Online', 'FF v3');
    ylim([0 3]);
end