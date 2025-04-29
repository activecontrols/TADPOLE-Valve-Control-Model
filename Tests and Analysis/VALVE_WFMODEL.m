% Time span
tspan = [0 13];

% Initial conditions
x0 = [14.3 0];

% Simulate system
P_tank = 310;
cvCOEFS = [-0.000000237649393   0.000037435620733  -0.001322241203911   0.015628964005178   0];
% cvCOEFS = [0.000005320163756   0.000008288008288   0.011123068289735  0];
% min(9 * t, 90)

rhoFluid = 0.035;       % 0.035 OX || 0.02836 IPA
g = 32.174 * 12;        % in/s^2
C_d =  0.45;            % 0.35 OX || 0.72 IPA
A_i = 0.0498;           % 0.0498 OX || 0.04031 IPA

mdot = 0.4:0.02:2.5;
DP_i = mdot.^2 / (2 * rhoFluid * g *(C_d * A_i)^2);
for i = 1:length(DP_i)
    fprintf("Pressure Drop at %.2f lbm/s:   %.2f psi\n", mdot(i), DP_i(i));
end

%% ODE Sim
%[t, x] = ode15s(@(t, x) plantColdflow(x, min(9 * t, 90), cvCOEFS, P_tank), tspan, x0);

%% Plots
% figure;
% angles = min(9 * t, 90);
% hold on
% plot(angles, x(:,2), 'b', 'LineWidth', 2);
% xlabel('Angle [deg]');
% ylabel('IPA Flow [lbm/s]');
% title('Mass Flow');
% grid on
% xline(75, 'r--');
% xline(25, 'r--');
% 
% 
% figure;
% hold on
% plot(angles, P_tank - x(:,1), 'b', 'LineWidth', 2);
% xlabel('Angle [deg]');
% ylabel('Pressure Drop of Valve [psi]');
% title('Pressure Drop across Valve');
% xline(75, 'r--');
% xline(25, 'r--');
% grid on
% hold on
