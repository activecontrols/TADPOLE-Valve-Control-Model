% Time span
tspan = [0 13];

% Initial conditions
x0 = [14.3 0];

% Simulate system
P_tank = 80;
cvCOEFS = [-0.000000237649393   0.000037435620733  -0.001322241203911   0.015628964005178   0];
% cvCOEFS = [0.000005320163756   0.000008288008288   0.011123068289735  0];
% min(9 * t, 90)
[t, x] = ode15s(@(t, x) plantWaterflow(x, min(9 * t, 90), cvCOEFS, P_tank), tspan, x0);

%% Plots
figure;
angles = min(9 * t, 90);
hold on
plot(angles, x(:,2), 'b', 'LineWidth', 2);
xlabel('Angle [deg]');
ylabel('Water Flow [lbm/s]');
title('Mass Flow');
grid on

figure;
hold on
plot(angles, P_tank - x(:,1), 'b', 'LineWidth', 2);
xlabel('Angle [deg]');
ylabel('Pressure Drop of Valve [psi]');
title('Pressure Drop across Valve');
grid on
hold on
