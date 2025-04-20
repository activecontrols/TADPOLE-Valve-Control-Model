% Time span
tspan = [0 13];

% Initial conditions
x0 = [0 0];

% Simulate system
cvCOEFS = [-0.000000237649393   0.000037435620733  -0.001322241203911   0.015628964005178   0];
[t, x] = ode45(@(t, x) plantWaterflow(x, min(9 * t, 90), cvCOEFS), tspan, x0);

%% Plots
figure(2);
plot(t, x(:,2), 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Water Flow [lbm/s]');
title('Mass Flow');
grid on
hold off

figure;
plot(min(9 * t, 90), 80 - x(:,1), 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Pressure Drop of Valve [psi]');
title('Pressure Drop across Valve');
grid on
hold on
