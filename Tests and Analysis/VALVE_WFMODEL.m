% Time span
tspan = [0 10];

% Initial conditions
x0 = [0 14.696];

% Simulate system
[t, x] = ode45(@(t, x) plantWaterflow(x, 10, 0, 0), tspan, x0);

%% Plots
figure(2);
subplot(1,2,1)
plot(t, x(:,1), 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Water Flow [lbm/s]');
title('Mass Flow');
grid on
hold off

subplot(1,2,2)
plot(t, x(:,2), 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Pressure Downstream of Valve [psi]');
title('Downstream Pressure');
grid on
hold off
