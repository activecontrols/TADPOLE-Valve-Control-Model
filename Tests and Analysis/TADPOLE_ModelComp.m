%% Test 1
clear;
ox_density = 0.04126099537; % lbs / in^3
ipa_density = 0.02836; % lbs / in^3
addpath('plant')

% pouts = 3:0.1:5;
angles = 0:1:90;
idx = 1;
pout_ox = zeros(91,1);
pout_ipa = zeros(91,1);
for i = angles
    pout_ipa(idx) = max(valveangle2pout(i, 550, 0.02836 * 1728, 2),0);
    pout_ox(idx) = max(valveangle2pout(i, 550, 0.04126099537 * 1728, 2),0);
    idx = idx + 1;
end
plot(angles, pout_ox)


%% Model 1
tspan = [0 2];

% Define initial conditions
x0 = [0 0 0 0];

% Step angles
angle_ox = 10;
angle_ipa = 10;

% Simulate system
[t, x] = ode45(@(t, x) nonlinear_plant(x, angle_ox, angle_ipa, t), tspan, x0);

%% Model 2
% Define initial conditions
x0 = [0 1e-5 0 0 0 0];           

[t2, x2] = ode45(@(t2, x2) nonlinear_plant2(x2, angle_ox, angle_ipa, t2), tspan, x0);

%% Plots
figure(2);
hold off
subplot(2,3,1)
plot(t, x(:,1), 'b', 'LineWidth', 2);
hold on
plot(t2, x2(:,1), 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Oxidizer Flow [lbm/s]');
title('Step Response');
legend('Model 1', 'Model 2');
grid on
hold off

subplot(2,3,2)
plot(t, x(:,2), 'b', 'LineWidth', 2);
hold on
plot(t2, x2(:,2), 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Fuel Flow [lbm/s]');
title('Step Response');
legend('Model 1', 'Model 2');
grid on
hold off

subplot(2,3,3)
plot(t, x(:,3), 'b', 'LineWidth', 2);
hold on
plot(t2, x2(:,3), 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Chamber Pressure [psi]');
title('Step Response');
legend('Model 1', 'Model 2');
grid on
hold off

subplot(2,3,4)
plot(t2, x2(:,5), 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Pressure Downstream of Valve OX [psi]');
title('Step Response');
legend('Model 2')
grid on
hold off

subplot(2,3,5)
plot(t2, x2(:,6), 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Pressure Downstream of Valve IPA [psi]');
title('Step Response');
legend('Model 2')
grid on
hold off