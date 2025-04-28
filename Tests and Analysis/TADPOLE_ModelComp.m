%% Test 1
clear;
ox_density = 0.04126099537; % lbs / in^3
ipa_density = 0.02836; % lbs / in^3
water_density = 0.0360724;

% pouts = 3:0.1:5;
angles = 0:1:90;
idx = 1;
pout_ox = zeros(91,1);
pout_ipa = zeros(91,1);
mdot = zeros(91,1);
for i = angles
    pout_ipa(idx) = max(valveangle2pout(i, 80, 0.03, 0.35),0);
    pout_ox(idx) = max(valveangle2pout(i, 80, 0.03, 0.35),0);
    mdot(idx) = valve_angle_to_mdot(i, 110, 14.696, water_density);
    idx = idx + 1;
end
figure(1);
plot(angles, 80 - pout_ox, 'b','LineWidth',0.8);
grid on
xlabel('Angles [deg]');
ylabel('Pressure Drop');
title('Pressure Drop vs Valve Angle');


%% Model 1
tspan = [0 5];

% Define initial conditions
x0 = [0 0 0 0];

% Step angles
angle_ox = 52;   %[52 30]
angle_ipa = 51;  %[51 35]

% Simulate system
[t, x] = ode45(@(t, x) nonlinear_plant(x, angle_ox, angle_ipa, t), tspan, x0);

%% Model 2
% Define initial conditions
P_atm = 14.696; %psi

x0 = [0 0 14.3 14.3 14.3 0 0];           

ox_tank = 600;
ipa_tank = 700;
[t2, x2] = ode45(@(t2, x2) nonlinear_plant2(x2, (t2 > 2) * angle_ox, (t2 > 2) * angle_ipa, t2, 0), tspan, x0(1:5));
[t3, x3] = ode45(@(t3, x3) nonlinear_plant3(x3, (t3 > 2) * angle_ox, (t3 > 2) * angle_ipa, t3, 0, ox_tank, ipa_tank), tspan, x0);

%% Plots
figure(2);
hold off
subplot(2,3,1)
% plot(t, x(:,1), 'b', 'LineWidth', 2);
hold on
%plot(t2, x2(:,1), 'r', 'LineWidth', 2);
plot(t3, x3(:,1), 'g', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Oxidizer Flow [lbm/s]');
title('Step Response');
legend('Model 2', 'Model 3');
grid on
hold off

subplot(2,3,2)
% plot(t, x(:,2), 'b', 'LineWidth', 2);
hold on
%plot(t2, x2(:,2), 'r', 'LineWidth', 2);
plot(t3, x3(:,2), 'g', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Fuel Flow [lbm/s]');
title('Step Response');
legend('Model 2', 'Model 3');
grid on
hold off

subplot(2,3,3)
% plot(t, x(:,3), 'b', 'LineWidth', 2);
hold on
%plot(t2, x2(:,3), 'r', 'LineWidth', 2);
plot(t3, x3(:,3), 'g', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Chamber Pressure [psi]');
title('Step Response');
legend('Model 2', 'Model 3');
grid on
hold off

subplot(2,3,4)
%plot(t2, x2(:,4), 'r', 'LineWidth', 2);
hold on
plot(t3, x3(:,4), 'g', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Pressure Downstream of Valve OX [psi]');
title('Step Response');
legend('Model 2', 'Model 3')
grid on
hold off

subplot(2,3,5)
%plot(t2, x2(:,5), 'r', 'LineWidth', 2);
hold on;
plot(t3, x3(:,5), 'g', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Pressure Downstream of Valve IPA [psi]');
title('Step Response');
legend('Model 2', 'Model 3')
grid on
hold off

hold off
subplot(2,3,6)
% plot(t, x(:,1), 'b', 'LineWidth', 2);
hold on
%plot(t2, (t2 > 2) .* x2(:,1) ./ x2(:,2), 'r', 'LineWidth', 2);
plot(t3, (t3 > 2) .* x3(:,1) ./ x3(:,2), 'g', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('OF Ratio');
title('Step Response');
legend('Model 2', 'Model 3');
ylim([1, 1.4])
xlim(tspan)
grid on
hold off
