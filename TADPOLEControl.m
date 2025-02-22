%% Setup
clear;
clear mex; % resets values inside PID
addpath('closed_loop_valve_control_cpp');
addpath('plant')

%% Delays and noise
actuator_delay = 0.1131;
noise_var_actuators = 0.1039 / 50;
noise_var_outputs = 1;
noise_var_mdot = 5e-5;
%model_error = 0.9 + 0.2*rand;

timeDelta = 1/100;

%% TESTS 
tspan = [0 2];

% Define state vector and initials
%x = [mdot_ox; mdot_ipa; Pc];
x0 = [0 0 0];

% Time constants
tau_mdot = 0.11 / 4;
tau_pc = 1.5 / 4;

% Step angles
angle_ox = 90;
angle_ipa = 90;

% Simulate system
[t, x] = ode45(@(t, x) nonlinear_plant(x, angle_ox, angle_ipa, 0), tspan, x0);

% Plots step response
figure(1);
subplot(2,2,1)
plot(t, x(:,1), 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Oxidizer Flow [lbm/s]');
title('Step Response');
grid on

subplot(2,2,2)
plot(t, x(:,2), 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Fuel Flow [lbm/s]');
title('Step Response');
grid on

subplot(2,2,3)
plot(t, x(:,3), 'k', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Chamber Pressure [psi]');
title('Step Response');
grid on

% Angle to Mdot Response
angles = linspace(1,90,400);
mdot_ipa = zeros(400,1);
mdot_ox = zeros(400,1);
idx = 1;

for i = angles
    mdot_ipa(idx) = valve_angle_to_mdot(i, 400, 150, 0.02836);
    mdot_ox(idx) = valve_angle_to_mdot(i, 400, 150, 0.04126099537);
    idx = idx + 1;
end

%Display
subplot(2,2,4)
plot(angles, mdot_ox, 'g', 'LineWidth', 2);
hold on
plot(angles, mdot_ipa, 'r', 'LineWidth', 2);
xlabel('Valve Angle [deg]');
ylabel('Mass Flow');
title('Mass Flow vs. Valve Angle');
xlim([0 90]);
legend('Mass Flow Oxidizer', 'Mass Flow IPA','Location','northwest')
grid on 
hold off

%Thrust curve
timeSim = linspace(0,20,200);
t_thrust = zeros(200,1);
high_bound = zeros(200,1);
low_bound = zeros(200,1);
idx = 1;

for time = timeSim
    thrust = thrustCurve(time);
    t_past = thrustCurve(time - 0.5);
    t_future = thrustCurve(time + 0.5);
    high_bound(idx) = max([thrust, t_past, t_future]) + 0.05 * 550;
    low_bound(idx) = min([thrust, t_past, t_future]) - 0.05 * 550;
    t_thrust(idx) = thrust;
    idx = idx + 1;
end

figure(2);
plot(timeSim, low_bound, 'b', 'LineWidth', 2);
hold on
plot(timeSim, t_thrust, 'g', 'LineWidth', 2);
plot(timeSim, high_bound, 'r', 'LineWidth', 2);
grid on
xlim([0 20]);
ylim([0 600]);
xlabel('Time [s]');
ylabel('Target Thrust');
title('Thrust Curve');
legend('Low Bound', 'Target', 'High Bound','Location','southeast');
hold off
