%% Setup
clear;
clear mex; % resets values inside PID
quick_compiler
addpath('closed_loop_valve_control_cpp');
addpath('plant')
addpath('TADPOLE 10K')

%Delays and noise
actuator_delay = 0.1131;
noise_var_actuators = 0.1039 / 50;
noise_var_pc = 2*0.28;
noise_var_mdot = 1e-7;
timeDelta = 1/500;          % seconds
notch_freq = 150;           % Hz
deadtime_Pc = 0.055;         % seconds

% Manifold pressure data for display tests
ox_manifold_table = [
    192.5, 210.556, 228.611, 246.667, 264.722, 282.778, 300.833, 318.889, 336.944, 355.0, 373.056, 391.111, 409.167, 427.222, 445.278, 463.333, 481.389, 499.444, 517.5, 535.556, 550.0;
    118.055, 127.34, 136.64, 146.082, 155.44, 165.182, 175.069, 184.896, 194.888, 205.049, 215.378, 225.702, 236.381, 246.93, 257.695, 268.683, 279.647, 290.878, 301.999, 313.482, 322.79
];

ipa_manifold_table = [
    192.5, 210.556, 228.611, 246.667, 264.722, 282.778, 300.833, 318.889, 336.944, 355.0, 373.056, 391.111, 409.167, 427.222, 445.278, 463.333, 481.389, 499.444, 517.5, 535.556, 550.0;
    120.844, 130.535, 140.262, 150.16, 159.99, 170.243, 180.67, 191.054, 201.631, 212.408, 223.382, 234.368, 245.752, 257.017, 268.529, 280.3, 292.062, 304.129, 316.095, 328.466, 338.509
];

cf_table = [
    220, 550;
    1.12, 1.3
];

%Load Simulink Model
x0 = [0 0 0 0];
model = 'TADPOLE_Closed_Loop';
load_system(model);
out = sim(model);

%Thrust Data
data = out.ThrustCurves.Data;
timeSim = out.ThrustCurves.Time;
high_bound = data(:,3);
low_bound = data(:,2);
t_thrust = data(:,1);
cloop_thrust = data(:,4);
oloop_thrust = data(:,5);

% OL Chamber Pressure Data
chamber_pressure = out.ChamberData.Data;

% Mass Flow Ratio Data
dataMFR = out.MFRData.Data;
closedMFR = dataMFR(:,1);
openMFR = dataMFR(:,2);

%% 5 Thrust Curve Tests
figure(2);

subplot(2,1,1);
plot(timeSim(1000:end), low_bound(1000:end), 'b', 'LineWidth', 1);
hold on
plot(timeSim(1000:end), t_thrust(1000:end), 'y', 'LineWidth', 2);
plot(timeSim(1000:end), high_bound(1000:end), 'r', 'LineWidth', 1);
% plot(timeSim(1000:end), cloop_thrust(1000:end), 'g', 'LineWidth', 1);
% plot(timeSim(1000:end), oloop_thrust(1000:end), 'm', 'LineWidth', 1);
grid on
xlim([0 20]);
ylim([0 700]);
xlabel('Time [s]');
ylabel('Target Thrust [lbf]');
xticks(0:1:20)
title('Thrust Curve');
legend('Low Bound','Target', 'High Bound','Modeled Thrust [CL]','Modeled Thrust [OL]','Location','southeast');
hold off

subplot(2,1,2);
plot(timeSim(1000:end), low_bound(1000:end), 'b', 'LineWidth', 1);
hold on
plot(timeSim(1000:end), t_thrust(1000:end), 'y', 'LineWidth', 2);
plot(timeSim(1000:end), high_bound(1000:end), 'r', 'LineWidth', 1);
plot(timeSim(1000:end), cloop_thrust(1000:end), 'g', 'LineWidth', 1);
% plot(timeSim(1000:end), oloop_thrust(1000:end), 'm', 'LineWidth', 1);
grid on
xlim([0 20]);
ylim([0 700]);
xlabel('Time [s]');
ylabel('Target Thrust [lbf]');
xticks(0:1:20)
title('Expected Response');
legend('Low Bound','Target', 'High Bound','Modeled Thrust [CL]','Modeled Thrust [OL]','Location','southeast');
hold off

sgtitle('Closed Loop: Slope Tracking');

%% Graphs
tspan = [0 10];

% Define state vector and initials
%x = [mdot_ox; mdot_ipa; Pc]
x0 = [0 0 0 0];

% Step angles
angle_ox = 30;
angle_ipa = 30;

% Simulate system
[t, x] = ode45(@(t, x) nonlinear_plant(x, angle_ox, angle_ipa, t), tspan, x0);

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
    mdot_ipa(idx) = valve_angle_to_mdot(i, 820, 322.79, 0.02836);
    mdot_ox(idx) = valve_angle_to_mdot(i, 820, 338.51, 0.04126099537);
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

figure(2);
plot(timeSim, low_bound, 'b', 'LineWidth', 1);
hold on
plot(timeSim, t_thrust, 'y', 'LineWidth', 1);
plot(timeSim, high_bound, 'r', 'LineWidth', 1);
plot(timeSim, cloop_thrust, 'g', 'LineWidth', 1);
plot(timeSim, oloop_thrust, 'm', 'LineWidth', 1);
grid on
xlim([0 17]);
ylim([0 700]);
xlabel('Time [s]');
ylabel('Target Thrust [lbf]');
xticks(0:1:18)
title('Simulated Thrust Curve');
legend('Low Bound', 'Target', 'High Bound', 'Modeled Thrust [CL]','Modeled Thrust [OL]','Location','southeast');
hold off

figure(3);
plot(timeSim, closedMFR, 'g', 'LineWidth', 1);
hold on
plot(timeSim, openMFR, 'r', 'LineWidth', 1);
grid on 
xlim([0 16]);
ylim([0.9 1.7]);
xlabel('Time [s]');
ylabel('Mass Flow Ratio')
title('Simulated Mass Flow Ratio: Open Loop vs. Closed Loop')
legend('Closed Loop', 'Open Loop')
hold off

% FFT on OL Chamber Pressure
fourier = fft(chamber_pressure);
figure(4);
plot(abs(fourier),'r','LineWidth',1);
grid on
xlim([0 1000]);
ylim([0 0.8*10^5]);
title('Fourier Transform for Chamber Pressure')
xlabel('Frequency [Hz]');
ylabel('Amplitude')
