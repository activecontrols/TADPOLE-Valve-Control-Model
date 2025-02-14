addpath('valve_control_cpp');
% mex valve_control_wrap.cpp valve_controller.cpp; % convert float to double first

thrust = 220:0.5:550;  % Define range of x values
ox_vals = zeros(size(thrust));  % Preallocate for efficiency
ipa_vals = zeros(size(thrust));

% Compute y1 and y2 for each x
for i = 1:length(thrust)
    [ox_vals(i), ipa_vals(i)] = valve_control_wrap(thrust(i));
end

plot(thrust, ox_vals, 'LineWidth', 1.5, 'DisplayName', 'ox');  % Blue line with circles
hold on;
plot(thrust, ipa_vals, 'LineWidth', 1.5, 'DisplayName', 'ipa');  % Red line with squares
hold off;

% Labels and legend
xlabel('Input Thrust');
ylabel('Output Angle');
legend('show');
grid on;
