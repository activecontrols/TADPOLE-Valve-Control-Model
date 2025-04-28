function [valve_ox, valve_ipa] = mdot2valves(mdot_ox, mdot_ipa, chamber_pressure)
    
    %Tables and data
    ox_manifold_table = [
        192.5, 210.556, 228.611, 246.667, 264.722, 282.778, 300.833, 318.889, 336.944, 355.0, 373.056, 391.111, 409.167, 427.222, 445.278, 463.333, 481.389, 499.444, 517.5, 535.556, 550.0;
        118.055, 127.34, 136.64, 146.082, 155.44, 165.182, 175.069, 184.896, 194.888, 205.049, 215.378, 225.702, 236.381, 246.93, 257.695, 268.683, 279.647, 290.878, 301.999, 313.482, 322.79
    ];

    ipa_manifold_table = [
        192.5, 210.556, 228.611, 246.667, 264.722, 282.778, 300.833, 318.889, 336.944, 355.0, 373.056, 391.111, 409.167, 427.222, 445.278, 463.333, 481.389, 499.444, 517.5, 535.556, 550.0;
        120.844, 130.535, 140.262, 150.16, 159.99, 170.243, 180.67, 191.054, 201.631, 212.408, 223.382, 234.368, 245.752, 257.017, 268.529, 280.3, 292.062, 304.129, 316.095, 328.466, 338.509
    ];

    % Cv mapping 
    alpha_OX = 2.50;
    beta_OX = 58;
    gamma_OX = 11;

    alpha_IPA = 2.95;
    beta_IPA = 63;
    gamma_IPA = 10;

    % Max commands
    mdot_ox = max(mdot_ox, 0.1);
    mdot_ipa = max(mdot_ipa, 0.1);

    % Convert
    OF_RATIO = 1.2;              % target OF

    %Manifold pressures
    %ox_manifold_pressure = clamped_interpolation(thrust, ox_manifold_table);
    %ipa_manifold_pressure = clamped_interpolation(thrust, ipa_manifold_table);

    %Fuel Data
    ox_tank_pressure = 600; % psi
    ipa_tank_pressure = 700; % psi
    ox_density = 0.04126099537; % lbs / in^3
    ipa_density = 49.06838 / 1728; % lbs / in^3
    h2o_density = 0.0361; %lbs / in^3
    g = 32.174 * 12;
    C_do = 0.35;
    C_di = 0.72;
    A_if = 0.04031006898;                   %in^2
    A_io = 0.04875965463;                   %in^2

    % Injector pressure drop
    DP_io = 1 / (2 * ox_density * g *(C_do * A_io)^2);
    DP_if = 1 / (2 * ipa_density * g *(C_di * A_if)^2);

    %Calculations OX
    pressure_delta = ox_tank_pressure - chamber_pressure - DP_io * mdot_ox^2;
    pressure_delta = max(0, pressure_delta); % block negative under sqrt and divide by 0
    cvOX = 60/231 * mdot_ox .* sqrt(1 ./ (ox_density * h2o_density * (pressure_delta)));

    %Calculations IPA
    pressure_delta = ipa_tank_pressure - chamber_pressure - DP_if * mdot_ipa^2;
    pressure_delta = max(0, pressure_delta); % block negative under sqrt and divide by 0
    cvIPA = 60/231 * mdot_ipa .* sqrt(1 ./ (ipa_density * h2o_density * (pressure_delta)));

    %Angles
    valve_ox = max(-gamma_OX * log(alpha_OX/ cvOX + 1) + beta_OX, 15);
    valve_ipa = max(-gamma_IPA * log(alpha_IPA / cvIPA + 1) + beta_IPA, 15);
    
end