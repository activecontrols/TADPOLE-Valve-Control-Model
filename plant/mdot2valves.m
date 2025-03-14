function [valve_ox, valve_ipa] = mdot2valves(mdot_tot, thrust)
    
    %Tables and data
    ox_manifold_table = [
        192.5, 210.556, 228.611, 246.667, 264.722, 282.778, 300.833, 318.889, 336.944, 355.0, 373.056, 391.111, 409.167, 427.222, 445.278, 463.333, 481.389, 499.444, 517.5, 535.556, 550.0;
        118.055, 127.34, 136.64, 146.082, 155.44, 165.182, 175.069, 184.896, 194.888, 205.049, 215.378, 225.702, 236.381, 246.93, 257.695, 268.683, 279.647, 290.878, 301.999, 313.482, 322.79
    ];

    ipa_manifold_table = [
        192.5, 210.556, 228.611, 246.667, 264.722, 282.778, 300.833, 318.889, 336.944, 355.0, 373.056, 391.111, 409.167, 427.222, 445.278, 463.333, 481.389, 499.444, 517.5, 535.556, 550.0;
        120.844, 130.535, 140.262, 150.16, 159.99, 170.243, 180.67, 191.054, 201.631, 212.408, 223.382, 234.368, 245.752, 257.017, 268.529, 280.3, 292.062, 304.129, 316.095, 328.466, 338.509
    ];

    cv_table = [
        0.000, 0.070, 0.161, 0.378, 0.670, 1.000, 1.450, 2.050, 2.780, 3.710, 4.960;
        0, 9, 18, 27, 36, 45, 54, 63, 72, 81, 90
    ];

    IN3_TO_GAL = 0.004329;       % convert cubic inches to gallons
    PER_SEC_TO_PER_MIN = 60;     % convert per second to per minute
    LB_TO_TON = 0.000453592;     % convert lb to metric tons
    PER_IN3_TO_PER_M3 = 61023.7; % convert per in^3 to per m^3
    OF_RATIO = 1.2;              % target OF

    %Manifold pressures
    ox_manifold_pressure = clamped_interpolation(thrust, ox_manifold_table);
    ipa_manifold_pressure = clamped_interpolation(thrust, ipa_manifold_table);

    %Fuel Data
    ox_tank_pressure = 820; % psi
    ipa_tank_pressure = 820; % psi
    ox_density = 0.04126099537; % lbs / in^3
    ipa_density = 0.02836; % lbs / in^3

    %Mass Balance
    mdot_ox = mdot_tot / (1 + OF_RATIO) * OF_RATIO;
    mdot_ipa = mdot_tot / (1 + OF_RATIO);
    
    %Calculations OX
    pressure_delta = ox_tank_pressure - ox_manifold_pressure;
    pressure_delta = max(0, pressure_delta); % block negative under sqrt and divide by 0
    cvOX = mdot_ox * IN3_TO_GAL * PER_SEC_TO_PER_MIN * sqrt(LB_TO_TON * PER_IN3_TO_PER_M3 / (ox_density * pressure_delta));

    %Calculations IPA
    pressure_delta = ipa_tank_pressure - ipa_manifold_pressure;
    pressure_delta = max(0, pressure_delta); % block negative under sqrt and divide by 0
    cvIPA = mdot_ipa * IN3_TO_GAL * PER_SEC_TO_PER_MIN * sqrt(LB_TO_TON * PER_IN3_TO_PER_M3 / (ipa_density * pressure_delta));

    %Angles
    valve_ox = clamped_interpolation(cvOX, cv_table);
    valve_ipa = clamped_interpolation(cvIPA, cv_table);
    
end