function xdot = nonlinear_plant(x, angle_ox, angle_ipa, t)
    mdot_ox = x(1);
    mdot_ipa = x(2);
    Pc = x(3);
    thrust = x(4);

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
    
    ox_manifold_pressure = clamped_interpolation(x(4), ox_manifold_table);
    ipa_manifold_pressure = clamped_interpolation(x(4), ipa_manifold_table);

    
    ox_tank_pressure = 820; % psi
    ipa_tank_pressure = 820; % psi
    ox_density = 0.04126099537; % lbs / in^3
    ipa_density = 0.02836; % lbs / in^3
    
    tadpole_AREA_OF_THROAT = 1.69; % in^2
    tadpole_C_STAR = 4998.0654;    % ft / s
    GRAVITY_FT_S = 32.1740;        % Gravity in (ft / s^2)
    c_f = clamped_interpolation(x(4), cf_table);

    oxError = 1.02 + 0.075*sin(0.42*t);
    ipaError = 0.99 - 0.075*cos(0.85*t);
    chugMagnitude = max(1-(x(4)/550), 0)*0.0375;

    %Sum of a high frequency (chugging) error and low freq. error
    PcError = 1.075 + (chugMagnitude*sin(185*t) + 0.05*cos(1*t));

    f_ox = valve_angle_to_mdot(angle_ox, ox_tank_pressure, ox_manifold_pressure, ox_density)...
        * oxError;
    f_ipa = valve_angle_to_mdot(angle_ipa, ipa_tank_pressure, ipa_manifold_pressure, ipa_density)...
        * ipaError;
    f_Pc = (mdot_ox + mdot_ipa) * tadpole_C_STAR / tadpole_AREA_OF_THROAT / GRAVITY_FT_S...
        * PcError;
    f_thrust = f_Pc * c_f * tadpole_AREA_OF_THROAT;

    % Time constants
    tau_ox = 0.045 / 4;
    tau_ipa = 0.075 / 4;
    tau_pc = 0.015 / 4;

    xdot = zeros(4,1);
    xdot(1) = (-mdot_ox + f_ox) / tau_ox;
    xdot(2) = (-mdot_ipa + f_ipa) / tau_ipa;
    xdot(3) = (-Pc + f_Pc) / tau_pc;
    xdot(4) = (-thrust + f_thrust) / tau_pc;
end

