function xdot = nonlinear_plant(x, angle_ox, angle_ipa, prev_thrust, t)
    mdot_ox = x(1);
    mdot_ipa = x(2);
    Pc = x(3);

    ox_manifold_table = [
        192.5, 210.556, 228.611, 246.667, 264.722, 282.778, 300.833, 318.889, 336.944, 355.0, 373.056, 391.111, 409.167, 427.222, 445.278, 463.333, 481.389, 499.444, 517.5, 535.556, 550.0;
        118.055, 127.34, 136.64, 146.082, 155.44, 165.182, 175.069, 184.896, 194.888, 205.049, 215.378, 225.702, 236.381, 246.93, 257.695, 268.683, 279.647, 290.878, 301.999, 313.482, 322.79
    ];

    ipa_manifold_table = [
        192.5, 210.556, 228.611, 246.667, 264.722, 282.778, 300.833, 318.889, 336.944, 355.0, 373.056, 391.111, 409.167, 427.222, 445.278, 463.333, 481.389, 499.444, 517.5, 535.556, 550.0;
        120.844, 130.535, 140.262, 150.16, 159.99, 170.243, 180.67, 191.054, 201.631, 212.408, 223.382, 234.368, 245.752, 257.017, 268.529, 280.3, 292.062, 304.129, 316.095, 328.466, 338.509
    ];
    
    ox_manifold_pressure = clamped_interpolation(prev_thrust, ox_manifold_table);
    ipa_manifold_pressure = clamped_interpolation(prev_thrust, ipa_manifold_table);

    
    ox_tank_pressure = 820; % psi
    ipa_tank_pressure = 820; % psi
    ox_density = 0.04126099537; % lbs / in^3
    ipa_density = 0.02836; % lbs / in^3

    f_ox = valve_angle_to_mdot(angle_ox, ox_tank_pressure, ox_manifold_pressure, ox_density);
    f_ipa = valve_angle_to_mdot(angle_ipa, ipa_tank_pressure, ipa_manifold_pressure, ipa_density);

    
    tadpole_AREA_OF_THROAT = 1.69; % in^2
    tadpole_C_STAR = 4998.0654;    % ft / s
    GRAVITY_FT_S = 32.1740;        % Gravity in (ft / s^2)


    f_Pc = (mdot_ox + mdot_ipa) * tadpole_C_STAR / tadpole_AREA_OF_THROAT / GRAVITY_FT_S;

    oxError = 1 + 0.15*sin(1/2*t);
    ipaError = 1 + 0.15*sin(1/3*t);
    PcError = 1 + 0.1*sin(1/5*t);

    % Time constants
    tau_mdot = 0.11 / 4;
    tau_pc = 0.75 / 4;

    xdot = zeros(3,1);
    xdot(1) = (-mdot_ox + f_ox * oxError) / tau_mdot;
    xdot(2) = (-mdot_ipa + f_ipa * ipaError) / tau_mdot;
    xdot(3) = (-Pc + f_Pc * PcError) / tau_pc;
end

