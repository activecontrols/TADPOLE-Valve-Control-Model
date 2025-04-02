function xdot = plantWaterflow(x, angle_w, angle_ipa, t)

    %States
    mdot_w = x(1);
    P_out_w = x(2);

    %Data
    water_tank_pressure = 90;   %psi
    water_density = 0.0361;     %lb/in^3

    %% Parameters

    g = 32.174 * 12;                        %in/s
    d = (0.5 - 2*0.065);                    %diameter of line
    A_b = pi * (d / 2)^2;                   %in^2
    l_w = 179;                              %in
    C_fric = 0.03;                          %line friction coef
    l_eq_water = 2;                         %eq line length ox
    
    % DP_l_w = C_fric * l_eq_water / (2 * d * g * water_density * A_b^2);
    DP_l_w = 0;

    % Valve settling times
    tau_valve_w = 0.1;

    % State Derivative
    xdot = zeros(2,1);

    %% ODEs
    P_atm = 14.696;

    xdot(1) = (P_out_w - P_atm - DP_l_w * mdot_w^2) * g * (A_b/l_w);
    xdot(2) = (-P_out_w + valveangle2pout(angle_w, water_tank_pressure, water_density, mdot_w)) / tau_valve_w;

end

