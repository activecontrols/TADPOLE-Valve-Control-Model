function xdot = nonlinear_plant2(x, angle_ox, angle_ipa, t)

    %States
    mdot_ox = x(1);
    mdot_ipa = x(2);
    Pc = x(3);
    thrust = x(4);          % Set outside equations
    P_out_ox = x(5);
    P_out_ipa = x(6);
    distMode = 1;           % 1 for disturbances, 0 for none

    %Data
    ox_tank_pressure = 820 + distMode * 25*sin(4*t); % psi, with disturbance
    ipa_tank_pressure = 820 + distMode * 25*sin(3*t); % psi, with disturbance
    ox_density = 71.1936 / 1728; %lb/in^3
    ipa_density = 49.06838 / 1728; %lb/in^3

    %% Parameters

    A_t = 1.697;                            %in^2
    V_c = 67.3196;                          %in^3
    R_u = 10.7315 * 12^2;                   %psi*ft^3*lb/mol/°R
    R = R_u / 18.8;                         %psi*ft^3*lb/mol/°R 
    T_c = tempfcn(mdot_ox / mdot_ipa);      %Rankine
    g = 32.174 * 12;                        %in/s
    cstar = fcn_cstar(mdot_ox / mdot_ipa);  %ft/s
    l = 0.165;                              %in
    D = 0.033;                              %in
    A_i = l / (12 * pi * (D/2)^2);          %in^2
    d = (0.5 - 2*0.065);                    %diameter of line
    A_b = pi * (d / 2)^2;                   %in^2
    l_f = 120;                              %in
    l_o = 240;                              %in
    A_if = 0.04031006898;                   %in^2
    A_io = 0.04875965463;                   %in^2
    C_fric = 0.03;                          %line friction coef
    C_d = 0.7;                              %injector coef
    l_eq_ox = 7;                            %eq line length ox
    l_eq_ipa = 7;                           %eq line length ipa
    
    DP_i_ox = 1 / (2 * ox_density * g *(C_d * A_io)^2);
    DP_l_ox = C_fric * l_eq_ox / (2 * d * g * ox_density * A_b^2);

    DP_i_ipa = 1 / (2 * ipa_density * g *(C_d * A_if)^2);
    DP_l_ipa = C_fric * l_eq_ipa / (2 * d * g * ipa_density * A_b^2);

    % Valve settling times
    tau_valve_ox = 0.1;
    tau_valve_ipa = 0.1;

    % State Derivative
    xdot = zeros(6,1);

    %% ODEs
    xdot(1) = (P_out_ox - Pc - (DP_i_ox + DP_l_ox) * mdot_ox^2) * g * (A_b/l_o);
    xdot(2) = (P_out_ipa - Pc - (DP_i_ipa + DP_l_ipa) * mdot_ipa^2) * g * (A_b/l_f);
    xdot(3) = (R * T_c / V_c)*(mdot_ox + mdot_ipa - A_t*(g /12) / cstar * Pc)*(1 + 0.04*distMode*sin(0.5*t)); 
    xdot(4) = 0;
    xdot(5) = (-P_out_ox + valveangle2pout(angle_ox, ox_tank_pressure, ox_density, mdot_ox)) / tau_valve_ox;
    xdot(6) = (-P_out_ipa + valveangle2pout(angle_ipa, ipa_tank_pressure, ipa_density, mdot_ipa)) / tau_valve_ipa;

end

    