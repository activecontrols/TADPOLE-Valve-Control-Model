function xdot = nonlinear_plant2(x, angle_ox, angle_ipa, t)

    %States
    mdot_ox = x(1);
    mdot_ipa = x(2);
    Pc = x(3);
    thrust = x(4);          % Set outside equations
    P_out_ox = x(5);
    P_out_ipa = x(6);

    %Data
    ox_tank_pressure = 550; % psi
    ipa_tank_pressure = 550; % psi
    ox_density = 71.1936; %lb/ft^3
    ipa_density = 49.06838; %lb/ft^3

    %% FOR NOW, PURELY STEADY STATE (NO VALVE)

    A_t = 1.697;                            %in^2
    V_c = 67.3196;                          %in^3
    R_u = 10.7315 * 12^2;                   %psi*ft^3*lb/mol/°R
    R = R_u / 18.8;                         %psi*ft^3*lb/mol/°R 
    T_c = 4885.2;                           %F
    g = 32.174;                             %ft/s
    cstar = 4998;                           %ft/s
    l = 0.165;                              %in
    D = 0.033;                              %in
    A_i = l / (12 * pi * (D/2)^2);          %in^2
    A_b = pi * ((0.5 - 0.065*2) / 2)^2;     %in^2
    l_f = 120;                              %in
    l_o = 240;                              %in
    A_if = 0.04031006898;                   %in^2
    A_io = 0.04875965463;                   %in^2

    E_o = 1 - 2*(A_io / A_b)^2 + 0.03*(l_o / (0.5-0.065 * 2));
    E_f = 1 - 2*(A_if / A_b)^2 + 0.03*(l_f / (0.5-0.065 * 2));

    % Valve settling times
    tau_valve_ox = 0.11 / 4;
    tau_valve_ipa = 0.11 / 4;

    % State Derivative
    xdot = zeros(6,1);
    xdot(1) = (P_out_ox - Pc - E_o / (2*ox_density*g*A_io/12^2) * mdot_ox^2)* g ...
        *(20/A_b + A_i)^(-1);
    xdot(2) = (P_out_ipa - Pc - E_f / (2*ipa_density*g*A_if/12^2) * mdot_ipa^2)* g ...
        *(10/A_b + A_i)^(-1);
    xdot(3) = (R * T_c / V_c)*(mdot_ox + mdot_ipa - A_t*g / cstar * Pc); 
    xdot(4) = 0;
    xdot(5) = (-P_out_ox + valveangle2pout(angle_ox, ox_tank_pressure, ox_density, mdot_ox)) / tau_valve_ox;
    xdot(6) = (-P_out_ipa + valveangle2pout(angle_ipa, ipa_tank_pressure, ipa_density, mdot_ipa)) / tau_valve_ipa;

end

    