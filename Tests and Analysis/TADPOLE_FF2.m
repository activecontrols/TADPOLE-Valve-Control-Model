%Values
mdot_ox = 1.5;
A_b = pi * ((0.5 - 0.065*2) / 2)^2;     %in^2
l_f = 120;                              %in
l_o = 240;                              %in
A_if = 0.04031006898;                   %in^2
A_io = 0.04875965463;                   %in^2

E_o = 1 - 2*(A_io / A_b)^2 + 0.03*(l_o / (0.5-0.065 * 2));
E_f = 1 - 2*(A_if / A_b)^2 + 0.03*(l_f / (0.5-0.065 * 2));

cv_table = [
        0.000, 0.070, 0.161, 0.378, 0.670, 1.000, 1.450, 2.050, 2.780, 3.710, 4.960;
    0, 9, 18, 27, 36, 45, 54, 63, 72, 81, 90
];

IN3_TO_GAL = 0.004329;       % convert cubic inches to gallons
PER_SEC_TO_PER_MIN = 60;     % convert per second to per minute
LB_TO_TON = 0.000453592;     % convert lb to metric tons
PER_IN3_TO_PER_M3 = 61023.7; % convert per in^3 to per m^3
PER_IN3_TO_PER_FT3 = 1728;   % convert per in^3 to per ft^3
density = 0.0412;            % lb/in^3   

P_tank = 550;                % psi  
Pc = 250;                    % psi
P_manifold = 338;            % psi

cv1 = IN3_TO_GAL * PER_SEC_TO_PER_MIN * mdot_ox ...
    * sqrt(LB_TO_TON * PER_IN3_TO_PER_M3 / (density * (P_tank - Pc) -...
    (E_o * mdot_ox^2 / (2 * 32.174 * PER_IN3_TO_PER_FT3 * A_io^2))));

cv2 = IN3_TO_GAL * PER_SEC_TO_PER_MIN * mdot_ox ...
    * sqrt(LB_TO_TON * PER_IN3_TO_PER_M3 / (density * (P_tank - P_manifold)));

OX_NEW = clamped_interpolation(cv1, cv_table);
OX_OLD = clamped_interpolation(cv2, cv_table);