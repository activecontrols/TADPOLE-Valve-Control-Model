function thrust = cp_to_thrust(cp, prevThrust)
    tadpole_AREA_OF_THROAT = 1.69; % in^2

    cf_table = [
        220, 550;
        1.12, 1.3
    ];
  
    c_f = clamped_interpolation(prevThrust, cf_table);
    thrust = cp * c_f * tadpole_AREA_OF_THROAT;
end

