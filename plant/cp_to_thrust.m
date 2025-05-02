function thrust = cp_to_thrust(cp)
    tadpole_AREA_OF_THROAT = 1.69; % in^2

    cf_table = [
        120, 242;
        1.08, 1.347
    ];
  
    c_f = clamped_interpolation(cp, cf_table);
    thrust = cp .* c_f * tadpole_AREA_OF_THROAT;
end

