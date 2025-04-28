function cstar_out = fcn_cstar(OF, Pc)

    throttle = Pc / 250;
    cstar_eff_max = 0.71;
    cstar_eff_min = 0.82;
    slope = -(cstar_eff_min - cstar_eff_max) / 0.6;
    cstar_eff = slope * throttle + (cstar_eff_max - slope);
    cstar_eff = min(max(0.5, cstar_eff), 0.9);

    
    cstar_array = [5289.4, 5322, 5352.5, 5381.0, 5407.6, 5432.4, 5455.3, 5476.5, 5496, 5513.9, 5530.3];
    cstar_array = cstar_array * cstar_eff;
    cstar_array = [1.1:0.02:1.3;
                   cstar_array];

    cstar_out = clamped_interpolation(OF, cstar_array);
end

