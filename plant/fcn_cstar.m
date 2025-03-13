function cstar_out = fcn_cstar(OF)

    cstar_eff = 0.92;
    cstar_array = [5289.4, 5322, 5352.5, 5381.0, 5407.6, 5432.4, 5455.3, 5476.5, 5496, 5513.9, 5530.3];
    cstar_array = cstar_array * cstar_eff;
    cstar_array = [1.1:0.02:1.3;
                   cstar_array];
    cstar_out = clamped_interpolation(OF, cstar_array);
end

