function out = clamped_interpolation(value, table)
    value = max(min(value, table(1,end)), table(1,1));
    if value < table(1,1)
        out = table(2,1);
    elseif value > table(1, end)
        out = table(2, end);
    else
        out = interp1(table(1,:), table(2,:), value);
    end
end