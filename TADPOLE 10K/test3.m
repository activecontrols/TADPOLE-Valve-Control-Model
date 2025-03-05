function thrust = test3(time)

% Closed Loop Frequency Sweep

% Generates a rounded triangle wave using a fourier series approximation
maxT = 550;
freq = 3 / 26;

if time < 4
    thrust = maxT;
elseif time < 17
    f_series = 0;
    for n = 0:1:4
        f_series = f_series + ...
            (-1)^n / (2*n + 1)^2 * sin(2*pi*freq*(2*n + 1)*(time - 1.84));
    end
    f_series = 8 / (pi^2 * 0.9596) * 0.3 * f_series + 0.7;
    thrust = maxT * f_series;
else
    thrust = 0.4*maxT;
end
