function thrust = test2(time)

% Open Loop Frequency Sweep
maxT = 550;
if time < 2
    thrust = 0.8*maxT;
elseif time < 19
    thrust = maxT * (0.15*cos(pi*(time - 2)^2 / 17) + 0.65);
else 
    thrust = maxT*0.5;
end

