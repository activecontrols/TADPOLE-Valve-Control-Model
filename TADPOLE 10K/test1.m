function thrust = test1(time)

% OPEN LOOP, TURN OFF PID CONTROL
maxT = 550;
if time < 4
    thrust = 1*maxT;
elseif time < 6
    thrust = 0.8*maxT;
elseif time < 8
    thrust = 0.9*maxT;
elseif time < 10
    thrust = 0.7*maxT;
elseif time < 12
    thrust = 0.5*maxT;
elseif time < 14
    thrust = 0.7*maxT;
elseif time < 16
    thrust = 0.6*maxT;
elseif time < 20
    thrust = 0.8*maxT;
else 
    thrust = 0.8*maxT;
end