function filteredData = movingavg(rawData)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ENGR 132 
% Program Description 
% This function receives a set of raw data arranged in a column for each
% dataset, and of first column representing time. It then filters the data
% through two steps. It first uses a rolling window median filter to kill
% noise in the data, and then runs it through a post filtering step which
% linearly interpolates in between data drops and fixes them.
%
% Function Call
% filteredData = M1B_sub2_011_24_pplata(rawData)
%
% Input Arguments
% rawData; array of raw data
%
% Output Arguments
% filteredData; the filtered data
%
% Assignment Information
%   Assignment:     M1B, Problem 4
%   Team member:    Pablo Plata, pplata@purdue.edu [repeat for each person]
%   Team ID:        011-24
%   Academic Integrity:
%     [] We worked with one or more peers but our collaboration
%        maintained academic integrity.
%     Peers we worked with: Name, login@purdue [repeat for each]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ____________________
%% INITIALIZATION
% Get data parameters and make a copy
data = rawData;
data(isnan(data)) = 0;
res = size(data,1);
cols = size(data,2);

%% ____________________
%% CALCULATIONS

% Filtering Step
horizon = 4;
avg = zeros(res+1, cols);
filter = zeros(res+1, cols);
for j = 1:1:cols
    first_avg = mean(data(1:horizon/2, j));
    filter(1:horizon/2, j) = first_avg;
end
count = 1;
slopevec = zeros(ceil(horizon/res), cols);
for j = 2:1:cols
    for i = horizon/2:horizon:res
        h_bound = min(i + horizon, res);
        next_bound = min(i + 2*horizon, res);
        current_avg = mean(data(i:h_bound, j), 'omitmissing');
        next_avg = mean(data(h_bound:next_bound, j), 'omitmissing');
        local_slope = (next_avg - current_avg) ...
            / (horizon);
        slopevec(i+horizon/2:h_bound+horizon/2, j) = local_slope;
        local_interp = current_avg + local_slope .* (0:1:horizon).';
        avg(i:h_bound, j) = current_avg;
        limitInterp = i+horizon/2-h_bound+horizon/2;
        filter(i+horizon/2:h_bound+horizon/2, j) = local_interp(1:end-limitInterp);
        count = count + 1;
    end
    count = 1;
end

%% ____________________
%% FORMATTED TEXT/FIGURE DISPLAYS


%% ____________________
%% RESULTS

% Return filtered data
filter(isnan(filter)) = 0;
filteredData = filter;
%% ____________________
%% ACADEMIC INTEGRITY STATEMENT
% We have not used source code obtained from any other unauthorized
% source, either modified or unmodified. Neither have we provided
% access to my code to another. The program we are submitting
% is our own original work.



