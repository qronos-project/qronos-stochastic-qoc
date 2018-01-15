% Cyclic Average
% [avg, num_repetitions] = cyclic_average(values, len)
% returns:
% - average
%   avg(i) = mean([values(i), values(i+len), values(i+2*len), ...])
% - number of cycles
%   num_repetitions = length(values) / len, rounded down
function [avg, num_repetitions] = cyclic_average(values, len)
num_repetitions = 0;
avg = zeros(len, 1);
% cyclic average of simulation
while (num_repetitions + 1) * len <= length(values)
    avg = avg + values(num_repetitions * len + (1:len));
    num_repetitions = num_repetitions + 1;
end
avg = avg / num_repetitions;
end

