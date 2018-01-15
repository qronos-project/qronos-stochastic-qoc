function [y] = integer_logspace(a, b)
% y = integer_logspace(a,b) returns ca. 50 log-spaced points, where
%   y(i) is integer,
%   y(1) == a,
%   y(end) == b, and
%   y(i+1) > y(i).
assert(a == floor(a), 'a must be integer')
assert(b == floor(b), 'b must be integer')
y = unique([a, round(logspace(log10(a), log10(b))), b]);
end

