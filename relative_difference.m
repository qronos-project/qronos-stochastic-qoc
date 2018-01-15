% relative_difference(a, b)
% relative_difference(a, b, epsilon)
% elementwise relative error between a and b
% ignoring numerical errors up to epsilon (which defaults to 10*eps)
function relative_error = relative_difference(a, b, epsilon)
if (nargin == 2)
    epsilon = 10 * eps;
end
relative_error = abs(a - b) ./ min(a, b);
% if both a and b are zero, we take the freedom to define that as zero
% relative error as well, because |a-b| <= relative_error*max(a,b) still
% holds.
relative_error(a == b & b == 0) = 0;

% Comparing values below epsilon does not make any sense, as the result is
% only noise and not the real error.
values_below_threshold = (max(abs(a), abs(b)) < epsilon) & (relative_error > 0);
if any(values_below_threshold)
    warning('relative_difference(): zeroing relative difference for values below numerical threshold')
    relative_error(values_below_threshold) = 0;
end
end