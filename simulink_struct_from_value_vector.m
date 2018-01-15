function [o] = simulink_struct_from_value_vector(v)
s = size(v);
% Wrap vector of values v[k] into a Simulink struct for use in a "From Workspace" block.
% The timing depends on the sample time set in Simulink.
%
% Higher dimensions are also possible:
%    The last dimension of v is time, other dimensions are the signal:
%    at timestep k, the signal value is v(:, :, ..., :, k)
o = struct();
o.time = [];
o.signals.dimensions = s(1:end - 1);

if length(s) == 2
    % For some unknown reason, a 1D time series has to be transposed
    o.signals.values = v';
else
    o.signals.values = v;
end
end

