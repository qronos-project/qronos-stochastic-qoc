% Global parameters used by all examples
%
% simulation_speedup_factor: Speedup factor: simulation length is shortened
% by this factor (Verification is no longer possible).

% Set to 1000 or 10000 for rough testing if all code runs.
% Set to Inf to disable all simulations.
%
% Long-running scripts will also decrease the number of QoC model data
% points if simulation_speedup_factor is set to a large value.
% Scripts will issue warnings whenever something is omitted due to speedup.

simulation_speedup_factor = 1;
% uncomment to completely disable simulation:
% simulation_speedup_factor = Inf;
% uncomment to almost disable simulation, just see if runs successfully for one cycle:
% simulation_speedup_factor = 10000;

if (simulation_speedup_factor ~= 1)
    warning('Simulation has been shortened by enabling simulation_speedup_factor, at the cost of reduced accuracy (lower number of runs). Use this setting only for fast testing and not for numeric verification.')
end