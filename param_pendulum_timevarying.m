% Parameter file used in example_pendulum_varying_delay:
% Enable deterministic time-varying delay.
% To be called after param_pendulum.

clear delay_y delay_u

% Normalized delays (delta_t/T) of u and y:
% delay_u_normalized(k)  in range (-0.5, +0.5)
%   [Note: currently only the range [5e-4, .5) is supported by the
%   simulation; you will get an assertion error if you try something
%   outside]
% k = 1 ... L
% Acuation, i.e., the update of of u(t)
% occurs at t = (k - 1) * T + delay_u_timeseries(k) * T,
% where (k - 1) * T is the nominal actuation time-point.
% The timing for y is defined in the same way.
delay_u_normalized = ones(300, 1) * .1;
delay_u_normalized(1:49) = .01;
delay_u_normalized(50:99) = .2;
delay_y_normalized = ones(300, 1) * .01;
delay_y_normalized(150:199) = .3;
% run long enough to average over x runs
simulation_runs = 30e3;
if simulation_speedup_factor == Inf
    T_sim_length = 0;
else
    T_sim_length = max(61, length(delay_u_normalized) * simulation_runs * T / simulation_speedup_factor);
end
% set to zero to disable simulation:
% T_sim_length = 0;

delay_u_sim_is_random = false;
delay_y_sim_is_random = false;
