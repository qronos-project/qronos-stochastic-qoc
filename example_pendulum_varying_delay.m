% Example: QoC for time-varying delays.
% The QoC-model is compared to a simulation for three cases:
% 1. Base case: deterministic time-varying delay (Figure in HSCC2018 publication)
% 2. extended to uniformly random delay for y (plot shows the maximum),
%    still deterministic delay for u.
% 3. extended to time-varying disturbance variance H(t)
%
% Cases 2 and 3 use a different time-varying pattern of delay.

%% Setup
worker_init
for variant = 1:3
    randomize = (variant == 2);
    timevarying_disturbance = (variant == 3);
    
    param_pendulum
    param_pendulum_calc
    param_pendulum_timevarying
    plotname = 'pendulum_varying_delay';
    plottitle = 'QoC for pendulum example';
    delay_random_func = [];
    delay_model_random_points = 1;
    if randomize
        % Case 2
        % Random, uniformly distributed delay for y.
        % The distribution is from 0 to delta_t[k], where delta_t[k] is the
        % delay shown in the plot
        % To make it more interesting, we choose a larger maximum delay_y.
        delay_y_normalized(150:199) = .49;
        % We need increased simulation length because the convergence is
        % more problematic with stochastic timing
        % example values for remaining error:
        % 20k runs  approx 4.2%
        % 60k runs  approx 3.2%
        % 120k runs approx 2.5%
        % (slightly depends on the RNG seeds)
        T_sim_length = T_sim_length * 6;
        plotname = 'pendulum_varying_delay_random';
        plottitle = [plottitle, ' (\Delta t_y uniformly random)']; %#ok<AGROW>
        delay_y_sim_is_random = true;
        delay_u_sim_is_random = false;
        delay_model_random_points = 100;
        delay_random_func = @(delay_u_param, delay_y_param, k, z) deal(delay_u_param, delay_y_param * z / delay_model_random_points);
    end
    if timevarying_disturbance
        % Case 3
        delay_u_normalized = 0.15 + 0.02 * sin(pi / 42 * (1:length(delay_u_normalized)));
        delay_y_normalized = 0.05 + 0.02 * sin(pi / 17 * (1:length(delay_u_normalized)));
        delay_u_normalized = round(delay_u_normalized * T / T_sim) / (T / T_sim);
        delay_y_normalized = round(delay_y_normalized * T / T_sim) / (T / T_sim);
        delay_u_normalized = delay_u_normalized';
        delay_y_normalized = delay_y_normalized';
        H = ones(1, 1, length(delay_u_normalized)) * H;
        H(150:end) = 4 * H(1, 1, 1);
        plotname = 'pendulum_varying_delay_varying_disturbance';
        plottitle = [plottitle, ' (H(t) increased to 4*H(0) at 30s)']; %#ok<AGROW>
    end
    if (~randomize && ~timevarying_disturbance)
        % Case 1
        plottitle = [plottitle, ' (Example for HSCC 2018 publication)']; %#ok<AGROW>
    end
    verification_compare_timevarying_qoc
end

%%
worker_finalize