
% Computes time-varying QoC by simulation and by the QoC model.
% Compares and plots both.
% All parameters are passed from workspace, see
% example_integrator for a simple example of using this script.
%
% This script is called from example_pendulum_varying_delay and similar
% examples.
%
%
% Parameters (passed as workspace variables):
% Ap, Bp, C, Gp, N_p, n_d, Q_tilde, R_tilde: see controlledSystem()
% T, delay_random_func, delay_model_random_points, Ad, Bd, Cd, fd, gd, H, x_r, u_r:
%    see controlledSystem.timevarying_cost()
% delay_u_normalized, delay_y_normalized: delay for y and u in range
%    T_sim/T < delay < 0.5.
%    TODO: extend this range by generalizing t_cost and the simulation.
% delay_u_sim_is_random, delay_y_sim_is_random:
%    if true, delay_{u,y} is uniformly random in the simulation, with the
%    given value as the maximum of the probability distribution.
% T_sim: simulation timestep
% T_sim_length: simulation end time. Choose M times larger than
%     length(delay_u)*T_sim, so that the cyclic average of M runs is
%     calculated.
% plottitle: title text for figure
% plot_name: filename (prefix) for figure


cs = controlledSystem(Ap, Bp, C, Gp, N_p, n_d);


%% Run simulation

% Note regarding indices of time-varying sequence:
% The sequences are delay_u_normalized(k, :) (same for delay_y) and
% H(:, :, k).
% In the simulation (and in theory), Matlab's k index (which is actually the discrete time k+1)
% maps to the time range from t = (k-1)T - T/2 until t = (k-1)T + T/2.


% Convert delays for u and y to simulation input format:
% Convert to number of simulation timesteps. Accept 1e-4 relative tolerance.
delay_u_sim = round(delay_u_normalized * T / T_sim, 4);
% compensate 'delay of delay' in simulink
% (Simulink needs the delay value two steps earlier)
delay_u_sim = circshift(delay_u_sim, [-2, 0]);
delay_u_sim = simulink_struct_from_value_vector(delay_u_sim');
% same for y
delay_y_sim = round(delay_y_normalized * T / T_sim, 4);
delay_y_sim = circshift(delay_y_sim, [-2, 0]);
delay_y_sim = simulink_struct_from_value_vector(delay_y_sim');

% similar conversion for time-varying noise power
% "H_sqrt_sim[k] = cholesky(H[k-2]) for each k"
H_sqrt_sim = nan(size(H));
for k = 1:size(H, 3)
    delay_compensation_steps = 2;
    k_shifted = mod((k + delay_compensation_steps) - 1, size(H, 3)) + 1;
    H_sqrt_sim(:, :, k) = chol_or_zero(H(:, :, k_shifted));
end
H_sqrt_sim = simulink_struct_from_value_vector(H_sqrt_sim);
if size(gd, 2) == 1
    assert(all(all(gd(:, 1) == 0)), 'gd must be zero if it is only one timestep long')
else
    assert(all(all(gd(:, 1:2) == 0)), 'gd must be zero for timesteps k=1 and k=2 (startup)')
end

% Adjust timing of gd, fd, u_r for simulation
gd_sim = simulink_struct_from_value_vector(circshift(gd, [0, -1]));
x_r_sim = simulink_struct_from_value_vector(x_r);
assert(all(all(u_r(:, 1) == 0)), 'u_r must be zero for timestep k=1')
fd_sim = simulink_struct_from_value_vector(circshift(fd, [0, -1]));
u_r_sim = simulink_struct_from_value_vector(circshift(u_r, [0, 1]));

if T_sim_length > 0
    assert(T_sim_length > T * length(delay_u_normalized), 'T_sim_length is too short for one complete run - this is useless')
    tic
    out = run_sim_once('verification_simulink', struct());
    write_timing_toc_to_file(['plot/time_', plotname, '_simulation.txt'], ['Computation time for simulation for plot ', plotname, ' :']);
    
    J_sim_raw = out.out.get('Jt');
    
    L = length(delay_u_normalized);
    [J_sim, num_runs] = cyclic_average(J_sim_raw, L);
else
    warning('Simulation disabled')
    J_sim = [];
end

%% Run QoC-model

% To match the simulation (and Matlab's crazy 1-indexing), k=1 is t=0.
% Therefore, J(k=1) is evaluated at the initialisation.
k_vec = 1:length(delay_u_normalized);
t = (k_vec - 1) * T;


% In the simulation, the cost is computed just before the nominal start of
% the cycle.
% Therefore, delays must be positive in this implementation, although the
% QoC model does support negative delays.
% TODO: generalize the simulation to arbitrary t_cost, then remove this
% restriction. Currently, the simulation output would look shifted by one
% cycle if delay_u=0 and delay_y is some step sequence.
assert(all(all(delay_u_normalized >= T_sim / T)))
assert(all(all(delay_y_normalized >= T_sim / T)))
t_cost = -eps;
% cs.timevarying_cost(T, delay_u, delay_y, delay_random_func, delay_random_points, Ad, Bd, Cd, fd, gd, H, Q_tilde, R_tilde, t_cost, x_r, u_r)
tic
% As the cost is sampled before u is updated, u_r needs to be delayed by
% one step. This is due to our choice of t_cost=-eps, which lies in the
% middle of the control period and not at the end where u_r has already
% been updated. (See above for the choice of t_cost).
[J_model, ~] = cs.timevarying_cost(T, delay_u_normalized * T, delay_y_normalized * T, delay_random_func, delay_model_random_points, Ad, Bd, Cd, fd, gd, H, Q_tilde, R_tilde, t_cost, x_r, [zeros(size(u_r, 1), 1), u_r(:, 1:end - 1)]);
write_timing_toc_to_file(['plot/time_', plotname, '_model.txt'], ['Computation time for QoC model for plot ', plotname, ' :']);


%% Plot
figure
H_timevarying = any(any(any(diff(H, 1, 3) ~= 0)));

plot_counter = 1;
num_plots = 4 + H_timevarying;

ax = [];
ax(end + 1) = subplot(num_plots, 1, plot_counter);
plot_counter = plot_counter + 1;
title(plottitle)
hold on
grid on
h = stairs(t - T / 2, delay_u_normalized, '-');
set(h, 'DisplayName', 'u')
ylabel('\Deltat_u/T')
ylim([0, max(max([delay_y_normalized, delay_u_normalized])) + 0.04])
plot_delay_y = any(delay_y_normalized ~= 0);
if plot_delay_y
    if ~delay_u_sim_is_random && ~delay_y_sim_is_random
        ylabel('\Deltat/T')
    else
        ylabel('max. \Deltat/T')
    end
    h = stairs(t - T / 2, delay_y_normalized, '-');
    set(h, 'DisplayName', 'y')
    legend show
end

if H_timevarying
    ax(end + 1) = subplot(num_plots, 1, plot_counter);
    plot_counter = plot_counter + 1;
    if ~(size(H, 1) == 1 && size(H, 2) == 1)
        warning('plotting nonscalar H is not yet implemented')
    end
    H_plot = squeeze(H(1, 1, :));
    stairs(t - T / 2, H_plot);
    ylabel('H');
    ylim([0, max(H_plot) * 1.1]);
    grid on
end

ax(end + 1) = subplot(num_plots, 1, plot_counter + [0, 1]);
plot_counter = plot_counter + 2;
hold on
if (~isempty(J_sim))
    h = stairs(t, J_sim, '-');
    set(h, 'LineWidth', 3)
    set(h, 'DisplayName', sprintf('Simulation (averaged over %d runs)', num_runs));
end


h = stairs(t, J_model, '-');
set(h, 'DisplayName', 'QoC-model');
set(h, 'LineWidth', 1)


legend show
ylabel('J')
ylim([0, 1.2 * max([J_model; J_sim])])
grid on

% To be fair, the relative error is computed w.r.t. the smaller of
% both data points: rel. error = |a-b|/min(a,b)   [for a,b>0]
if ~isempty(J_sim)
    assert(all(J_sim >= 0));
    assert(all(J_model >= -10 * eps));
    
    relative_error = relative_difference(J_sim, J_model);
    
    ax(end + 1) = subplot(num_plots, 1, plot_counter);
    plot_counter = plot_counter + 1;
    h = stairs(t, relative_error);
    grid on
    ylim([0, 0.03])
    ylabel('|\DeltaJ_{rel}|')
    xlabel('t')
end
linkaxes(ax, 'x')
xlim([0, round(max(55 / 60 * max(t), max(t) - 5))])
save_plot(plotname, 1 + 0.1 * H_timevarying)


ignore_startup_samples = 0;
if any(any(diff(x_r, 1, 2) ~= 0)) || any(any(diff(u_r, 1, 2) ~= 0))
    warning(['Ignoring first sample of J[k] in the verification, because ', ...
        'the simulation`s reset semantics do not strictly match the model ', ...
        'for nonzero reference signals']);
    % TODO improve the reset handling in the simulation
    ignore_startup_samples = 1;
end
% The last few samples at the end are ignored, because the cyclic reset
% is already kicking in -- In this respect, the simulation is not
% perfectly implemented.
ignore_final_samples = 4;
if T_sim_length < length(delay_u_normalized) * 20e3 * T
    warning('Simulation time too short, not checking error bounds with an assertion')
else
    assert(all(relative_error((ignore_startup_samples + 1):end - ignore_final_samples) < .03), 'relative error is >3%')
end

%% Plot convergence over number of runs
if T_sim_length > 0
    runs = integer_logspace(1, floor(T_sim_length / (T * L) + eps));
    max_error_for_runs = nan(size(runs));
    for i = 1:length(runs)
        relative_error_for_given_number_of_runs = relative_difference(J_model, cyclic_average(J_sim_raw(1:(L * runs(i))), L));
        max_error_for_runs(i) = max(relative_error_for_given_number_of_runs((ignore_startup_samples + 1):end - ignore_final_samples));
    end
    figure
    loglog(runs, max_error_for_runs);
    title(['error convergence for simulation: ', sprintf('\n'), plottitle])
    xlabel('number of runs')
    ylabel('max. |\DeltaJ_{rel}|')
    save_plot([plotname, '-convergence'])
end

%% Plot and verify reference trajectory

if (any(any(x_r ~= 0)) || any(any(u_r ~= 0)))
    % Simulate x and u without noise
    % (must be equal to x_r and u_r, otherwise the feedforward parameters are wrong)
    x_d_noisefree = zeros(length(Ad), length(delay_u_normalized));
    x_p_noisefree = zeros(length(Ap), length(delay_u_normalized));
    u_noisefree = zeros(size(delay_u_normalized'));
    for i = 1:(length(u_r) - 1)
        u_noisefree(:, i) = Cd * x_d_noisefree(:, i) + gd(:, i);
        x_p_noisefree(:, i + 1) = sys_discrete.a * x_p_noisefree(:, i) + sys_discrete.b * u_noisefree(:, i);
        x_d_noisefree(:, i + 1) = Ad * x_d_noisefree(:, i) + Bd * C * x_p_noisefree(:, i) + fd(:, i);
    end
    figure
    num_plots = size(Bp, 2) + length(Ap);
    for i = 1:length(Ap)
        suffix = sprintf('_{,%d}', i);
        subplot(num_plots, 1, i)
        h = stairs(x_d_noisefree(i, :), 'k');
        set(h, 'DisplayName', ['x_d', suffix])
        hold on
        h = stairs(x_p_noisefree(i, :), 'b');
        set(h, 'DisplayName', ['x_p', suffix])
        h = stairs(x_r(i, :), 'r');
        set(h, 'DisplayName', ['x_r', suffix])
        legend show
        if i == 1
            title(['noise-free simulation of reference signal:', sprintf('\n'), plottitle])
        end
    end
    
    for i = 1:size(Bp, 2)
        subplot(num_plots, 1, length(Ap) + i)
        h = stairs(u_noisefree(i, :), 'b');
        set(h, 'DisplayName', 'u')
        hold on
        h = stairs(u_r(i, :), 'r');
        set(h, 'DisplayName', 'u_r')
        legend show
    end
    xlabel('k')
    
    assert(max(max(abs(u_noisefree - u_r))) < 1e-10, 'Reference trajectory u_r does not match plant model with controller')
    assert(max(max(abs(x_p_noisefree - x_r))) < 1e-10, 'Reference trajectory x_r does not match plant model with controller')
end
