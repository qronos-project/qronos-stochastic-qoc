% Example: Stationary QoC for inverse pendulum, static delay of either y or u.
% The QoC model is verified by two Simulink simulations:
% - verification_simulink_simplified.slx:
%   A simple one for this case (also limited to delay_y <= 0 and
%   delay_u >=0),
% - verification_simulink.slx:
%   the general one also used in other examples for varying delays.

worker_init
% Setup parameters
param_pendulum
T_sim_length = 1e5 / simulation_speedup_factor;
% Note regarding simulation time:
% vericiation_simulink_simplified requires a slightly longer simulation time to converge within the same error bound as verification_simulink.
% verification_simulink already converges to less than 3% error within T_sim_length=1e4.
% This is probably due to chance (different randomness due to different noise implementation in the simulations).
% TODO: In-depth analysis of convergence and influence of RNG algorithm and seed.
param_pendulum_calc

% sweep over this range of relative delay:
num_parameter_points = 11;
if simulation_speedup_factor > 1e3
    warning('Reducing number of QoC-model points because simulation_speedup_factor is large')
    num_parameter_points = 5;
end
parameter_vec = linspace(-.45, .45, num_parameter_points);

% round delay to nearest integer multiples of simulation stepsize
parameter_vec = round(parameter_vec * T / T_sim) * T_sim / T;
parameter_vec = unique(parameter_vec);

% enable verification by Simulink simulation
enable_simulation = T_sim_length > 0;

% initialize output
Jstat = nan(size(parameter_vec));
Jsim = Jstat;
Jsim_simplified = Jstat;

% delay either y or u:
for delay_var = {'y', 'u'}
    % Matlab is stupid -- unpack string out of 1x1 cell array delay_var
    delay_var = delay_var{1}; %#ok<FXSET>
    for l = 1:length(parameter_vec)
        J = 0;
        if strcmp(delay_var, 'u')
            delay_u = parameter_vec(l) * T;
            delay_y = 0;
        else
            delay_u = 0;
            delay_y = parameter_vec(l) * T;
        end
        % compute stationary QoC
        cs = controlledSystem(Ap, Bp, C, Gp, N_p, n_d);
        Jstat(l) = cs.static_cost(T, delay_u, delay_y, Ad, Bd, Cd, fd, gd, H, Q_tilde, R_tilde);
        
        % setup parameters for Simulink simulation
        % (see example_pendulum_varying_delay.m)
        % additional code is necessary for timevarying delays.
        % The first two cycles will not be delayed, but that doesn't
        % matter, as we are interested only in the stationary cost.
        % values should be integer -- round with 1e-4 tolerance to ignore
        % minimal floating point calculation errors.
        delay_u_sim = simulink_struct_from_value_vector(round(delay_u / T_sim, 4));
        delay_u_sim_is_random = false;
        delay_y_sim = simulink_struct_from_value_vector(round(delay_y / T_sim, 4));
        delay_y_sim_is_random = false;
        
        % Run Simulink simulations
        skip_simulation_for_this_point = false;
        if simulation_speedup_factor > 1000
            warning('Skipping simulation for most parameter points because simulation_speedup_factor is enabled')
            % only simulate 3 points
            skip_simulation_for_this_point = mod(l, round(length(parameter_vec) / 3)) ~= 0;
        end
        
        if ~enable_simulation
            warning('Simulation is disabled')
        elseif ~skip_simulation_for_this_point
            H_sqrt_sim = simulink_struct_from_value_vector(chol(H(:, :, 1)));
            % x_r, gd, u_r, fd  are zero
            x_r_sim = simulink_struct_from_value_vector(zeros(length(Ap), 1));
            u_r_sim = simulink_struct_from_value_vector(zeros(size(Bp, 2), 1));
            gd_sim = u_r_sim;
            fd_sim = x_r_sim;
            out = run_sim_once('verification_simulink', struct());
            Jsim(l) = out.J;
            Jstat(l)
            out.J
            if delay_u >= 0 && delay_y <= 0
                out = run_sim_once('verification_simulink_simplified', struct());
                Jsim_simplified(l) = out.J;
                out.J
            else
                % the simplified simulation cannot handle this case
                Jsim_simplified(l) = NaN;
            end
            disp(' ')
        end
    end
    write_timing_toc_to_file('plot/time_example_pendulum_static_delay.txt');
    
    % Plot
    figure
    h = plot(parameter_vec, Jstat, '+-');
    set(h, 'DisplayName', 'QoC-model')
    hold on
    h = plot(parameter_vec, Jsim, 'o-');
    set(h, 'DisplayName', 'Simulink')
    h = plot(parameter_vec, Jsim_simplified, '.-');
    set(h, 'DisplayName', 'Simulink (simplified)')
    xlabel(sprintf('\\Delta t_%s / T', delay_var))
    ylabel('J')
    ylim([0, 5])
    % BUG in Matlab 2015a (and possibly other versions): values much larger
    % than the y-limit are plotted wrong. example:
    % figure; plot([1e17 1 2]); ylim([0 5])
    % looks more like [-inf 1 2] (wrong sign of infinity!)
    title(sprintf('Cost for static delay of %s[k]', delay_var))
    legend('show')
    plotname = strcat('example_pendulum_static_delay_', delay_var);
    save_plot(plotname);
    
    if ~enable_simulation
        warning('Values not verified by simulation');
    elseif simulation_speedup_factor > 1
        warning('Cannot verify values by simulation, as simulation_speedup_factor is enabled')
    else
        % Test that values are plausible and match
        assert(sum(Jsim < 3) > length(parameter_vec) / 4, 'too few valid cost points')
        assert(sum(Jsim > 0.1) > length(parameter_vec) / 2, 'too few valid cost points')
        for l = 1:length(Jsim)
            assertAlmostEqualOrBothLarge(Jsim(l), Jstat(l), 5, 0.05);
            % tighter tolerance for smaller J values
            assertAlmostEqualOrBothLarge(Jsim(l), Jstat(l), 2, 0.03);
            % compare with simplified simulation if applicable
            if ~isnan(Jsim_simplified(l))
                assertAlmostEqualOrBothLarge(Jsim_simplified(l), Jstat(l), 5, 0.05);
                assertAlmostEqualOrBothLarge(Jsim_simplified(l), Jstat(l), 2, 0.03);
            end
        end
    end
end
%%
worker_finalize