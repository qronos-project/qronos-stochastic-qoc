% Simple, self-contained example
% Double integrator, Y(s) = (U(s)+D(s))/s^2
% LQR controller with trajectory tracking

worker_init

% The parameters are documented in verification_compare_timevarying_qoc.

% continuous plant
Ap = [0, 1; 0, 0];
Bp = [0; 1];
Gp = [0; 1];
C = [1, 0];


% cost matrices (chosen so that J = 1 stationary with no delays; y and u have roughly equal contribution)
Q_tilde = [0.00968, 0; 0, 0.0470];
R_tilde = 0.768;

% measurement noise
N_p = 1e-2;

% sequence of time-varying delay
delay_u_normalized = [zeros(1, 50), .3 * ones(1, 50), zeros(1, 100)]' + 1e-3;
delay_y_normalized = [zeros(1, 150), .4 * ones(1, 50)]' + 1e-3;

% noise variance
H_const = 1;
% fill into time-varying sequence (which actually does not vary)
H = zeros([size(H_const), size(delay_u_normalized, 1)]);
for i = 1:size(H, 3)
    H(:, :, i) = H_const;
end

% discrete controller
n_d = 2;
T = 1;

% controller design
% discretize the system
sys = ss(Ap, Bp, C, []);
sys_discrete = c2d(sys, T);
lqr_design = true; % change to false for pole placement controller
if lqr_design
    % design controller with LQR
    
    K = -dlqr(sys_discrete.a, sys_discrete.b, Q_tilde, R_tilde);
    
    % discrete-time kalman filter design
    [~, L, ~, ~, ~] = kalmd(sys, H_const, N_p * T, T);
    L = -L;
    
    % this call to kalmd(...) equals:
    % a) exact noise discretization (see appendix "stochastic discretization" of HSCC2018 publication)
    H_discretized = integral(@(tau) expm(Ap * (tau)) * Gp * H_const * Gp' * (expm(Ap * (tau)))', 0, T, 'ArrayValued', true, 'AbsTol', 1e-10, 'RelTol', 1e-8);
    % (or: roughly approximate noise discretization:)
    % assert(all(Bp == Gp));
    % H_discretized_approx = sys_discrete.b * H_const * sys_discrete.b' * T;
    % b) kalman as transposed LQR design
    L_lqr_transposed = -dlqr(sys_discrete.a', sys_discrete.c', H_discretized, N_p)';
    assert(all(all(abs(L_lqr_transposed - L) < 42 * eps)))
else
    % design controller with pole placement
    K = -place(sys_discrete.a, sys_discrete.b, exp([-1, -1.1] * T));
    L = -place(sys_discrete.a', C', exp([-1.2, -1.3] * T))';
end

% trajectory planning based on set reference control input
u_r = zeros(1, length(delay_u_normalized));
u_r(4) = 1;
u_r(5) = - 1;
% u_r(10) = -1;
% u_r(11) = 1;
% calculate reference trajectory
x_r = zeros(length(Ap), length(u_r));
for i = 2:length(u_r)
    x_r(:, i) = sys_discrete.a * x_r(:, i - 1) + sys_discrete.b * u_r(i - 1);
end

% discrete controller
% x_d[k+1] = Ad x_d[k] + B_d y[k] + f_d[k]
% != full-state observer + state feedback
% x_hat[k+1] = A x_hat[k] + L ( C x_hat[k] - y[k] ) + B u[k]
% u[k] = Cd x_d[k] + gd[k]
% != state feedack
% u[k] = K (x_d[k] - x_r[k]) + u_r[k]
Ad = sys_discrete.a + L * C + sys_discrete.b * K;
Bd = -L;
Cd = K;
gd = -K * x_r + u_r;
fd = sys_discrete.b * gd;

% settings for verification by simulation
T_sim = T * 1e-3; % noise discretization -- only for Simulink
T_sim_length = 1e5 * T * length(delay_u_normalized) / simulation_speedup_factor; % Simulink: end time of simulation
% uncomment to disable simulation:
% T_sim_length = 0
% delay randomness in QoC model: disable
delay_model_random_points = 1;
delay_random_func = [];
% delay randomness in simulation: disable
delay_u_sim_is_random = 0;
delay_y_sim_is_random = 0;

plotname = 'example_integrator';
plottitle = 'Double integrator example';

verification_compare_timevarying_qoc
worker_finalize
