% Example for MIMO: Pendulum with redundant input and output
worker_init
% Take the pendulum example and convert it to pseudo-MIMO:
param_pendulum
param_pendulum_calc
param_pendulum_timevarying
% add a second input and output,
% just duplicate and scale the corresponding controller feedback gains,
Bp = [Bp, Bp];
K = [K; K] / 2;
% rescale weighting matrix so that costs are the same as the SISO case
% if both components are equally delayed.
R_tilde = blkdiag(R_tilde, R_tilde) * 2;
u_r = [u_r; u_r] / 2;
% output:
C = [C; C];
% For equal effect, use twice the original noise variance
% (if y is averaged as (y1+y2)/2, the variance is again N_p)
N_p = blkdiag(N_p, N_p) * 2;
L = [L, L] / 2;

% recalculate matrices Ad, Bd, Cd for new controller and observer gains
param_pendulum_calc

% for the delay sequences for u2 and y2, shifted versions of u1 and y1 are
% used
delay_u_normalized = [delay_u_normalized, circshift(delay_u_normalized, 15)];
delay_y_normalized = [delay_y_normalized, circshift(delay_y_normalized, 15)];
plotname = 'mimo_varying_delay';
plottitle = 'QoC example: MIMO varying delay';
delay_random_func = [];
delay_model_random_points = 1;
verification_compare_timevarying_qoc
%%
worker_finalize