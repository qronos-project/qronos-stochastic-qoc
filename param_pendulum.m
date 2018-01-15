% Parameters used in example_pendulum_* and some other examples building
% upon that. See there for usage and see
% verification_compare_timevarying_qoc for a documentation of the variable
% names.
%
% This file contains the basic parameters which may be modified by param_pendulum_*.

% linearized inv. pendulum
xi = 0.5;
omega0 = pi;

upright = +1;
Ap = [0, 1; upright * omega0 ^ 2, -2 * xi * omega0];
Bp = [0; omega0 / 9.81];
Gp = [0; 1];
C = [1, 0];
H = 1e-3;
N_p = 1e-6;

% observer-based discrete-time state feedback
n_d = 2;

T = 0.2;
T_sim = 1e-4; % noise discretization -- only for Simulink
T_sim_length = max(61, 1e4 / simulation_speedup_factor); % Simulink: end time of simulation

% Weighting factors are chosen so that u and x have approximately the same
% cost influence when delays are zero.
Q_tilde = 550 * eye(2);
R_tilde = 0.8;
x_r = [0; 0];
u_r = 0;

K = 'place';
L = 'place';
lambda_controller = NaN;
lambda_observer = NaN;