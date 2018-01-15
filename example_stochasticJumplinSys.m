% This file is a playground for using stochasticJumplinearSystem.
% The resulting P matrix is plotted.
%
% This example is not part of run_all, because the parameters are just
% arbitrarily chosen for demonstration purposes and the results are not
% verified in any way.

clear
close all
% System matrices
% continuous: dx/dt = A x + G n, noise covariance H.
A = [-1, 0; 1, -0.4];
G = [1; 0];
H = 1;

% discrete: x[k+1] = Ajump x[k] + v[k], covariance(v[k])=N.
Ajump = diag([1, 0.95]);
N = 0.01 * eye(2);

% timestep
dt = 1;

N_timesteps = 10;
Pout = zeros(2, 2, N_timesteps);
P = zeros(2, 2);
for k = 1:N_timesteps
    Pout(:, :, k) = P;
    P = stochasticJumplinearSystem.continuousUpdate(P, A, G, H, dt);
    plot_matrix(P);
    pause(0.2)
    P = stochasticJumplinearSystem.discreteUpdate(P, Ajump, N);
    plot_matrix(P);
    pause(0.4)
end