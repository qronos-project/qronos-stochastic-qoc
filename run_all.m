% Run all unit-tests and examples. Plots are opened as Matlab figures and
% also saved into subfolders of plot-archive/ together with the
% corresponding workspace files. Due to the stochastic simulations, the
% examples may take a few days(!).
%
% For faster execution (at the cost of decreased simulation accuracy, and
% therefore higher relative error between simulation and model), increase
% simulation_speedup_factor in param_global.m .

clear all
close all
datetime
disp('Running unittests')
worker_init
assert(runxunit(), 'Unittests failed')
disp('Running many examples. This may take a few days.')

% This one produces the figure from the HSCC2018 publication (and further figures):
% The figure from the paper is the very first figure window that opens.
example_pendulum_varying_delay

% extension to MIMO
example_mimo

% simple, self-contained example
example_integrator

% static QoC calculation
example_pendulum_static_delay

disp('All tests and examples completed successfully')
datetime