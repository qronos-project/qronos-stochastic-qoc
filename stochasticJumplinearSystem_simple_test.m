function stochasticJumplinearSystem_simple_test
% Unittests for stochasticJumplinearSystem

%% simple example: system G(s)=1/(1+s) driven by white noise u(t) with
% E[u(t)u(tau)]=delta(t-tau).
% Output: Y(s) = G(s) U(s).
% What is the variance of y(t) for t -> infinity?

% Compare this with the simulink version (stochasticJumplinearSystem_simple_test_simulink.slx), the output is approx. 0.5 for both.

Ac = -1;
% Bc = 1;
% C = 1;
Q = 1;
G = 1;
H = 1;
P = zeros(size(Ac));
[Pinf, ~] = stochasticJumplinearSystem.continuousUpdate(P, Ac, G, H, 99);
J = stochasticJumplinearSystem.cost(Pinf, Q);
assertElementsAlmostEqual(J, 0.5);
% simulink output converges to 0.5 for long enough simulation time.
% (not auto-tested)


%% More complicated test for t_end=9999

% For t=9999, the calculation is numerically problematic.
% Test that the fallback and warning message work.
lastwarn('')
% suppress warning
warning('off', 'stochasticJumplinearSystemPrivate:computePhiPsiIllConditioned')

% this should cause a warning:
[Pinf, ~] = stochasticJumplinearSystem.continuousUpdate(P, Ac, G, H, 9999);

% re-enable warning
% Matlab has no try...finally - argh!
warning('on', 'stochasticJumplinearSystemPrivate:computePhiPsiIllConditioned')

% Test that warning occured
[~, id] = lastwarn();
assertEqual(id, 'stochasticJumplinearSystemPrivate:computePhiPsiIllConditioned');
lastwarn('');
% Test that fallback worked
J = stochasticJumplinearSystem.cost(Pinf, Q);
assertElementsAlmostEqual(J, 0.5);
