function stochasticJumplinearSystem_private_test
% Unittests for stochasticJumplinearSystem_private

% test computePhiPsi()

reltol = 1e-8;
A = [1, 1 + 1e-10; 42, 42 + 1e-7];
assertElementsAlmostEqual(A, A .* (1 + (2 * rand(size(A)) - 1) * reltol))

A = rand(7, 7);
G = rand(7, 3);
H = rand(3, 3);
delta_t = .42;
checkComputePhiPsi(A, G, H, delta_t)

A = zeros(7);
A(1, 1) = 1;
A(2, 1) = 1337;
G = eye(7);
H = 42;
delta_t = 4;
checkComputePhiPsi(A, G, H, delta_t)

% this causes a warning because it is numerically problematic:
A = -1;
G = 1;
H = 1;
delta_t = 999;
lastwarn('')
% suppress warning
warning('off', 'stochasticJumplinearSystemPrivate:computePhiPsiIllConditioned')

% this should cause a warning:
checkComputePhiPsi(A, G, H, delta_t);

% re-enable warning
% Matlab has no try...finally - argh!
warning('on', 'stochasticJumplinearSystemPrivate:computePhiPsiIllConditioned')

% Test that warning occured
[~, id] = lastwarn();
assertEqual(id, 'stochasticJumplinearSystemPrivate:computePhiPsiIllConditioned');
lastwarn('');
end

function checkComputePhiPsi(A, G, H, delta_t)
[Phi1, Psi1] = stochasticJumplinearSystem_private.computePhiPsi(A, G, H, delta_t);
[Phi2, Psi2] = stochasticJumplinearSystem_private.computePhiPsi_naive(A, G, H, delta_t);
assertElementsAlmostEqual(Phi1, Phi2)
assertElementsAlmostEqual(Psi1, Psi2)
end