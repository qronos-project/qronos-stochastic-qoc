% Parameters used in example_pendulum_* and some other examples building
% upon that. See there for usage and see
% verification_compare_timevarying_qoc for a documentation of the variable
% names.
%
% This file is to be called after param_pendulum. Inbetween, values which
% this file reads may be modified, e.g. to modify the plant or controller.

sys = ss(Ap, Bp, C, []);
sys_discrete = c2d(sys, T);

% calculate L,K if not given
if (strcmp(K, 'place'))
    if isnan(lambda_controller)
        lambda_controller = exp(T .* [-10, -11]);
    end
    K = -place(sys_discrete.a, sys_discrete.b, lambda_controller);
else
    assert(isfloat(K))
end

if (strcmp(L, 'place'))
    if isnan(lambda_observer)
        lambda_observer = exp(T .* [-20, -22]);
    end
    L = -place(sys_discrete.a', C', lambda_observer)';
else
    assert(isfloat(L))
end

Ad = sys_discrete.a + L * C + sys_discrete.b * K;
Bd = -L;
Cd = K;
fd = zeros(length(Ap), 1);
gd = zeros(size(Bd, 2), 1);

