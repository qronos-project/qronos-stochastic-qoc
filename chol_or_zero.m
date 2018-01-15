function [R] = chol_or_zero(A)
%CHOL_OR_ZERO Cholesky factorization, extended for the case A=0.
% The result satisfies R'*R=A.

if all(all(A == 0))
    R = zeros(size(A));
else
    R = chol(A);
end
end

