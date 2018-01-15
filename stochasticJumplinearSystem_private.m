classdef stochasticJumplinearSystem_private
    % internal calculations of stochasticJumplinearSystem,
    % exposed for unit-tests
    
    properties
    end
    
    methods(Static)
        function [Phi, Psi] = computePhiPsi(A, G, H, delta_t)
            % compute:
            % Phi = expm(A * delta_t);
            % Psi = integral(@(tau) expm(A * tau) * G * H * G' * (expm(A * tau))', 0, delta_t, 'ArrayValued', true);
            % in a more efficient way, using the formulas from:
            % Loan, C. V.: Computing integrals involving the matrix
            % exponential -- IEEE Transactions on Automatic Control, 1978,
            % 23, pp. 395-404
            %
            % there: we want Q(\Delta) -- see the unnumbered formulas in the paper just below (1.4)
            % with \Delta=delta_t
            % and s = delta_t - tau (substitution)
            % A=A'
            % Q_c = GHG'
            
            F = expm([-A, G * H * G'; zeros(size(A)), A'] * delta_t);
            n = length(A);
            Psi = F(n + (1:n), n + (1:n))' * F(1:n, n + (1:n));
            % also computes expm(A'*delta_t):
            Phi = F(n + (1:n), n + (1:n))';
            
            % if expm() fails, we need to switch back to the naive version:
            if ~all(all(isfinite(F)))
                [Phi, Psi] = stochasticJumplinearSystem_private.computePhiPsi_naive(A, G, H, delta_t);
                warning('stochasticJumplinearSystemPrivate:computePhiPsiIllConditioned', 'Ill conditioned numerics for matrix exponential - Falling back to integral method for calculating Phi, Psi')
            end
        end
        
        function [Phi, Psi] = computePhiPsi_naive(A, G, H, delta_t)
            % naive and slow version of the above function, for testing
            % and as a fallback for ill-conditioned cases
            Psi = integral(@(tau) expm(A * (tau)) * G * H * G' * (expm(A * (tau)))', 0, delta_t, 'ArrayValued', true, 'AbsTol', 1e-10, 'RelTol', 1e-8);
            Phi = expm(A * delta_t);
        end
    end
    
end

