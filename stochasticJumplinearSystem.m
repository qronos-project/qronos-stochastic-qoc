classdef stochasticJumplinearSystem
    % covariance calculations for the stochastic discretization of a
    % jump-linear system
    
    properties
    end
    
    methods(Static)
        function Pnext = discreteUpdate(P, A, N)
            % Pnext = discreteUpdate(P, A, N)
            % discrete jump event
            % x[k+1] = A x[k] + v[k],
            % where
            % covariance(v[k]) = N,
            % covariance(x[k]) = P
            % covariance(x[k+1]) = Pnext.
            %
            % see jump() in [Gaukler et al. 2018]
            Pnext = A * P * A' + N;
        end
        
        function [Pnext, Phi] = continuousUpdate(P, A, G, H, delta_t)
            % [Pnext, Phi] = continuousUpdate(P, A, G, H, delta_t)
            % continuous transition for duration delta_t.
            % d x(t) / dt = A x(t) + G d(t),
            % where
            % autocorrelation(d(t)) = H delta(tau).
            % covariance(x(0)) = P
            % covariance(x(delta_t)) = Pnext
            % Phi is the discretized transition matrix expm(A*delta_t).
            % see elapse() in [Gaukler et al. 2018]
            assert(delta_t >= 0)
            [Phi, Psi] = stochasticJumplinearSystem_private.computePhiPsi(A, G, H, delta_t);
            Pnext = Phi * P * Phi' + Psi;
        end
        
        function J = cost(P, Q)
            % J = cost(P, Q)
            % compute cost J = expectation(x^T Q x)
            % from P = covariance(x).
            J = trace(Q * P);
        end
    end
end

