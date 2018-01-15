% QoC computations for a closed loop consisting of continuous-time plant,
% discrete-time controller and sample-hold units (representing
% analog-to-digital and vice versa) with varying timing. Builds upon
% stochasticJumplinearSystem.
%
% See [Gaukler et al. 2018] for a reference of the variable names.
classdef controlledSystem
    properties
        n_c
        n_d
        n_y
        n_u
        Ap
        Bp
        C
        Gp
        N_p
    end
    methods
        function self = controlledSystem(Ap, Bp, C, Gp, N_p, n_d)
            self.Ap = Ap;
            self.n_c = length(Ap);
            self.Bp = Bp;
            s = size(Bp);
            self.n_u = s(2);
            self.C = C;
            s = size(C);
            self.n_y = s(1);
            self.Gp = Gp;
            self.N_p = N_p;
            self.n_d = n_d;
        end
        
        % start covariance
        function P0 = P0(self)
            P0 = zeros(self.n_c + self.n_d + self.n_u + self.n_y + 1);
            P0(end, end) = 1;
        end
        
        % [Pnext, A] = elapse(P, delta_t, H)
        % elapse time
        % arguments:
        %  P: state covariance matrix
        %  delta_t: elapsed time
        %  H: continuous noise covariance
        % returns:
        %  Pnext = new state covariance matrix
        %  A = state transition matrix
        function [Pnext, A] = elapse(self, P, delta_t, H)
            A = zeros(length(P));
            A(1:self.n_c, 1:self.n_c) = self.Ap;
            A(1:self.n_c, self.n_c + self.n_d + self.n_y + (1:self.n_u)) = self.Bp;
            G = zeros(length(P), length(H));
            G(1:self.n_c, :) = self.Gp;
            [Pnext, A] = stochasticJumplinearSystem.continuousUpdate(P, A, G, H, delta_t);
        end
        
        % [Pnext, A] = sample(P, j)
        % 'sample' event
        % arguments:
        %  P: state covariance matrix
        %  j: sensor index
        % returns:
        %  Pnext = new state covariance matrix
        %  A = state transition matrix
        function [Pnext, A] = sample(self, P, j)
            assert(isdiag(self.N_p));
            A_sample = eye(length(P));
            % Selector matrix e_j e_j^T,  where e_j is the j-th unity
            % vector
            selector = zeros(self.n_y);
            selector(j, j) = 1;
            A_sample(self.n_c + self.n_d + (1:self.n_y), 1:self.n_c) = selector * self.C;
            A_sample(self.n_c + self.n_d + (1:self.n_y), self.n_c + self.n_d + (1:self.n_y)) = eye(self.n_y) - selector;
            % Note:
            % The equivalent calculation for tmp = e_j e_j^T N_p e_j e_j^T  is:
            % tmp=zeros(n); tmp(j,j)=N_p(j,j);
            N_sample = zeros(length(P));
            N_sample(self.n_c + self.n_d + j, self.n_c + self.n_d + j) = self.N_p(j, j);
            
            Pnext = stochasticJumplinearSystem.discreteUpdate(P, A_sample, N_sample);
            A = A_sample;
        end
        
        % [Pnext, A] = compute(P, Ad_k, Bd_k, fd_k)
        % 'compute' event
        % arguments:
        %  P: state covariance matrix
        %  Ad_k, Bd_k, fd_k: controller transition matrices/vectors.
        %  gd_k is assumed zero.
        % returns:
        %  Pnext = new state covariance matrix
        %  A = state transition matrix
        function [Pnext, A] = compute(self, P, Ad_k, Bd_k, fd_k)
            assert(all(size(fd_k) == [self.n_d, 1]), 'fd_k has wrong dimension')
            A_compute = eye(length(P));
            A_compute(self.n_c + (1:self.n_d), :) = [zeros(self.n_d, self.n_c), Ad_k, Bd_k, zeros(self.n_d, self.n_u), fd_k];
            Pnext = stochasticJumplinearSystem.discreteUpdate(P, A_compute, zeros(length(P)));
            A = A_compute;
        end
        
        % [Pnext, A] = actuate(P, j, Cd_k, gd_k)
        % 'actuate' event
        % arguments:
        %  P: state covariance matrix
        %  j: actuator index
        %  Cd_k, gd_k: controller transition matrices/vectors.
        % returns:
        %  Pnext = new state covariance matrix
        %  A = state transition matrix
        function [Pnext, A] = actuate(self, P, j, Cd_k, gd_k)
            A_actuate = eye(length(P));
            % Selector matrix e_j e_j^T,  where e_j is the j-th unity
            % vector
            selector = zeros(self.n_u);
            selector(j, j) = 1;
            A_actuate(self.n_c + self.n_d + self.n_y + (1:self.n_u), :) = [zeros(self.n_u, self.n_c), selector * Cd_k, zeros(self.n_u, self.n_y), eye(self.n_u) - selector, selector * gd_k];
            Pnext = stochasticJumplinearSystem.discreteUpdate(P, A_actuate, zeros(length(P)));
            A = A_actuate;
        end
        
        % pseudo-event which changes nothing
        % parameters: see elapse()
        function [Pnext, A] = noop(~, P)
            A = eye(length(P));
            Pnext = P;
        end
        
        % J = cost(P, Q_tilde, R_tilde, x_r, u_r)
        % compute cost (QoC) from P, Q_tilde, R_tilde
        % assuming x_r(t) == 0
        function J = cost(self, P, Q_tilde, R_tilde, x_r, u_r)
            assert(all(size(Q_tilde) == [self.n_c, self.n_c]))
            assert(all(size(R_tilde) == [self.n_u, self.n_u]))
            assert(all(all(R_tilde == R_tilde')))
            assert(all(all(Q_tilde == Q_tilde')))
            % x_r and u_r may be omitted together.
            assert(nargin == 4 || nargin == 6)
            if nargin == 6
                assert(all(size(x_r) == [length(self.Ap), 1]))
                assert(all(size(u_r) == [size(self.Bp, 2), 1]))
            end
            Q = zeros(length(P));
            Q(1:self.n_c, 1:self.n_c) = Q_tilde;
            Q(self.n_c + self.n_d + self.n_y + (1:self.n_u), self.n_c + self.n_d + self.n_y + (1:self.n_u)) = R_tilde;
            if nargin == 6
                % x_r, u_r given -> add entries
                % (which would otherwise be zero anyway)
                Q(end, 1:self.n_c) = - x_r' * Q_tilde;
                Q(1:self.n_c, end) = - Q_tilde * x_r;
                Q(self.n_c + self.n_d + self.n_y + (1:self.n_u), end) = - R_tilde * u_r;
                Q(end, self.n_c + self.n_d + self.n_y + (1:self.n_u)) = - u_r' * R_tilde;
                Q(end, end) = x_r' * Q_tilde * x_r + u_r' * R_tilde * u_r;
            end
            J = stochasticJumplinearSystem.cost(P, Q);
        end
        
        % [J, Ad_total] = static_cost(T, delay_u, delay_y, Ad, Bd, Cd, fd, gd, H, Q_tilde, R_tilde)
        % compute stationary QoC for constant delay
        function [J, Ad_total] = static_cost(self, T, delay_u, delay_y, Ad, Bd, Cd, fd, gd, H, Q_tilde, R_tilde)
            assert(size(H, 1) == size(self.Gp, 2));
            assert(size(H, 2) == size(self.Gp, 2));
            assert(size(H, 3) == 1, 'timevarying H is not permitted')
            % delays must be within +- T/2.
            assert(abs(delay_u) < T / 2)
            assert(abs(delay_y) < T / 2)
            % Generate list of discrete events
            events = struct('t', {}, 'f', {});
            events(1).t = delay_y;
            events(1).f = @(P) self.sample(P, 1);
            events(2).t = delay_u;
            events(2).f = @(P) self.actuate(P, 1, Cd, gd);
            events(3).t = T / 2;
            events(3).f = @(P) self.compute(P, Ad, Bd, fd);
            % pseudo-event to reach end
            events(4).t = T;
            events(4).f = @(P) self.noop(P);
            for j = 1:length(events)
                % wrap around negative times (we can do this because
                % the timing is constant)
                if events(j).t < 0
                    events(j).t = events(j).t + T;
                end
            end
            % sort by time
            [~, sorted_event_indices] = sort([events(:).t]);
            
            timesteps = 1000;
            P = self.P0();
            Ad_total = eye(length(P));
            for i = 1:timesteps
                t = 0;
                for j = sorted_event_indices
                    [P, Astep] = self.elapse(P, events(j).t - t, H);
                    Ad_total = Ad_total * Astep;
                    t = events(j).t;
                    [P, Astep] = events(j).f(P);
                    Ad_total = Ad_total * Astep;
                end
                % elapse time until end of cycle
                [P, Astep] = self.elapse(P, T - t, H);
                Ad_total = Ad_total * Astep;
                % compute cost
                J = self.cost(P, Q_tilde, R_tilde);
            end
        end
        
        % [J, Ad_total] = timevarying_cost(T, delay_u, delay_y, delay_random_func, delay_random_points, Ad, Bd, Cd, fd, gd, H, Q_tilde, R_tilde, t_cost, x_r, u_r)
        % compute time-varying QoC for varying delays
        % The delays must be within +- T/2, because k*T+T/2 is used as
        % timing boundary.
        %
        %
        % T, Ad, Bd, Cd, Q_tilde, R_tilde:
        %     constant system parameters,
        %     see [Gaukler et al., HSCC2018].
        %     (Note: timevarying Ad, Bd, Cd are not yet implemented)
        % t_cost: The cost in each cycle is computed at k*T+t_cost, where
        %     t_cost must be in the range (-T/2, T/2].
        %
        % Time-varying parameters and delay
        % H: noise covariance (*last* dimension is time)
        %    size(H) == [n_d, n_d, timepoints]
        % gd, fd, x_r, u_r: time-varying controller parameters for reference tracking
        %   (*last* dimension is time, i. e., one column per time step)
        %    size(gd) == [n_u, timepoints]
        %    size(fd) == [n_y, timepoints],
        %    size(x_r) == [length(self.Ap), timepoints]
        %    size(u_r) == [self.n_u, timepoints]
        %    or, if the value is constant, timepoints=1 is also permitted.
        %
        %
        % delay_u: delay for u (*first* dimension is time)
        %    size(delay_u) == [timepoints, n_u]
        % delay_y: delay for y, same format as delay_y
        %
        % (TODO change code so that all parameters have time as the last dimension)
        % These sequences start at discrete time 0, such as [ y[k=0], y[k=1],
        % ... ].
        %
        %
        % Delay randomness:
        % For deterministic delays, specify
        % delay_random_func=[], delay_random_points=1.
        % For random delays, specify a function handle
        % delay_random_func= @(delay_u_param, delay_y_param, k, z) deal(my_delay_u, my_delay_y)
        % with the parameters:
        % - z is the random parameter
        %   that will be iterated in the range 1:delay_random_points.
        %   All values of z are assigned the same probability.
        % - delay_u_param, delay_y_param: value of delay_u(k), delay_y(k)
        %   (typically used for maximum or average value, if delays are
        %   time-varying)
        % - k is the current cycle index (for time-dependent pdf)
        % and the outputs:
        % - my_delay_u, my_delay_y: replace this with your code for
        %   computing the actual value of delay_u, delay_y.
        % deal(..., ...) is just Matlab's obscure way to return two
        % arguments from an anonymous function, see 'doc anonymous-functions'.
        %
        function [J, Ad_total] = timevarying_cost(self, T, delay_u, delay_y, delay_random_func, delay_random_points, Ad, Bd, Cd, fd, gd, H, Q_tilde, R_tilde, t_cost, x_r, u_r)
            % check dimensions of delay_u and delay_y
            % size(delay_u) == [timepoints, n_u]
            timepoints = size(delay_u, 1);
            assert(size(delay_u, 2) == self.n_u, 'wrong dimension of delay_u: it must be size(delay_u) == [timepoints, n_u]');
            assert(all(size(delay_y) == [timepoints, self.n_y]), 'wrong dimension of delay_y or delay_u: it must be size(delay_y) == [timepoints, n_y], with same number of timepoints as delay_u');
            
            % if gd, fd, x_r or u_r is constant in time, expand array in time dimension
            if all(size(gd) == [self.n_u, 1])
                gd = gd * ones(1, timepoints);
            end
            if all(size(u_r) == [self.n_u, 1])
                u_r = u_r * ones(1, timepoints);
            end
            if all(size(fd) == [self.n_d, 1])
                fd = fd * ones(1, timepoints);
            end
            if all(size(x_r) == [length(self.Ap), 1])
                x_r = x_r * ones(1, timepoints);
            end
            assert(all(size(gd) == [self.n_u, timepoints]), 'wrong dimension of gd (or delay_u): it must be size(gd) == [n_u, timepoints], with same number of timepoints as delay_u, or with timepoints=1');
            assert(all(size(u_r) == [self.n_u, timepoints]), 'wrong dimension of u_r (or delay_u): it must be size(u_r) == [n_u, timepoints], with same number of timepoints as delay_u, or with timepoints=1');
            assert(all(size(fd) == [self.n_d, timepoints]), 'wrong dimension of fd (or delay_u): it must be size(fd) == [n_d, timepoints], with same number of timepoints as delay_u, or with timepoints=1');
            assert(all(size(x_r) == [length(self.Ap), timepoints]), 'wrong dimension of x_r (or delay_u): it must be size(x_r) == [n_p, timepoints], with same number of timepoints as delay_u, or with timepoints=1');
            
            n_disturbance = size(self.Gp, 2);
            assert(n_disturbance <= 1, 'nonscalar d(t) is not yet implemented');
            
            % if H is constant in time, expand along time dimension
            if size(H, 3) == 1 && all(size(H) == [n_disturbance, n_disturbance])
                tmp = zeros(n_disturbance, n_disturbance, timepoints);
                for i = 1:timepoints
                    tmp(i) = H;
                end
                H = tmp;
            end
            assert((ndims(H) == 2 && all(size(H) == [n_disturbance, n_disturbance])) ...
                || (ndims(H) == 3 && all(size(H) == [n_disturbance, n_disturbance, timepoints])));%#ok<*ISMAT>
            
            if (~isa(delay_random_func, 'function_handle'))
                % deterministic delays
                assert(isempty(delay_random_func))
                assert(delay_random_points == 1)
                delay_random_func = @(delay_u_param, delay_y_param, k, z) deal(delay_u_param, delay_y_param);
            end
            
            J = nan(timepoints, 1);
            
            P_now_averaged = self.P0();
            Ad_total = eye(length(P_now_averaged));
            t = 0; % time relative to current sampling instant. We don't use absolute time to avoid inaccuracy.
            % The initialization at t=0 is counterintuitive
            % because we are halfway in the first control cycle:
            % If delay_u(1) < 0, the output u[k=0] will be skipped!
            % We still follow that definition because a classical
            % simulation also starts at t=0.
            for k = 1:timepoints
                % The Matlab index k equals k-1 from the publication
                % (HSCC2018) and refers to t=(k-1)*T  +- T/2.
                t_end_previous = t;
                P_previous_averaged = P_now_averaged;
                P_now_averaged = zeros(length(P_now_averaged));
                J_now_averaged = 0;
                for z = 1:delay_random_points
                    % iterate over all random timings
                    % in one cycle
                    
                    % compute:
                    % sampled cost J_now
                    % and covariance P at end (t=T/2)
                    J_now = NaN;
                    
                    % initialize with the previous averaged covariance
                    % at the previous end-of-cycle time
                    % (always t=T/2, except for t=0 for the first cycle k=0)
                    P = P_previous_averaged;
                    t = t_end_previous;
                    
                    % choose delays according to random parameter
                    [delay_u_now, delay_y_now] = delay_random_func(delay_u(k, :), delay_y(k, :), k, z);
                    
                    assert(all(abs(delay_u_now) < T / 2), 'delay u must be max. +-T/2')
                    assert(all(abs(delay_y_now) < T / 2), 'delay y must be max. +-T/2')
                    
                    % Generate list of discrete events
                    % as struct of
                    %  - time t (offset to the beginning of the current
                    %  control period, nominally 0 for sample/actuate, valid
                    %  range -T/2 < t < T/2)
                    % - covariance update function f
                    events = struct('t', {}, 'f', {});
                    for i = 1:self.n_y
                        events(end + 1).t = delay_y_now(i);
                        events(end).f = @(P) self.sample(P, i);
                    end
                    for i = 1:self.n_u
                        events(end + 1).t = delay_u_now(i);
                        events(end).f = @(P) self.actuate(P, i, Cd, gd(:, k));
                    end
                    events(end + 1).t = T / 2;
                    events(end).f = @(P) self.compute(P, Ad, Bd, fd(:, k));
                    
                    % the cost is computed at t_cost (usually the boundary)
                    assert(t_cost > -T / 2)
                    assert(t_cost <= T / 2)
                    
                    % ensure that there is an event at t=T_cost so that the cost
                    % is calculated
                    if ~(any([events(:).t] == t_cost)) && ~(k == 1 && t_cost < 0)
                        % add pseudo-event at t=T_cost.
                        % Except if t_cost would be before the start of
                        % absolute time (k=1, t=0), because it would be ignored
                        % anyway and cause a warning a few lines below.
                        events(end + 1).t = t_cost;
                        events(end).f = @(P) self.noop(P);
                    end
                    
                    % ensure that there is an event at t=T/2
                    % (end of current cycle)
                    if ~(any([events(:).t] == T / 2))
                        % add pseudo-event at t=T/2
                        events(end + 1).t = T / 2;
                        events(end).f = @(P) self.noop(P);
                    end
                    
                    % sort by time
                    [~, sorted_event_indices] = sort([events(:).t]);
                    for j = sorted_event_indices
                        if k == 1 && events(j).t < 0
                            % events before the start of absolute time (k=1, t=0) will be skipped
                            warning('negative delay at k=0 ignored')
                            continue
                        end
                        [P, Astep] = self.elapse(P, events(j).t - t, H(:, :, k));
                        Ad_total = Ad_total * Astep;
                        t = events(j).t;
                        [P, Astep] = events(j).f(P);
                        Ad_total = Ad_total * Astep;
                        if t == t_cost
                            J_now = self.cost(P, Q_tilde, R_tilde, x_r(:, k), u_r(:, k));
                        end
                    end
                    if (k == 1 && t_cost < 0)
                        % special case at startup for negative t_cost:
                        % the cost would have to be sampled before start of
                        % time, therefore it is set to zero.
                        assert(isnan(J_now));
                        J_now = 0;
                    end
                    assert(~isnan(J_now), 'cost calculation failed or was not triggered');
                    J_now_averaged = J_now_averaged + J_now / delay_random_points;
                    
                    
                    % ensure we are at the end of the current cycle,
                    % so that we don't average over P at different times
                    assert(t == T / 2);
                    t = t - T; % reset relative time
                    P_now_averaged = P_now_averaged + P / delay_random_points;
                end
                J(k) = J_now_averaged;
            end
        end
    end
    
end

