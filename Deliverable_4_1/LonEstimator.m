classdef LonEstimator
    properties
        % continous sub-system
        sys
        % Extended linearization points
        xs_hat, us_hat
        % Extended system matrices
        A_hat, B_hat, C_hat
        % Observer gain matrix
        L
    end
    
    methods
        % Setup the longitudinal disturbance estimator
        function est = LonEstimator(sys, Ts)

            xs = sys.UserData.xs;
            us = sys.UserData.us;
            
            % Discretize the system and extract the A,B,C,D matrices
            [~, Ad, Bd, Cd, ~] = Car.c2d_with_offset(sys, Ts);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % Extended state includes velocity and disturbance: z = [V; d]
            % We only need velocity dynamics (second row of Ad, Bd)
            A = Ad(2, 2); % Extract V dynamics
            B_dist = Bd(2); % Extract input effect on V
            C = [1, 0];  % Measure velocity [V; d]

            % Extended state dynamics

            % Store linearization points
            est.xs_hat = [xs(2); 0]; % velocity and zero disturbance
            est.us_hat = us;      % throttle input

            %Extended system matrices
            est.A_hat = [A B_dist;   % Original V dynamics with disturbance effect
                         0 1];       % Disturbance dynamics (constant) 
            est.B_hat = [B_dist;     % Input effect on V
                         0];         % Input doesn't affect disturbance
            est.C_hat = C;       % Measure velocity only
            
            % Design observer gain L
            % Place poles for desired estimator dynamics
            poles = [0.7, 0.8]; % can be tuned
            est.L = -place(est.A_hat', est.C_hat', poles)';

            est.sys = sys;
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        % This function takes in the the estimate, input, and measurement
        % at timestep i and predicts the next (extended) state estimate at
        % the next timestep i + 1.
        function z_hat_next = estimate(est, z_hat, u, y)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   z_hat      - current estimate (V, dist)
            %   u          - longitudinal input (u_T)
            %   y          - longitudinal measurement (x, V)
            % OUTPUTS
            %   z_hat_next - next time step estimate
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % Estimation equation:
            % z_hat_next = A_hat*z_hat + B_hat*u + L*(y - C_hat*z_hat)
            z_hat_next = est.A_hat * z_hat + ...
                         est.B_hat * (u - est.us_hat) + ...
                         est.L *(est.C_hat * z_hat - y); 
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
