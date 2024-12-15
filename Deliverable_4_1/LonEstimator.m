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

            % Get the index for velocity in the longitudinal subsystem
            % In the longitudinal model, state 1 is position, state 2 is velocity
            velocity_idx = 2; % Index of velocity in the longitudinal subsystem
            
            % Set up the extended state vector that includes both velocity and disturbance
            % xs_hat = [velocity; disturbance] where:
            % - xs(velocity_idx) is the steady-state velocity from linearization point
            % - 0 is the initial guess for the disturbance (assumed zero initially)
            est.xs_hat = [xs(velocity_idx); 0]; % [V; d]
            
            % Store the steady-state input for reference
            % This is needed when converting between absolute and deviation coordinates
            est.us_hat = us;
            
            % Create the augmented system matrix A_hat for the extended state [V; d]
            % The matrix has form: [a b; 0 1] where:
            % - a = Ad(velocity_idx,velocity_idx) is how velocity evolves
            % - b = Bd(velocity_idx) is how the disturbance affects velocity
            % - [0 1] represents that disturbance stays constant (persistence model)
            est.A_hat = [Ad(velocity_idx,velocity_idx), Bd(velocity_idx);
                         0,                             1];
            
            % Create the augmented input matrix B_hat for the extended state
            % The matrix has form: [b; 0] where:
            % - b = Bd(velocity_idx) is how the input affects velocity
            % - 0 indicates input doesn't affect disturbance
            est.B_hat = [Bd(velocity_idx); 
                         0];
            
            % Create the augmented output matrix C_hat
            % The matrix has form: [c 0] where:
            % - c = Cd(velocity_idx,velocity_idx) is the velocity measurement
            % - 0 indicates we can't measure the disturbance directly
            est.C_hat = [Cd(velocity_idx,velocity_idx), 0]; % Only V is measured
            
            % Choose observer poles for desired estimation dynamics
            % Poles closer to 0 give faster estimation
            % Poles closer to 1 give slower, more filtered estimation
            % Values of 0.6 and 0.7 give moderately fast response
            poles = [0.6; 0.7];
            
            % Calculate the observer gain matrix L using pole placement
            % We transpose A_hat and C_hat for the place command, then transpose the result
            % This L matrix will determine how quickly our estimates converge to true values
            est.L = place(est.A_hat', est.C_hat', poles)';
            
            % Store the original continuous-time system model in the estimator object
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
            %   y          - longitudinal velocity measurement (V)
            % OUTPUTS
            %   z_hat_next - next time step estimate
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
           % Convert all signals to deviation coordinates by subtracting their linearization points
            % This is necessary because our linear model works with deviations from steady state
            
            % Convert input from absolute to deviation coordinates
            u_dev = u - est.us_hat; % Input deviation
            
            % Convert measurement from absolute to deviation coordinates
            % y contains the measured values [position; velocity]
            % est.xs_hat(1) is the steady-state velocity we linearized around
            y_dev = y - est.xs_hat(1); % Measurement deviation
            
            % Convert state estimate from absolute to deviation coordinates
            % z_hat contains our current estimates of [velocity; disturbance]
            % est.xs_hat contains the steady-state values [velocity_ss; disturbance_ss]
            z_dev = z_hat - est.xs_hat; % State estimate deviation
            
            % Apply the observer equation in deviation coordinates:
            % z_dev_next = A*z_dev + B*u_dev + L*(y - C*z_dev) where:
            % - est.A_hat * z_dev: How states evolve naturally
            % - est.B_hat * u_dev: Effect of input on states
            % - est.L * (y_dev - est.C_hat * z_dev): Correction based on measurement error
            z_dev_next = est.A_hat * z_dev + ...
                         est.B_hat * u_dev + ...
                         est.L * (y_dev - est.C_hat * z_dev);
            
            % Convert the next state estimate back to absolute coordinates
            % This gives us the actual velocity and disturbance estimates
            z_hat_next = z_dev_next + est.xs_hat;

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
