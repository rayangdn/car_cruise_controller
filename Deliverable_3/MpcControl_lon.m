classdef MpcControl_lon < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   V_ref, u_ref - reference state/input
            %   d_est        - disturbance estimate
            %   x0other      - initial state of other car
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps
            N = N_segs + 1;              % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets
            V_ref = sdpvar(1);
            u_ref = sdpvar(1);

            % Disturbance estimate (Ignore this before Todo 4.1)
            d_est = sdpvar(1);

            % Initial states
            x0 = sdpvar(nx, 1);
            x0other = sdpvar(nx, 1); % (Ignore this before Todo 5.1)

            % Input to apply to the system
            u0 = sdpvar(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system.
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            
            % Cost matrices for the MPC objective function
            % Q_track: State cost matrix that penalizes deviations from reference state
            % - No penalty on position (first state)
            % - Weight of 2 on velocity error (second state)
            Q_track = diag([0, 2]);
            
            % R_track: Input cost matrix that penalizes control effort
            % - Weight of 5 on throttle usage
            R_track = 1;
            
            % More conservative weights for terminal control
            Q_term = Q_track/2;
            
            % Input constraints: -1 ≤ u ≤ 1
            % These represent physical limits on the throttle
            M = [-1; 1];    % Constraint matrix
            m = [1; 1];     % Constraint bounds
            
            % Initialize optimization variables
            X = sdpvar(nx, N);    % State trajectory: [position; velocity]
            U = sdpvar(nu, N-1);  % Input trajectory: [throttle]
            
            % Initialize constraints list with initial state constraint
            % Convert initial state to deviation coordinates by subtracting linearization point
            con = (X(:,1) == x0 - mpc.xs);
            
            % Build constraints and objective over prediction horizon
            obj = 0;
            for k = 1:N-1
                % State dynamics constraint in deviation coordinates
                % x(k+1) = A*x(k) + B*u(k)
                con = con + (X(:,k+1) == mpc.A*X(:,k) + mpc.B*U(:,k));
                
                % Input constraints in absolute coordinates
                % Transform from deviation coordinates: u_absolute = u_deviation + u_steady
                con = con + (M*(U(:,k) + mpc.us) <= m);
                
                % State tracking cost
                % Define reference state (only tracking velocity)
                x_ref = [0; V_ref];
                state_error = X(:,k) - (x_ref - mpc.xs);
                obj = obj + state_error'*Q_track*state_error;
                
                % Input tracking cost
                % Track reference input in deviation coordinates
                input_error = U(:,k) - (u_ref - mpc.us);
                obj = obj + input_error'*R_track*input_error;
            end
            
            % Add terminal state cost for stability
            x_ref = [0; V_ref];
            state_error = X(:,N) - (x_ref - mpc.xs);
            obj = obj + state_error'*Q_term*state_error;
            
            % Constraint to extract first input to apply to system
            % Convert back to absolute coordinates
            con = con + (u0 == U(:,1) + mpc.us);
            
            % Store variables for debugging
            debugVars = {X, U};
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {x0, V_ref, u_ref, d_est, x0other}, {u0, debugVars{:}});
        end
        
        % Computes the steady state target which is passed to the
        % controller
        function [Vs_ref, us_ref] = compute_steady_state_target(mpc, ref, d_est)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            %   d_est  - disturbance estimate (Ignore before Todo 4.1)
            % OUTPUTS
            %   Vs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Steady-state subsystem
            A = mpc.A(2, 2);
            B = mpc.B(2, 1);

            % Subsystem linearization steady-state
            xs = mpc.xs(2);
            us = mpc.us;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            % Store the target velocity as our reference
            % This is the absolute velocity we want to achieve
            Vs_ref = ref;
            
            % Compute the required steady-state input (throttle) to maintain the reference velocity
            % At steady state, the state doesn't change, so x(k+1) = x(k)
            % This means: 0 = A*(Vs_ref - xs) + B*(us_ref - us)
            % Solving for us_ss:
            % us_ref = us - A*(Vs_ref-xs)/B
            % where:
            %   - A, B are the linearized system matrices for velocity
            %   - xs is the linearization velocity point
            %   - us is the linearization input point
            %   - (Vs_ref-xs) is how far we want to deviate from the linearization point
            us_ref = us - (A*(Vs_ref-xs))/B;
            
            % Enforce input constraints by saturating the computed input
            % The input must stay within ±1 
            % This ensures we respect the physical limitations of the throttle
            us_ref = min(max(us_ref, -1), 1);

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
