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
            
            % Define cost matrices for the MPC objective function
            % Q penalizes state deviations from reference
            % We only penalize velocity error (second state) by setting Q = diag([0, 1])
            Q = diag([0, 1]);     % State cost matrix: No penalty on position, weight of 1 on velocity
            R = 1;                % Input cost matrix: Penalty of 1 on throttle usage
            
            % Define input constraints: -1 ≤ u - us ≤ 1
            % These constraints ensure the throttle input stays within ±1 of the linearization point us
            M = [-1; 1];          % Constraint matrix for -u ≤ 1 and u ≤ 1
            m = [1; 1];          % Constraint bounds
            
            % Set up MPC optimization problem
            
            % Define optimization variables
            % X represents the state trajectory: position and velocity deviations from xs
            % U represents the input trajectory: throttle deviations from us
            X = sdpvar(nx, N);    % States for all N timesteps (nx × N matrix)
            U = sdpvar(nu, N-1);  % Inputs for N-1 timesteps (nu × (N-1) matrix)
            
            % Set up constraints
            
            % Initial state constraint
            % The initial state deviation must equal the difference between the actual initial state
            % and the linearization point
            con = (X(:,1) == x0 - mpc.xs);
            
            % Loop through the prediction horizon to set up dynamics and input constraints
            for k = 1:N-1
                % System dynamics constraint
                % The state evolution follows the discrete-time linearized dynamics:
                % x(k+1) - xs = A(x(k) - xs) + B(u(k) - us)
                con = con + (X(:,k+1) == mpc.A*X(:,k) + mpc.B*U(:,k));
                
                % Input constraints
                % The absolute input (U + us) must stay within ±1
                % Transform deviation coordinates to absolute: u = U + us
                con = con + (M*(U(:,k) + mpc.us) <= m);
            end
            
            % Constraint on first input to be applied
            % Convert the input from deviation coordinates back to absolute coordinates
            con = con + (u0 == U(:,1) + mpc.us);
            
            % Set up the objective function
            obj = 0;
            
            % Loop through the prediction horizon to sum up stage costs
            for k = 1:N-1
                % State error term
                % x_ref is the target state in absolute coordinates
                % Convert to deviation coordinates by subtracting xs
                x_ref = [0; V_ref];                    % Target state (no position reference, only velocity)
                state_error = X(:,k) - (x_ref - mpc.xs);   % Error in deviation coordinates
                obj = obj + state_error'*Q*state_error;    % Quadratic state cost
                
                % Input error term
                % Track the reference input in deviation coordinates
                input_error = U(:,k) - (u_ref - mpc.us);   % Error in deviation coordinates
                obj = obj + input_error'*R*input_error;    % Quadratic input cost
            end
            
            % Add terminal cost (same form as stage cost)
            x_ref = [0; V_ref];
            state_error = X(:,N) - (x_ref - mpc.xs);
            obj = obj + state_error'*Q*state_error;
            
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
            % This means: 0 = A*(Vs - xs) + B*(us - us_ss)
            % Solving for us_ss:
            % us_ss = us - A*(Vs-xs)/B
            % where:
            %   - A, B are the linearized system matrices for velocity
            %   - xs is the linearization velocity point
            %   - us is the linearization input point
            %   - (Vs_ref-xs) is how far we want to deviate from the linearization point
            us_ref = us - (A*(Vs_ref-xs))/B;
            
            % Enforce input constraints by saturating the computed input
            % The input must stay within ±1 of the linearization point us
            % This ensures we respect the physical limitations of the throttle
            us_ref = min(max(us_ref, mpc.us-1), mpc.us+1);

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
