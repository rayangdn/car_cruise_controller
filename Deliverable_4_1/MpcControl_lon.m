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

            % Prediction horion optimization variables
            X = sdpvar(nx, N); % States over horizon
            U = sdpvar(nu, N-1); % Inputs over horizon
            
            % Cost matrices
            Q = diag([0, 1]); % Only penalize velocity error, not position (states are [x, V])
            R = 1;            % Penalize throttle usage
            
            % Terminal cost matrix
            %P = dlyap(mpc.A, Q); doesnt work???
            P = Q;

            % Constraints
            con = [];

            % Initial state constraint
            con = con + (X(:,1) == x0 - mpc.xs);

            % System dynamics constraints
            con = con + (X(:,2:N) == mpc.A*X(:,1:N-1) + mpc.B*U(:,1:N-1));
            
            % Input constraints |U_T| ≤ 1
            con = con + (-1 - mpc.us <= U <= 1 - mpc.us);

            % Set input to apply (add back steady-state offset)
            con = con + (u0 == U(:,1) + mpc.us);
            
            % Terminal set constraint (approximated with simple box constraint)
            % WHAT SHOULD WE PUT???
            % Objective function
            obj = 0;
            for k = 1:N-1
                % State error cost (deviation from target)
                obj = obj + (X(:,k) - [0; V_ref-mpc.xs(2)])'*Q*(X(:,k) - [0; V_ref-mpc.xs(2)]);
                % Input cost (penalize deviation from u_ref)
                obj = obj + (U(:,k) - (u_ref-mpc.us))'*R*(U(:,k) - (u_ref-mpc.us));
            end
            % Terminal cost
            obj = obj + (X(:,N) - [0; V_ref-mpc.xs(2)])'*P*(X(:,N) - [0; V_ref-mpc.xs(2)]);
                        
            % Debug variables
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
            Vs_ref = ref;
            us_ref = (1-A)/B * (ref-xs) + us - 1/B * d_est;
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
