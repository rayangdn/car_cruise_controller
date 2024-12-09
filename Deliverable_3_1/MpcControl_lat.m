classdef MpcControl_lat < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps
            N = N_segs + 1;              % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);

            % Initial states
            x0 = sdpvar(nx, 1);
            x0other = sdpvar(nx, 1); % (Ignore this, not used)

            % Input to apply to the system
            u0 = sdpvar(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            
            % Predicted state trajectory (deviation from steady state)
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            % Cost matrices
            Q = diag([1, 10]); % Penalize y and theta error, higher weight on heading
            R = 1;             % Penalize steering angle usage
            P = Q;             % Terminal cost

            % Constraints
            con = [];

            % Initial state constraint
            con = con + (X(:,1) == x0 - mpc.xs);
            
            % Dynamic constraints
            con = con + (X(:,2:N) == mpc.A*X(:,1:N-1) + mpc.B*U(:,1:N-1));
            
            % Input constraints |δ| ≤ 30° = 0.5236 rad
            con = con + (-0.5236 - mpc.us <= U <= 0.5236 - mpc.us);
            
            % State constraints:
            % -0.5 m ≤ y ≤ 3.5 m
            % |θ| ≤ 5° = 0.0873 rad
            con = con + (-0.5 - mpc.xs(1) <= X(1,:) <= 3.5 - mpc.xs(1));    % y position
            con = con + (-(5 * pi / 180) - mpc.xs(2) <= X(2,:) <= (5 * pi / 180) - mpc.xs(2)); % theta

                        % Objective function
            % Objective function
            obj = 0;
            for k = 1:N-1
                % State cost (needs to consider xs offset)
                obj = obj + (X(:,k) - [x_ref(1)-mpc.xs(1); x_ref(2)-mpc.xs(2)])'*Q*(X(:,k) - [x_ref(1)-mpc.xs(1); x_ref(2)-mpc.xs(2)]);
                % Input cost
                obj = obj + (U(:,k) - (u_ref-mpc.us))'*R*(U(:,k) - (u_ref-mpc.us));
            end
            % Terminal cost
            obj = obj + (X(:,N) - [x_ref(1)-mpc.xs(1); x_ref(2)-mpc.xs(2)])'*P*(X(:,N) - [x_ref(1)-mpc.xs(1); x_ref(2)-mpc.xs(2)]);

            % Set input to apply (add back steady-state offset)
            con = con + (u0 == U(:,1) + mpc.us);

            % Debug variables
            debugVars = {X, U};
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {x0, x_ref, u_ref, x0other}, {u0, debugVars{:}});
        end
        
        % Computes the steady state target which is passed to the
        % controller
        function [xs_ref, us_ref] = compute_steady_state_target(mpc, ref)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Steady-state system
            A = mpc.A;
            B = mpc.B;

            % Linearization steady-state
            xs = mpc.xs;
            us = mpc.us;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            xs_ref = [ref; 0];  % Reference position and zero heading
            us_ref = 0;         % Zero steering angle at steady state
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
