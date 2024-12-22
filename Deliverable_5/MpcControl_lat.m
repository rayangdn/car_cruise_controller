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

            % State cost matrix Q penalizes deviations from reference state
            % - Weight of 4 on lateral position error (y)
            % - Weight of 4 on heading angle error (theta)
            Q_track = diag([1, 1]);
            
            % Input cost matrix R penalizes control effort
            % - Weight of 5 on steering angle changes
            R_track = 2;
            
            % State constraints define safe operating region
            % - Lateral position y must stay within road boundaries [-0.5, 3.5] meters
            % - Heading angle theta must stay within [-5°, 5°]
            F = [1  0;    % y ≤ 3.5
                -1  0;    % -y ≤ 0.5
                 0  1;    % theta ≤ 5°
                 0 -1];   % -theta ≤ 5°
            f = [3.5;              % Max y
                 0.5;              % Min y (negative)
                 5*pi/180;         % Max theta
                 5*pi/180];        % Min theta (symmetric)
            
            % Input constraints limit steering angle to ±30 degrees
            M = [1; -1];           % Steering constraints: δ ≤ 30° and -δ ≤ 30°
            m = [30*pi/180;        % Max steering angle
                 30*pi/180];       % Min steering angle (symmetric)
            
            % Terminal cost and controller for stability
            % More conservative weights for terminal control
            Q_term = Q_track/2;    % Terminal state cost
            R_term = R_track*2;               % Terminal input cost
            [Kt, Qf, ~] = dlqr(mpc.A, mpc.B, Q_term, R_term);  % LQR solution
            Kt = -Kt;  % Convert to state feedback form u = Kx
            
            % Compute maximal invariant set for terminal constraints
            % This ensures recursive feasibility and stability
            Xf = polytope([F; M*Kt], [f; m]);  % Initial polytope combining state and input constraints
            Acl = mpc.A + mpc.B*Kt;           % Closed-loop dynamics matrix
            
            % Iteratively compute the maximal invariant set
            while 1
                prevXf = Xf;
                [T, t] = double(Xf);
                preXf = polytope(T*Acl, t);    % One-step backward reachable set
                Xf = intersect(Xf, preXf);     % Intersect with current set
                if isequal(prevXf, Xf)
                    break
                end
            end
            [Ff, ff] = double(Xf);  % Get final polytope representation
                        
            % Set up optimization variables
            X = sdpvar(nx, N);     % State trajectory [y; theta]
            U = sdpvar(nu, N-1);   % Input trajectory [steering_angle]
            
            % Initialize constraint list with initial condition
            con = (X(:,1) == x0 - mpc.xs);  % Initial state in deviation coordinates
            
            % Build constraints and objective over prediction horizon
            obj = 0;
            for k = 1:N-1
                % Dynamic constraints (deviation coordinates)
                con = con + (X(:,k+1) == mpc.A*X(:,k) + mpc.B*U(:,k));
                
                % State constraints (convert to absolute coordinates)
                con = con + (F*(X(:,k) + mpc.xs) <= f);
                
                % Input constraints (convert to absolute coordinates)
                con = con + (M*(U(:,k) + mpc.us) <= m);
                
                % Accumulate objective: tracking cost
                state_error = X(:,k) - (x_ref - mpc.xs);
                input_error = U(:,k) - (u_ref - mpc.us);
                obj = obj + state_error'*Q_track*state_error + input_error'*R_track*input_error;
            end
            
            % Add terminal constraint and cost
            con = con + (Ff*(X(:,N) + mpc.xs) <= ff);  % Terminal set constraint
            state_error = X(:,N) - (x_ref - mpc.xs);
            obj = obj + state_error'*Qf*state_error;    % Terminal cost from LQR
            
            % Extract first input (convert back to absolute coordinates)
            con = con + (u0 == U(:,1) + mpc.us);
            
            % Store debugging variables
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

            % Set the reference state (xs_ref)
            % - First component (ref) is the desired lateral position (y)
            % - Second component (0) is the desired heading angle (theta)
            % We want the car to reach the target lateral position with zero heading angle
            % This ensures the car is parallel to the road when it reaches the target
            xs_ref = [ref; 0];
            
            % Set the reference input (us_ref)
            % At steady state, we want zero steering angle
            % This makes sense because:
            % Once we reach the target position with zero heading angle (parallel to road)
            % We want to drive straight (no steering)
            us_ref = 0;
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
