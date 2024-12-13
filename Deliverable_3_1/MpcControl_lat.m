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

            % Define cost matrices for the objective function
            % Q penalizes state deviations: lateral position (y) and heading angle (theta)
            % Higher weight on theta ensures good heading tracking
            Q = diag([1, 10]);     % State cost: [y_weight, theta_weight]
            R = 1;                 % Input cost: Penalty on steering angle
            
            % Define state constraints
            % The car must stay within road boundaries and maintain reasonable heading
            % y ∈ [-0.5, 3.5] meters (road boundaries)
            % theta ∈ [-5°, 5°] (heading angle limits)
            F = [1 0;              % Maximum y constraint
                 -1 0;             % Minimum y constraint
                 0 1;              % Maximum theta constraint
                 0 -1];           % Minimum theta constraint
            f = [3.5;              % Maximum y value
                 0.5;              % Minimum y value (negative)
                 5*pi/180;         % Maximum theta value
                 5*pi/180];        % Maximum theta value (symmetric)
            
            % Define input constraints
            % Limit steering angle to ±30 degrees
            M = [1; -1];           % For u ≤ 30° and -u ≤ 30°
            m = [30*pi/180;        % Maximum steering angle
                 30*pi/180];       % Maximum steering angle (symmetric)
            
            % Compute terminal cost and control law using LQR
            % This provides stability guarantees for the MPC controller
            [K, Qf,~] = dlqr(mpc.A,mpc.B,Q,R);   % Compute LQR solution
            K = -K;                              % Convert to state feedback form u = Kx
            
            % Compute maximal invariant set for terminal constraints
            % This set guarantees recursive feasibility and stability
            Xf = polytope([F;M*K],[f;m]);        % Initial polytope with state and input constraints
            Acl = mpc.A+mpc.B*K;                 % Closed-loop system matrix
            % Iterate until we find the maximal invariant set
            while 1
                prevXf = Xf;                     % Store current set
                [T,t] = double(Xf);              % Get inequality representation
                preXf = polytope(T*Acl,t);       % Compute one-step backward reachable set
                Xf = intersect(Xf, preXf);       % Intersect with current set
                if isequal(prevXf, Xf)           % Check if set has converged
                    break
                end
            end
            [Ff,ff] = double(Xf);                % Get final inequality representation
            
            % Visualize the constraint sets
            figure
            hold on; grid on;
            plot(polytope(F,f),'g');             % Plot state constraints in green
            plot(Xf,'r');                        % Plot terminal set in red
            xlabel('y position [m]');
            ylabel('steering angle [rad]');
            
            % Set up MPC optimization problem
            
            % Define optimization variables
            X = sdpvar(nx, N);    % States over horizon (deviation from xs)
            U = sdpvar(nu, N-1);  % Inputs over horizon (deviation from us)
            
            % Initial state constraint in deviation coordinates
            con = (X(:,1) == x0 - mpc.xs);
            
            % Loop through prediction horizon
            for k = 1:N-1
                % System dynamics in deviation coordinates
                % x(k+1) - xs = A(x(k) - xs) + B(u(k) - us)
                con = con + (X(:,k+1) == mpc.A*X(:,k) + mpc.B*U(:,k));
                
                % State constraints in absolute coordinates
                % Convert deviation state back to absolute for constraint checking
                con = con + (F*(X(:,k) + mpc.xs) <= f);
                
                % Input constraints in absolute coordinates
                % Convert deviation input back to absolute for constraint checking
                con = con + (M*(U(:,k) + mpc.us) <= m);
            end
            
            % Terminal constraint from invariant set (in deviation coordinates)
            % This ensures stability and recursive feasibility
            con = con + (Ff*(X(:,N) + mpc.xs) <= ff);
            
            % First input constraint (convert back to absolute coordinates)
            con = con + (u0 == U(:,1) + mpc.us);
            
            % Set up objective function
            obj = 0;
            
            % Add stage costs
            for k = 1:N-1
                % State error cost (in deviation coordinates)
                state_error = X(:,k) - (x_ref - mpc.xs);
                obj = obj + state_error'*Q*state_error;
                
                % Input error cost (in deviation coordinates)
                input_error = U(:,k) - (u_ref - mpc.us);
                obj = obj + input_error'*R*input_error;
            end
            
            % Add terminal cost using LQR 
            state_error = X(:,N) - (x_ref - mpc.xs);
            obj = obj + state_error'*Qf*state_error;
            
            % Store variables for debugging
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
