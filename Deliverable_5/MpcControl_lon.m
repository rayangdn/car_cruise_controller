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
          
            % Load pre-computed tube MPC components for robust control
            data = load('tube_mpc_data.mat');
            xsafe = [data.x_safe_pos; 0];  % Safe reference point [position; velocity]
            
            % Cost matrices for nominal trajectory tracking
            Q_track = data.Q_track;    % State cost matrix
            R_track = data.R_track;    % Input cost matrix

            % Get tightened constraint sets for robust feasibility
            % These account for the effect of disturbances
            Fx_t = data.X_tilde.A;     % Tightened state constraint matrix
            fx_t = data.X_tilde.b;     % Tightened state constraint bounds
            Fu_t = data.U_tilde.A;     % Tightened input constraint matrix
            fu_t = data.U_tilde.b;     % Tightened input constraint bounds

            % Setup optimization variables for nominal trajectories
            X = sdpvar(nx, N);     % Nominal state trajectory [position; velocity]
            U = sdpvar(nu, N-1);   % Nominal input trajectory [acceleration]
            
            % Initialize optimization problem
            obj = 0;    % Objective function
            con = [];   % Constraint list
            
            % Set initial state constraint in relative coordinates
            % Transform to deviation from safe reference point
            con = con + (X(:,1) == x0other - x0 - xsafe);
            
            % Build constraints and objective over prediction horizon
            for k = 1:N-1
                % System dynamics in relative coordinates
                % Note: Negative input due to relative dynamics transformation
                con = con + (X(:,k+1) == mpc.A*X(:,k) - mpc.B*U(:,k));
                
                % Apply tightened state and input constraints
                con = con + (Fx_t*X(:,k) <= fx_t);    % State constraints
                con = con + (Fu_t*U(:,k) <= fu_t);    % Input constraints
                
                % Stage cost: Penalize deviations from nominal trajectory
                obj = obj + X(:,k)'*Q_track*X(:,k) + U(:,k)'*R_track*U(:,k);
            end
            
            % Add terminal components for stability
            obj = obj + X(:,N)'*data.Qf*X(:,N);    % Terminal cost
            
            % Terminal set constraint for recursive feasibility
            Fterm = data.terminal_set.A;
            fterm = data.terminal_set.b;
            con = con + (Fterm*X(:,N) <= fterm);
            
            % Compute actual control input
            % u = u_nominal + K*(x - x_nominal)
            % where x is in relative coordinates
            delta_x = x0other - x0 - xsafe - X(:,1);  % State error
            con = con + (u0 == U(:,1) + data.K*delta_x);  % Tube controller
            
            % Store variables for debugging
            debugVars = {X, U};
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {x0, V_ref, u_ref, d_est, x0other}, {u0, debugVars{:}});
        end
        
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

            % Set the reference velocity (Vs_ref)
            % We directly use the requested reference as our target velocity
            Vs_ref = ref;
            
            % Set the reference input (us_ref)
            % The actual control action is determined by the tube MPC controller
            % This nominal value is not used since the control law provides the input
            us_ref = 0;

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end