classdef MpcControl_lon < MpcControlBase
    
    methods
        function mpc = MpcControl_lon(sys, Ts, H)
            % Call superclass constructor
            mpc = mpc@MpcControlBase(sys, Ts, H);

        end
        
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            
            % Horizon parameters
            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps
            N = N_segs + 1;              % Last index in 1-based Matlab indexing
            
            [nx, nu] = size(mpc.B);
            
            % Targets (not used in relative dynamics)
            V_ref = sdpvar(1);
            u_ref = sdpvar(1);
            
            % Disturbance estimate (not used in tube MPC)
            d_est = sdpvar(1);
            
            % Initial states (actual and lead car)
            x0 = sdpvar(nx, 1);
            x0other = sdpvar(nx, 1);
            
            % Input to apply to the system
            u0 = sdpvar(nu, 1);
            
            % YALMIP Decision variables
            X = sdpvar(nx, N); % Nominal state trajectory
            U = sdpvar(nu, N-1); % Nominal input trajectory


            % Load pre-computed tube MPC components
            data = load('tube_mpc_data.mat');
            
            % Safe reference point [position; velocity]
            xsafe = [data.xsafe_pos; 0];
            
            % Initialize objective and constraints
            obj = 0;
            con = [];
            
            % Initial state constraint (relative coordinates)
            con = con + (X(:,1) == x0other - x0 - xsafe);
            
            % Cost matrices from data
            Q = data.Q_track; % State cost for tracking
            R = data.R_track; % Input cost for tracking
            
            % Compute tightened constraint matrices
            Fx_t = data.X_tilde.A; % State constraints
            fx_t = data.X_tilde.b;
            Fu_t = data.U_tilde.A; % Input constraints
            fu_t = data.U_tilde.b;
        
            
            % Loop over prediction horizon
            for k = 1:N-1
                % Cost function
                obj = obj + X(:,k)'*Q*X(:,k) + U(:,k)'*R*U(:,k);
                
                % System dynamics
                con = con + (X(:,k+1) == mpc.A*X(:,k) - mpc.B*U(:,k)); % '-' becausse of the change of dynamics
                % State and input constraints
                con = con + (Fx_t*X(:,k) <= fx_t);  % Tightened state constraints
                con = con + (Fu_t*U(:,k) <= fu_t);  % Tightened input constraints
            end
            
            % Terminal cost and constraint
            obj = obj + X(:,N)'*data.Qf*X(:,N);
            
            % Get terminal set constraints
            Fterm = data.terminal_set.A;
            fterm = data.terminal_set.b;
            con = con + (Fterm*X(:,N) <= fterm);
            
            % Compute actual input: u = u_nominal + K*(x - x_nominal)
            % where x is in relative coordinates
            delta_x = x0other - x0 - xsafe - X(:,1);
            con = con + (u0 == U(:,1) + data.K*delta_x);
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {x0, V_ref, u_ref, d_est, x0other}, {u0, X, U});
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
            us_ref = 0;
            

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end