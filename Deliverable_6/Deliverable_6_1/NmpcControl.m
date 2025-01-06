classdef NmpcControl < handle

    properties
        % The NMPC problem
        opti

        % Problem parameters
        x0, ref, x0other

        % Most recent problem solution
        sol

        % The input that you want to apply to the system
        u0

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Add any variables you would like to read to debug here
        % and then store them in the NmpcControl function below.
        % e.g., you could place X here and then add obj.X = X
        % in the NmpcControl function below.
        % 
        % After solving the problem, you can then read these variables 
        % to debug via
        %   nmpc.sol.value(nmpc.X)
        % 
        X, U
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    end

    methods
        function obj = NmpcControl(car, H)

            import casadi.*

            N_segs = ceil(H/car.Ts); % Horizon steps
            N = N_segs + 1;          % Last index in 1-based Matlab indexing

            nx = 4;
            nu = 2;

            % Define the NMPC optimization problem
            opti = casadi.Opti();
            
            % Parameters (symbolic)
            obj.x0 = opti.parameter(nx, 1);       % initial state
            obj.ref = opti.parameter(2, 1);       % target y, velocity
            obj.x0other = opti.parameter(nx, 1);  % initial state of other car

            % SET THIS VALUE TO BE YOUR CONTROL INPUT
            obj.u0 = opti.variable(nu, 1);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            % Define your problem using the opti object created above

            % Cost function weights
            Q_track = diag([0, 1, 20, 8]);    % State tracking weights:
                                              % [x position (unused), y position, heading angle, velocity]
            R_track = diag([10, 10]);         % Input cost weights for smoother control:
                                              % [steering angle, throttle]
            Q_term = Q_track/2;               % Terminal cost weight (typically smaller than tracking cost)
            
            % Define optimization variables
            X = opti.variable(nx, N);         % State trajectory over horizon N
                                              % States: [x position, y position, heading angle θ, velocity V]
            U = opti.variable(nu, N-1);       % Control inputs over horizon (N-1 steps)
                                              % Inputs: [steering angle δ, throttle input uT]
            
            % Initial conditions constraints
            opti.subject_to(X(:,1) == obj.x0);  % Initial state must match current state
            opti.subject_to(obj.u0 == U(:,1));  % Initial input must match previous input (for smoothness)
            
            % System dynamics constraints using RK4 (Runge-Kutta 4th order) integration
            dt = car.Ts;                      % Sampling time
            for k = 1:N-1
                % RK4 integration steps for more accurate discretization
                k1 = car.f(X(:,k), U(:,k));                    % Slope at start
                k2 = car.f(X(:,k) + dt/2*k1, U(:,k));         % Slope at midpoint using k1
                k3 = car.f(X(:,k) + dt/2*k2, U(:,k));         % Slope at midpoint using k2
                k4 = car.f(X(:,k) + dt*k3, U(:,k));           % Slope at end
                x_next = X(:,k) + dt/6*(k1 + 2*k2 + 2*k3 + k4);  % Weighted average of slopes
                opti.subject_to(X(:,k+1) == x_next);          % Enforce dynamics constraint
            end
            
            % State and input constraints (physical and safety limits)
            opti.subject_to(-0.5 <= X(2,:) <= 3.5);           % y position bounds (road limits)
            opti.subject_to(-5*pi/180 <= X(3,:) <= 5*pi/180); % heading angle bounds (±5 degrees)
            opti.subject_to(-30*pi/180 <= U(1,:) <= 30*pi/180); % steering angle bounds (±30 degrees)
            opti.subject_to(-1 <= U(2,:) <= 1);               % normalized throttle bounds [-1,1]
            
            % Cost function computation
            cost = 0;
            for k = 1:N-1
                % State error vector: [x error (ignored), y error, heading error, velocity error]
                state_error = [0; X(2,k) - obj.ref(1); X(3,k); X(4,k) - obj.ref(2)];
                
                % Accumulate cost: quadratic state error + quadratic input cost
                cost = cost + state_error'*Q_track*state_error + U(:,k)'*R_track*U(:,k);
            end
            
            % Terminal cost ensuring stability
            state_error_N = [0; X(2,N) - obj.ref(1); X(3,N); X(4,N) - obj.ref(2)];
            cost = cost + state_error_N'*Q_term*state_error_N;
            
            % Set up optimization problem
            opti.minimize(cost);              % Define objective function to minimize
            
            % Store optimization variables for debugging and visualization
            obj.X = X;                        % Save predicted state trajectory
            obj.U = U;                        % Save predicted control inputs

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Store the defined problem to solve in get_u
            obj.opti = opti;

            % Setup solver
            options = struct;
            options.ipopt.print_level = 0;
            options.print_time = 0;
            options.expand = true;
            obj.opti.solver('ipopt', options);
        end

        function u = get_u(obj, x0, ref, x0other)

            if nargin < 4
                x0other = zeros(4, 1);
            end

            % Compute solution from x0
            obj.solve(x0(1:4), ref, x0other(1:4));

            u = obj.sol.value(obj.u0);
        end

        function solve(obj, x0, ref, x0other)

            % Pass parameter values
            obj.opti.set_value(obj.x0, x0);
            obj.opti.set_value(obj.ref, ref);
            obj.opti.set_value(obj.x0other, x0other);

            obj.sol = obj.opti.solve();   % actual solve
            
            % Set warm start for next solve
            obj.opti.set_initial(obj.sol.value_variables());
            obj.opti.set_initial(obj.opti.lam_g, obj.sol.value(obj.opti.lam_g));
        end
    end
end

