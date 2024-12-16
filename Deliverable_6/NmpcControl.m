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

            % Define variables for states and inputs over the horizon
            X = opti.variable(nx, N);    % states [x,y,θ,V] over horizon
            U = opti.variable(nu, N-1);  % inputs [δ,uT] over horizon
            
            % Initial input constraint 
            opti.subject_to(obj.u0 == U(:,1));

            % Initial state constraint 
            opti.subject_to(X(:,1) == obj.x0);

            % Initial input constraint 
            opti.subject_to(obj.u0 == U(:,1));

            % Initial state constraint
            opti.subject_to(X(:,1) == obj.x0);
            
            % Dynamic constraints using RK4 integration
            dt = car.Ts;
            for k = 1:N-1
                % RK4 integration
                k1 = car.f(X(:,k),         U(:,k));
                k2 = car.f(X(:,k) + dt/2*k1, U(:,k));
                k3 = car.f(X(:,k) + dt/2*k2, U(:,k));
                k4 = car.f(X(:,k) + dt*k3,   U(:,k));
                x_next = X(:,k) + dt/6*(k1 + 2*k2 + 2*k3 + k4);
                opti.subject_to(X(:,k+1) == x_next);
            end
            
            % State and input constraints 
            opti.subject_to(-0.5 <= X(2,:) <= 3.5);     % y position bounds
            opti.subject_to(-0.0873 <= X(3,:) <= 0.0873); % heading angle bounds
            opti.subject_to(-0.5236 <= U(1,:) <= 0.5236); % steering bounds
            opti.subject_to(-1 <= U(2,:) <= 1);   
            
            % Weights for the cost function
            Q_track = diag([0, 100, 10, 50]);
            R_track = diag([1, 1]); % Smoother control
            Q_term = 5*diag([0, 50, 5, 25]);
            
            % Cost function
            cost = 0;
            for k = 1:N-1
                % State error (track y position, theta angle and velocity)
                state_error = [0; X(2,k) - obj.ref(1); X(3,k); X(4,k) - obj.ref(2)];
                cost = cost + state_error'*Q_track*state_error + U(:,k)'*R_track*U(:,k);
                
            end
            
            % Terminal cost with higher weights
            state_error_N = [0; X(2,k) - obj.ref(1); X(3,k); X(4,k) - obj.ref(2)];
            cost = cost + state_error_N'*Q_term*state_error_N;
            
            % Set optimization objective and first input
            opti.minimize(cost);

            % Store variables for debugging
            obj.X = X;
            obj.U = U;

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
