classdef Car
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Car class
    %
    % Structure (Click VIEW > Collapse All for better overview)
    %
    % methods
    % - System
    % - Simulation
    % - Visualization
    %
    % methods (hidden)
    % - System
    % - Simulation
    % - Visualization
    %
    % methods (static)
    % - Helpers
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    properties
        sys        % State, input, output names and units
        Ts         % Sampling time
        
        % Physical properties and constraints (VW ID.3)
        mass   = 1800;   % mass of the car
        length = 4.3;    % vehicle lenght
        width  = 1.8;    % vehicle width
        lr     = 1.56;   % rear length
        lf     = 1.04;   % front length
        Cd     = 0.267;  % drag coefficient
        Cr     = 0.01;   % roll resistance
        rho    = 1.225;  % air density
        Af     = 2.36;   % frontal area
        Pmax   = 100000; % max power of motor
        g      = 9.81;   % gravitational acceleration
    end
    properties (Constant)
        % System dimensions
        dimx = 4; % Length of state vector
        dimu = 2; % Length of input vector
        
        % Input limits
        ubu = [deg2rad(30);   1]; % upper bound for input: [rad, -]
        lbu = [deg2rad(-30); -1]; % lower bound for input: [rad, -]
        
        % State limits for (linear) control
        ubx = [Inf   3.5 deg2rad(5)   Inf]';
        lbx = [-Inf -0.5 deg2rad(-5) -Inf]';
        
        % Indices in the state vector
        indx = struct('x', 1, 'y', 2, 'theta', 3, 'V', 4);
        indu = struct('delta', 1, 'u_T', 2);
    end
    properties (Hidden, Constant)
        % Plotting colors
        color = struct('meas', [0, 0.4470, 0.7410], 'ref', 'k');
    end
    
    methods
        %
        % Constructor
        %
        function obj = Car(Ts)
            
            obj.Ts = Ts;
            
            try % Test YALMIP installation
                sdpvar(2,1);
            catch
                error('Could not load YALMIP - check that it is installed properly and on the path.')
            end
            try % Test casadi installation
                import casadi.*
                casadi.SX.sym('x');
            catch
                error('Could not load casadi - check that it is installed properly and on the path.')
            end
            
            % Define system state, input, output names and inputs
            sys.StateName = { 'x', 'y', 'theta', 'V'};
            sys.StateUnit = { 'm', 'm', 'rad', 'm/s'};
            
            sys.InputName = {'delta', 'u_T'};
            sys.InputUnit = {'rad', ''};
            
            sys.OutputName = sys.StateName;
            sys.OutputUnit = sys.StateUnit;
            
            obj.sys = sys;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % System
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %
        % Kinematic bicycle dynamics x_dot = f(x,u)
        % ----------------------------------------------------
        % x = [x; y; theta; V]
        % x     [m]   X position in world frame
        % y     [m]   Y position in world frame
        % theta [rad] Orientation
        % V     [m/s] Velocity in driving direction
        % ----------------------------------------------------
        % u = [delta; u_T]
        % delta [rad] Steering angle
        % u_T Throttle
        %
        function [x_dot] = f(obj, x, u)
            
            if nargin < 3
                u = zeros(2, 1);
            end
            
            % Decompose state
            [~, ~, theta, V] = obj.parse_state(x);

            % Slip angle
            beta = atan(obj.lr * tan(u(Car.indu.delta)) / (obj.lr + obj.lf));
            
            x_pos_dot = V * cos(theta + beta);
            y_pos_dot = V * sin(theta + beta);
            theta_dot = V / obj.lr * sin(beta);
            
            F_motor = u(Car.indu.u_T) * obj.Pmax / max(abs(V), 1.0);
            F_drag  = 0.5 * obj.rho * obj.Cd * obj.Af * V^2;
            F_roll  = obj.Cr * obj.mass * obj.g;
            V_dot = (F_motor - F_drag - F_roll) / obj.mass;
            
            % Compose state derivative
            x_dot = [x_pos_dot; y_pos_dot; theta_dot; V_dot];
        end
        
        %
        % Compute steady point for velocity (theta_dot, V_dot = 0)
        %
        function [xs, us] = steady_state(obj, V_ref)
            
            xs = [0; 0; 0; V_ref];

            x = casadi.SX.sym('x', 4);
            u = casadi.SX.sym('u', 2);
            x_dot = obj.f(x, u);
            V_dot = casadi.Function('Vdot', {x,u}, {x_dot(4)});
            V_dot_jac = casadi.Function('Vdot', {x,u}, {jacobian(x_dot(4), u)});

            % Run newton's method to find us such that V_dot = 0
            us_guess = [0; V_ref];
            while 1
                us = us_guess - full(V_dot_jac(xs,us_guess)) \ full(V_dot(xs,us_guess));
                if norm(us - us_guess) < 1e-8
                    break;
                end
                us_guess = us;
            end

            % Clean up numerical inaccuracies
            xs(xs < 1e-5) = 0;
            us(us < 1e-5) = 0;
        end

        %
        % Return the linearization of the system around the
        % equilibrium point (xs, us)
        %
        function linSys = linearize(obj, xs, us)
            if nargin < 3
                fprintf('No equilibrium given... linearizing around a velocity of 100km/h\n');
                [xs, us] = obj.steady_state(100 / 3.6);
            end
            
            % Create casadi symbolic variables
            x = casadi.SX.sym('x', 4);
            u = casadi.SX.sym('u', 2);
            f = obj.f(x, u);
            
            % Create symbolic casadi function for automatic differentiation
            % of A = df/dx, B = df/du. Evaluate and densify.
            A = casadi.Function('A', {x,u}, {jacobian(f, x)});
            A = full(A(xs, us));
            B = casadi.Function('B', {x,u}, {jacobian(f, u)});
            B = full(B(xs, us));
            
            % Clean up numerical inaccuracies
            A(abs(A) < 1e-5) = 0;
            B(abs(B) < 1e-5) = 0;
            
            % Create state space representation
            linSys = ss(A, B, eye(4), zeros(4, 2));
            linSys.UserData.xs = xs;
            linSys.UserData.us = us;
            linSys.UserData.f_xs_us = obj.f(xs, us);
            
            linSys.StateName = obj.sys.StateName;
            linSys.StateUnit = obj.sys.StateUnit;
            
            linSys.InputName = obj.sys.InputName;
            linSys.InputUnit = obj.sys.InputUnit;
            
            linSys.OutputName = obj.sys.OutputName;
            linSys.OutputUnit = obj.sys.OutputUnit;
        end
        
        %
        % Decompose the system into two systems
        %
        function [sys_lon, sys_lat] = decompose(obj, linSys)
            
            % Split into two seperate systems
            I = obj.indx;
            
            % [x V], [u_T]
            idx = [I.x I.V];
            idu = obj.indu.u_T;
            idy = idx;
            
            sys_lon = obj.parse_system(linSys, idx, idu, idy, 'sys lon');
            
            % [y theta], [delta]
            idx = [I.y I.theta];
            idu = obj.indu.delta;
            idy = idx;
            
            sys_lat = obj.parse_system(linSys, idx, idu, idy, 'sys lat');
        end

        %
        % Merge linear controllers
        %
        function mpc = merge_lin_controllers(obj, mpc_lon, mpc_lat)
            
            % Get state indices
            xs = zeros(obj.dimx, 1);
            us = zeros(obj.dimu, 1);
            linSys = obj.linearize(xs, us);
            [sys_x, sys_y] = obj.decompose(linSys);
            
            Iu = zeros(2, 1);
            
            idx_x = sys_x.UserData.idx;
            idu_x = sys_x.UserData.idu;
            Iu_x  = Iu; Iu_x(idu_x) = 1;
            
            idx_y = sys_y.UserData.idx;
            idu_y = sys_y.UserData.idu;
            Iu_y  = Iu; Iu_y(idu_y) = 1;
            
            % Define a local function so we can set a break point in the
            % function to evaluate sub-controllers independently
            function u = merged_get_u(z_, ref_, z_other_)
                % If z_ is the state vector, 5:end = [], and mpc_x will be
                % evaluated with 0 disturbance
                u_lon = mpc_lon.get_u([z_(idx_x); z_(5:end)], ref_(2), z_other_(idx_x) );
                u_lat = mpc_lat.get_u( z_(idx_y),             ref_(1), z_other_(idx_y) );
                
                u = Iu_x .* u_lon + Iu_y .* u_lat;
            end
            
            function u = get_u_wrapper(z_, ref_, z_other_)
                % If only u requested, use lightweight function to avoid
                % unnecessary evaluations of T_opt, X_opt, U_opt
                if nargin < 3
                    z_other_ = zeros(size(z_));
                end

                u = merged_get_u(z_, ref_, z_other_);
            end
            
            % Return function handle
            mpc.get_u = @get_u_wrapper;
        end

        function u_handle = u_const(obj, ref)

            [~, us] = obj.steady_state(ref);

            function u = u_func(~, ~, ~)
                u = us;
            end

            u_handle = @u_func;
        end

        function u_handle = u_fwd_ref(~)

            function u = u_func(~, ref, ~)
                u = ref;
            end

            u_handle = @u_func;
        end

        function ref_handle = ref_robust(obj)

            [~, us] = obj.steady_state(120/3.6);

            function r = ref_func(t, ~, ~)
                if t < 7.5
                    r = [0; us(2)];
                elseif t < 15
                    r = [0; us(2)-0.5];
                else
                    r = [0; us(2)+0.5];
                end
            end

            ref_handle = @ref_func;
        end

        function ref_handle = ref_step(~, ref1, ref2, delay)

            function r = ref_func(t, ~, ~)
                if t < delay
                    r = ref1;
                else
                    r = ref2;
                end
            end

            ref_handle = @ref_func;
        end
        
    end
    
    methods (Static)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Helpers
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %
        % Split the state into its parts, in: State trajectory (nx x time)
        %
        function [x, y, theta, V] = parse_state(X)
            if nargout >= 1, x     = X(Car.indx.x,     :); end
            if nargout >= 2, y     = X(Car.indx.y,     :); end
            if nargout >= 3, theta = X(Car.indx.theta, :); end
            if nargout >= 4, V     = X(Car.indx.V,     :); end
        end
        
        %
        % Create sub-system from indices
        %
        function sub_sys = parse_system(sys, idx, idu, idy, name)
            
            [A, B, C, ~] = ssdata(sys);
            
            sub_sys = ss(A(idx,idx), B(idx,idu), C(idy,idx), 0);
            
            sub_sys.UserData.idx = idx;
            sub_sys.UserData.idu = idu;
            sub_sys.UserData.idy = idy;
            
            sub_sys.UserData.xs = sys.UserData.xs(idx);
            sub_sys.UserData.us = sys.UserData.us(idu);
            sub_sys.UserData.f_xs_us = sys.UserData.f_xs_us(idx);
            
            sub_sys.Name = name;
            sub_sys.StateName = sys.StateName(idx);
            sub_sys.StateUnit = sys.StateUnit(idx);
            
            sub_sys.InputName = sys.InputName(idu);
            sub_sys.InputUnit = sys.InputUnit(idu);
            
            sub_sys.OutputName = sys.OutputName(idy);
            sub_sys.OutputUnit = sys.OutputUnit(idy);
        end

        %
        % Discretize continous time system of form
        %     x_dot = f_xs_us + A (x - xs) + B (u - us)
        % to
        %     x_next = fd_xs_us + Ad (x - xs) + Bd (u - us)
        %
        function [fd_xs_us, Ad, Bd, Cd, Dd] = c2d_with_offset(sys, Ts)
            [nx, nu] = size(sys.B);
            xs = sys.UserData.xs;
            us = sys.UserData.us;

            c = sys.UserData.f_xs_us - sys.A * xs - sys.B * us;
            Ae = [sys.A sys.B c;
                  zeros(nu + 1, nx + nu + 1)];
            expAe = expm(Ae * Ts);

            Ad = expAe(1:nx, 1:nx);
            Bd = expAe(1:nx, nx+1:nx+nu);
            Cd = sys.C;
            Dd = sys.D;
            fd_xs_us = expAe(1:nx, nx+nu+1) + Ad * xs + Bd * us;
        end

    end
    
end