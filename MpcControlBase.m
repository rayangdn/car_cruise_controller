classdef MpcControlBase
    properties
        H, Ts
        ctrl_opti   % YALMIP optimizer object to compute control law
        % linearization points
        xs, us, f_xs_us
        % Discrete-time system matrices
        A, B, C, D
    end
    
    methods
        function mpc = MpcControlBase(sys, Ts, H)
            mpc.xs = sys.UserData.xs;
            mpc.us = sys.UserData.us;

            % Discretize the system and extract the A,B,C,D matrices
            [mpc.f_xs_us, mpc.A, mpc.B, mpc.C, mpc.D] = Car.c2d_with_offset(sys, Ts);

            % Horizon
            mpc.Ts = Ts;
            mpc.H = H;
            mpc.ctrl_opti = mpc.setup_controller();
        end
        
        % Compute the MPC controller
        function [u, varargout] = get_u(mpc, x, ref, xOther)

            nx = size(mpc.A, 1);

            if nargin < 4
                xOther = zeros(nx, 1);
            end

            if length(struct(mpc.ctrl_opti).diminOrig) == 5
                if length(x) > nx
                    d_est = x(nx+1:end);
                else
                    d_est = 0;
                end
                % Compute steady state target
                [ref_x, ref_u] = mpc.compute_steady_state_target(ref, d_est);
                % Compute the control action
                [sol, solve_status] = mpc.ctrl_opti({x(1:nx), ref_x, ref_u, d_est, xOther});
            else
                % Compute steady state target
                [ref_x, ref_u] = mpc.compute_steady_state_target(ref);
                % Compute the control action
                [sol, solve_status] = mpc.ctrl_opti({x, ref_x, ref_u, xOther});
            end
            if iscell(sol)
                u = sol{1};
            else
                u = sol;
            end
            
            nout = max(nargout, 1) - 1;
            for k = 1:nout
                varargout{k} = sol{k+1};
            end
            
            if solve_status ~= 0
                solve_status_str = yalmiperror(solve_status);
                fprintf([' [' class(mpc) ' control: ' solve_status_str(1:end-1) '] ']);
                u = nan(size(u));
            end
        end
    end
    
    methods (Abstract)
        setup_controller(mpc)
        compute_steady_state_target(mpc, ref, d_est);
    end
end
