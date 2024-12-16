clear all
close all
clc

%% System Setup and Linearization
% Initialize the car system and compute steady-state conditions for tracking.
% The car system represents a vehicle with longitudinal dynamics that we want
% to control to maintain a safe distance from a lead vehicle.

Ts = 1/10;                % Sampling time (10 Hz) for discrete-time system
car = Car(Ts);           % Create car object with given sampling time
Vs = 120/3.6;            % Target velocity: Convert 120 km/h to m/s (33.33 m/s)

% Compute steady-state values and linearize system around them
[xs, us] = car.steady_state(Vs);  % Find equilibrium point at desired velocity
sys = car.linearize(xs, us);      % Linearize nonlinear dynamics around equilibrium
[sys_lon, ~] = car.decompose(sys); % Extract longitudinal dynamics (ignore lateral)

% Convert continuous-time system to discrete-time for MPC implementation
sys_lon_d = c2d(sys_lon, Ts);     % Discrete-time conversion
[A, B, C, D] = ssdata(sys_lon_d); % Extract state-space matrices

% Adjust input matrix sign for relative dynamics
% We flip sign because we're dealing with relative dynamics between ego and lead car
B = -B;  % This matches equation (9) from the problem statement

%% Tracking Controller Design
% Design an LQR controller for tracking the reference trajectory.
% This controller will be used during normal operation.

% Define cost weights for tracking control
% States: [relative position; relative velocity]
Q_track = diag([8, 4]);  % Higher weight on position than velocity 
R_track = 15;             % Moderate penalty on control effort

% Compute optimal feedback gain using discrete-time LQR
K = -dlqr(A, B, Q_track, R_track); 

% Compute closed-loop system matrix for invariant set computation
Acl = A + B*K;

%% Minimal Robust Invariant Set Computation
% Calculate the smallest set that bounds the effect of disturbances.
% This set represents the tube around our nominal trajectory.
uTs = 0;
% Define input disturbance set (uncertain lead car throttle)
W = Polyhedron('lb', uTs-0.5, 'ub', uTs+0.5);  % Throttle can vary by ±0.5 TAKE INTO ACCOUNT UTs?
W = -B*W;                                       % Map throttle disturbance to state space

% Initialize visualization
figure('Name', 'Minimal Robust Invariant Set Computation');
hold on; grid on; axis equal;

% Initialize set sequence at origin
F{1} = Polyhedron('lb', [0;0], 'ub', [0;0]);

% Setup computation parameters
i = 1;
tol = 1e-2;  % Convergence tolerance
colors = winter(100);  % Color scheme for visualization

% Iteratively compute minimal robust invariant set
while true
    % Compute next set in sequence: F_{i+1} = F_i ⊕ A^i W
    F{i+1} = F{i} + Acl^(i)*W;
    F{i+1}.minHRep();  % Minimize representation for efficiency
    
    % Visualize current iteration
    plot(F{i+1}, 'Color', colors(i,:), 'Alpha', 0.3);
    title(sprintf('Iteration %d', i));
    
    % Check convergence based on system response decay
    if norm(Acl^i) < tol
        fprintf('Minimal robust invariant set converged after %d iterations\n', i);
        break;
    end
    
    % Iteration management
    i = i + 1;
    if i > 100
        warning('Maximum iterations reached in invariant set computation');
        break;
    end
    drawnow;
end

% Store final minimal robust invariant set
E = F{end};
E.minHRep();

% Finalize visualization
xlabel('Position Error');
ylabel('Velocity Error');
title('Minimal Robust Invariant Set');

%% Define Safe Distance and Constraints
% Set up the state and input constraints for the system.
% These ensure safety and feasible operation.

% Define safety parameters
xsafe_pos = 8;           % Desired safety gap (meters)
xsafe = [xsafe_pos; 0];  % Safety reference point [position; velocity]

% Define state constraints relative to safety gap
min_total_distance = 6;   % Minimum required gap between vehicles
max_total_distance = 1e3;  % Maximum desired following distance
max_rel_velocity = 1e3;    % Maximum allowed relative velocity

% Convert absolute constraints to relative constraints
x_min = [min_total_distance - xsafe_pos; -max_rel_velocity];
x_max = [max_total_distance - xsafe_pos;  max_rel_velocity];
X = Polyhedron('lb', x_min, 'ub', x_max);

% Define input constraints (throttle limits)
u_min = -1;  % Maximum braking
u_max = 1;   % Maximum acceleration
U = Polyhedron('lb', u_min, 'ub', u_max);

%% Compute Tightened Constraints
% Tighten the constraints to account for the tube (invariant set)
X_tilde = X - E;        % Tightened state constraints
X_tilde.minHRep();

KE = K * E;             % Effect of tracking controller on invariant set
U_tilde = U - KE;       % Tightened input constraints
U_tilde.minHRep();

%% Terminal Controller Design
% Design a more conservative controller for terminal set computation
% This ensures stability of the MPC scheme

% More conservative weights for terminal control
Q_term = Q_track/2;      % Half of tracking weights - more conservative
R_term = R_track*2;               % Double input penalty - smoother control

% Compute terminal controller and cost
[Kt, Qf] = dlqr(A, B, Q_term, R_term);
Kt = -Kt;
Acl_t = A + B*Kt;  % Terminal closed-loop system

%% Terminal Set Computation
% Compute the terminal set for the MPC controller
% This set ensures stability of the overall scheme

% Extract constraint matrices from polytopes
Fx = X_tilde.A; fx = X_tilde.b;  % State constraints
Fu = U_tilde.A; fu = U_tilde.b;  % Input constraints

% Combine state and input constraints for terminal set
F = [Fx; Fu*Kt];     % Combined constraint matrix
f = [fx; fu];        % Combined constraint bounds

% Initialize terminal set
terminal_set = Polyhedron('A', F, 'b', f);

% Iteratively compute maximal invariant set
max_iter = 100;
iter = 0;
while iter < max_iter
    prev_set = terminal_set;
    
    % Compute one-step backward reachable set
    pre_set = Polyhedron('A', [F; F*Acl_t], 'b', [f; f]);
    pre_set.minHRep();
    
    % Intersection with previous set
    terminal_set = intersect(terminal_set, pre_set);
    terminal_set.minHRep();
    
    % Check for convergence
    if terminal_set == prev_set
        fprintf('Terminal set computation converged after %d iterations\n', iter);
        break;
    end
    
    iter = iter + 1;
    if iter == max_iter
        warning('Maximum iterations (%d) reached in terminal set computation', max_iter);
    end
end

% Verify terminal set is not empty
if terminal_set.isEmptySet()
    error(['Terminal set is empty - adjust xsafe_pos or controller tuning.\n'...
          'Current xsafe_pos: %.2f'], xsafe_pos);
end

%% Results Visualization and Analysis
% Create visualization of the key sets and constraints
figure('Name', 'Constraints vs Tightened Constraints');
hold on;
plot(U, 'alpha', 0.3, 'color', 'blue', 'DisplayName', 'Constraints');
plot(U_tilde, 'alpha', 0.3, 'color', 'red', 'DisplayName', 'Tightened Constraints');
title('Constraints vs Tightened Constraints');
xlabel('Throttle input');
grid on;
% Create visualization of the key sets and constraints
figure('Name', 'Terminal Invariant Set');
hold on;
% plot(E, 'alpha', 0.3, 'color', 'blue', 'DisplayName', 'Invariant Set (Tracking)');
plot(terminal_set, 'alpha', 0.3, 'color', 'red', 'DisplayName', 'Terminal Set');
% plot(X_tilde, 'alpha', 0.1, 'color', 'green', 'DisplayName', 'Tightened Constraints');
title('Terminal Invariant Set');
xlabel('Relative Position Error (m)');
ylabel('Relative Velocity Error (m/s)');
% legend('Location', 'best');
grid on;

% Print analysis results
fprintf('\n=== Controller Analysis ===\n');
fprintf('Tracking weights: Q11=%.1f, Q22=%.1f, R=%.1f\n', Q_track(1,1), Q_track(2,2), R_track);
fprintf('Terminal weights: Q11=%.1f, Q22=%.1f, R=%.1f\n', Q_term(1,1), Q_term(2,2), R_term);
fprintf('Origin in terminal set: %d\n', terminal_set.contains([0;0]));
fprintf('Terminal set volume: %.2f\n', terminal_set.volume);
fprintf('Invariant set volume: %.2f\n', E.volume);

% Save all computed components for MPC implementation
save('tube_mpc_data.mat', ...
    'K', 'Kt', 'Qf', ...               % Controllers
    'Q_track', 'R_track', ...          % Tracking weights
    'Q_term', 'R_term', ...            % Terminal weights
    'E', 'terminal_set', ...           % Sets
    'X_tilde', 'U_tilde', ...          % Constraints
    'min_total_distance', 'xsafe_pos' ...  % Safety parameters
);