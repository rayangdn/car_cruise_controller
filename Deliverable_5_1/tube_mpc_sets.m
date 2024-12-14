clear all
close all
clc

%% System Setup and Linearization
% Initialize the car system and compute steady-state conditions for tracking

% Set basic parameters
Ts = 1/10;                % Sampling time (10 Hz)
car = Car(Ts);           % Create car object with given sampling time
Vs = 120/3.6;            % Convert desired velocity from km/h to m/s (120 km/h)

% Compute steady-state conditions and linearize the system
[xs, us] = car.steady_state(Vs);  % Get steady-state values for tracking
sys = car.linearize(xs, us);      % Linearize around steady-state
[sys_lon, ~] = car.decompose(sys); % Extract longitudinal dynamics

% Convert continuous system to discrete-time
sys_lon_d = c2d(sys_lon, Ts);
[A, B, C, D] = ssdata(sys_lon_d);

% Adjust input matrix sign convention
B = -B;  % Flip sign for conventional control formulation

%% Tracking Controller Design
% Design an LQR controller for reference tracking

% Define LQR cost weights
Q = diag([10, 1]);  % State weights: [position error, velocity error]
R = 1;             % Input weight (throttle control effort)

% Compute optimal feedback gain
K = -dlqr(A, B, Q, R); 

% Compute closed-loop system matrix
Acl = A + B*K;

%% Minimal Robust Invariant Set Computation
% Calculate the smallest set that bounds the effect of disturbances

% Define input disturbance set around steady-state throttle
uTs = us(2);                                    % Steady-state throttle value
W = Polyhedron('lb', uTs-0.5, 'ub', uTs+0.5);  % Throttle disturbance bounds
W = -B*W;                                       % Map disturbances through input matrix

% Iteratively compute the minimal robust invariant set
F{1} = Polyhedron('lb', [0;0], 'ub', [0;0]);   % Initialize at origin
i = 1;
tol = 1e-2;  % Convergence tolerance

while true
    % Compute next set in sequence
    F{i+1} = F{i} + Acl^(i)*W;
    F{i+1}.minHRep();  % Simplify representation
    
    % Check if system response has decayed sufficiently
    if norm(Acl^i) < tol
        break;
    end
    
    i = i + 1;
    if i > 100  % Prevent infinite loops
        warning('Maximum iterations reached in invariant set computation');
        break;
    end
end

% Final minimal robust invariant set
E = F{end};
E.minHRep();

% Visualize the minimal robust invariant set
figure;
hold on;
plot(E, 'alpha', 0.5, 'color', 'blue');
title('Minimal Robust Invariant Set E');
xlabel('Position Error');
ylabel('Velocity Error');
grid on;

%% Constraint Tightening
% Modify constraints to ensure robustness against disturbances

% Define safety parameters
min_gap = 6;           % Minimum safe distance between cars (meters)
xsafe_pos = min_gap + 2;  % Add safety margin to minimum gap

% Define state constraints (relative position and velocity)
% deltaX = x_lead - x_ego - xsafe_pos
X = Polyhedron('lb', [min_gap - xsafe_pos; -1e5], 'ub', [1e5; 1e5]);

% Define input constraints on throttle
U = Polyhedron('lb', -1, 'ub', 1);

% Compute tightened constraints using Pontryagin difference
X_tilde = X - E;      % Tightened state constraints
U_tilde = U - K*E;    % Tightened input constraints

% Visualize original and tightened constraints
figure;
hold on;
plot(X, 'alpha', 0.5, 'color', 'red');
plot(X_tilde, 'alpha', 0.5, 'color', 'blue');
title('Original and Tightened State Constraints');
xlabel('Relative Position \Delta x [m]');
ylabel('Relative Velocity \Delta V [m/s]');
legend('Original Constraints X', 'Tightened Constraints X̃');
grid on;

% Plot input constraints
figure;
hold on;
plot(U, 'alpha', 0.5, 'color', 'red');
plot(U_tilde, 'alpha', 0.5, 'color', 'blue');
title('Original and Tightened Input Constraints');
xlabel('Throttle Input u_T');
ylabel('');
legend('Original Constraints U', 'Tightened Constraints Ũ');
grid on;

%% Terminal Controller Design
% Compute terminal LQR controller
[Kt, Qf] = dlqr(A, B, Q, R);
Kt = -Kt;  % Adjust sign convention

%% Terminal Set Computation
% Calculate invariant terminal set for MPC stability

% Compute closed-loop system with terminal controller
Acl_t = A + B*Kt;

% Extract constraint matrices from polytopes
Fx = X_tilde.A; fx = X_tilde.b;  % State constraints
Fu = U_tilde.A; fu = U_tilde.b;  % Input constraints

% Combine state and input constraints
F = [Fx; Fu*Kt];     % State and control constraints
f = [fx; fu];        % Constraint bounds

% Initialize terminal set
terminal_set = Polyhedron('A', F, 'b', f);

% Iteratively compute maximal invariant set
while true
    prev_set = terminal_set;
    
    % Compute one-step backward reachable set
    pre_set = Polyhedron('A', [F; F*Acl_t], 'b', [f; f]);
    pre_set.minHRep();
    
    % Update terminal set
    terminal_set = intersect(terminal_set, pre_set);
    terminal_set.minHRep();
    
    % Check convergence
    if terminal_set == prev_set
        break;
    end
end

%% Terminal Set Verification
% Verify key properties of the computed terminal set

if terminal_set.isEmptySet
    error('Terminal set is empty. Consider retuning the controller or adjusting constraints.');
end

if ~terminal_set.contains([0;0])
    error('Terminal set does not contain origin. Consider retuning the controller.');
end

% Visualize terminal set
figure;
hold on;
plot(terminal_set, 'alpha', 0.3, 'color', 'red');
title('Terminal Set and State Constraints');
xlabel('Relative Position \Delta x [m]');
ylabel('Relative Velocity \Delta V [m/s]');
legend('Terminal Set');
grid on;

%% Save results
save('tube_mpc_data.mat', ...
    'K', ...                          % Tracking controller gain
    'E', ...                          % Minimal robust invariant set
    'X_tilde', 'U_tilde', ...         % Tightened constraints
    'Kt', 'Qf', ...                   % Terminal controller and cost matrix
    'terminal_set', ...               % Terminal constraint set
    'Q', 'R', ...                     % Cost matrices for tracking
    'min_gap', 'xsafe_pos' ...        % Safety parameters
);



