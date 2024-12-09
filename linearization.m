clear all
close all
clc

%% Linearization

%% Deliverable 2.1

% Given nonlinear system: f(x,u) = [x_dot; y_dot; theta_dot; V_dot]

% Step 1: Steady State Analysis
% At steady state fs(xs,us): [y_dot; theta_dot; V_dot] = 0
% This implies:
% - V*sin(theta + beta) = 0         -> beta = 0, theta = 0
% - V/lr * sin(beta) = 0            -> delta = 0
% - (Fmotor - Fdrag - Froll)/m = 0  -> uT solves force balance

% Step 2: System Matrices A = df/dx, B = df/du at (xs,us)
% A matrix components (4x4):
% A11 = d(x_dot)/dx = 0
% A12 = d(x_dot)/dy = 0
% A13 = d(x_dot)/dtheta = 0
% A14 = d(x_dot)/dV = 1

% A21 = d(y_dot)/dx = 0
% A22 = d(y_dot)/dy = 0
% A23 = d(y_dot)/dtheta = Vs
% A24 = d(y_dot)/dV = 0

% A31 = d(theta_dot)/dx = 0
% A32 = d(theta_dot)/dy = 0
% A33 = d(theta_dot)/dtheta = 0
% A34 = d(theta_dot)/dV = 0

% A41 = d(V_dot)/dx = 0
% A42 = d(V_dot)/dy = 0
% A43 = d(V_dot)/dtheta = 0
% A44 = d(V_dot)/dV = -(Pmax*uT/Vs^2 + rho*Cd*Af*Vs)/m

% B matrix components (4x2):
% B11 = d(x_dot)/ddelta = 0
% B12 = d(x_dot)/duT = 0

% B21 = d(y_dot)/ddelta = (V * lr)/(lr + lf)
% B22 = d(y_dot)/duT = 0

% B31 = d(theta_dot)/ddelta = V/(lr + lf)
% B32 = d(theta_dot)/duT = 0

% B41 = d(V_dot)/ddelta = 0
% B42 = d(V_dot)/duT = Pmax/(m*Vs)

% At steady state (xs,us), with theta=0, delta=0:
% A = [0    0     0     1;
%      0    0     Vs    0;
%      0    0     0     0;
%      0    0     0    -(Pmax*uTs/Vs^2 + rho*Cd*Af*Vs)/m];

% B = [0                           0;
%      (Vs * lf)/(lr + lf)         0;
%      Vs/(lr + lf)                0;
%      0                Pmax/(m*Vs)];

% Where Vs is steady state velocity
% uTs is steady state throttle solving force balance

%% Todo 2.1
Ts = 1/10;
car = Car(Ts);
Vs = 120/3.6; % 120 km/h
[xs, us] = car.steady_state(Vs); % Compute steady−state for which f s(xs,us) = 0
sys = car.linearize(xs, us); % Linearize the nonlinear model around xs, us

% Display system matrices
disp('A matrix:')
disp(sys.A)
disp('B matrix:')
disp(sys.B)

% Show which states affect each other
states = {'x', 'y', 'theta', 'V'};
inputs = {'delta', 'u_T'};

% Create connectivity matrices showing independent subsystems
connectivity_A = abs(sys.A) > 1e-10;
connectivity_B = abs(sys.B) > 1e-10;

disp('State connectivity:')
array2table(connectivity_A, 'RowNames', states, 'VariableNames', states)

disp('Input connectivity:')
array2table(connectivity_B, 'RowNames', states, 'VariableNames', inputs)

%% Todo 2.2
[sys_lon, sys_lat] = car.decompose(sys);
disp('Longitudinal dynamics')
sys_lon
disp('Latera dynamics')
sys_lat

%% Discretization

[fd_xs_us, Ad, Bd, Cd, Dd] = Car.c2d_with_offset(sys, Ts);
