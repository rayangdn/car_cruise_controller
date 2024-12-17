clear all
close all
clc

%% Initialize MPC Controllers

Ts = 1/10;                               % Sample time: 0.1 seconds
H = 2;                                  % Prediction horizon: 2 seconds

% Setup car and get linearized model
car = Car(Ts);
[xs, us] = car.steady_state(120/3.6);    % Linearize at 120 km/h
sys = car.linearize(xs, us);             % Get linearized system
[sys_lon, sys_lat] = car.decompose(sys); % Split into longitudinal and lateral

% Create MPC controllers
mpc_lon = MpcControl_lon(sys_lon, Ts, H);
mpc_lat = MpcControl_lat(sys_lat, Ts, H);
mpc = car.merge_lin_controllers(mpc_lon, mpc_lat);

% Create Longitudinal Estimator
estimator = LonEstimator(sys_lon, Ts);

%% Simulating Closed-Loop Combined MPC with Longitudinal Estimator

x0 = [0 0 0 80/3.6]'; % (x, y, theta, V)
ref1 = [0 80/3.6]';   % (y ref, V ref)
ref2 = [3 50/3.6]';   % (y ref, V ref)
params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = x0;
params.myCar.est_fcn = @estimator.estimate;
params.myCar.est_dist0 = 0;
params.myCar.u = @mpc.get_u;
params.myCar.ref = car.ref_step(ref1, ref2, 2); % delay reference step by 2s;
result = simulate(params);
visualization(car, result);

% [f1, f2, f3, f4, f5, f6] = plot_results(result);