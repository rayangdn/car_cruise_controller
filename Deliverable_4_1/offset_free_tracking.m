clear all
close all
clc

%% Controller initialization
Ts = 1/10;  % Sample time
H_lon = 20;  % Horizon length (2 seconds prediction horizon)

% Initialize car and get linearized system
car = Car(Ts);
[xs, us] = car.steady_state(120/3.6); % Linearize around v=120km/h
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);

% Create controllers
mpc_lon = MpcControl_lon(sys_lon, Ts, H_lon);
mpc_lat = MpcControl_lat(sys_lat, Ts, H_lon);
mpc = car.merge_lin_controllers(mpc_lon, mpc_lat);

estimator = LonEstimator(sys_lon, Ts);
x0 = [0 0 0 80/3.6]'; % (x, y, theta, V)
ref1 = [0 80/3.6]'; % (y ref, V ref)
ref2 = [3 50/3.6]'; % (y ref, V ref)
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