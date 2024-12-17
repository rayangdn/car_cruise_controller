clear all
close all
clc

%% Initialize MPC Controllers
Ts = 1/10;                               % Sample time: 0.1 seconds
H = 2;                                   % Prediction horizon: 2 seconds

% Setup car and get linearized model
car = Car(Ts);
[xs, us] = car.steady_state(120/3.6);    % Linearize at 120 km/h
sys = car.linearize(xs, us);             % Get linearized system
[sys_lon, sys_lat] = car.decompose(sys); % Split into longitudinal and lateral

% Create MPC controllers
mpc_lon = MpcControl_lon(sys_lon, Ts, H);
mpc_lat = MpcControl_lat(sys_lat, Ts, H);
mpc = car.merge_lin_controllers(mpc_lon, mpc_lat);


%% Simulating Closed-Loop robuste tube MPC 

ref = [0 120/3.6]';
params = {};
params.Tf = 25;
params.myCar.model = car;
params.myCar.x0 = [0 0 0 115/3.6]';
params.myCar.u = @mpc.get_u;
params.myCar.ref = ref;
params.otherCar.model = car;
params.otherCar.x0 = [8 0 0 120/3.6]';
%params.otherCar.u = car.u_const(100/3.6);
params.otherCar.u = car.u_fwd_ref();
params.otherCar.ref = car.ref_robust();
result = simulate(params);
visualization(car, result);

%[f1, f2, f3, f4, f5, f6] = plot_results(result);