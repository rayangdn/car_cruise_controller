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

%% Simulate Open-Loop Longitudinal MPC

% Setup initial conditions and reference
x_lon = [0 80/3.6]';                     % Start at 80 km/h
ref_lon = 120/3.6;                       % Target 120 km/h

% Get controller prediction
[u_lon, X_lon, U_lon] = mpc_lon.get_u(x_lon, ref_lon);

% Plot longitudinal results
figure;
% Velocity plot
subplot(2,1,1);
actual_velocity = X_lon(2,:) + mpc_lon.xs(2);
plot(0:Ts:(size(X_lon,2)-1)*Ts, actual_velocity*3.6);
hold on;
plot([0, (size(X_lon,2)-1)*Ts], [ref_lon, ref_lon]*3.6, 'r--');
ylabel('Velocity [km/h]');
xlabel('Time [s]');
title('Predicted velocity trajectory');
legend('Predicted', 'Reference');
grid on;

% Throttle plot
subplot(2,1,2);
actual_input = U_lon + mpc_lon.us;
plot(0:Ts:(size(U_lon,2)-1)*Ts, actual_input);
ylabel('Throttle [-]');
xlabel('Time [s]');
title('Predicted input trajectory');
grid on;

%% Simulate Open-Loop Lateral MPC

% Setup initial conditions and reference
x_lat = [0 0]';                          % Start at center of lane
ref_lat = 3;                             % 3m lane change

% Get controller prediction
[u_lat, X_lat, U_lat] = mpc_lat.get_u(x_lat, ref_lat);

% Plot lateral results
figure;
% Lateral position plot
subplot(3,1,1);
actual_position = X_lat(1,:) + mpc_lat.xs(1);
plot(0:Ts:(size(X_lat,2)-1)*Ts, actual_position);
hold on;
plot([0, (size(X_lat,2)-1)*Ts], [ref_lat, ref_lat], 'r--');
ylabel('Lateral position [m]');
xlabel('Time [s]');
title('Predicted lateral position trajectory');
legend('Predicted', 'Reference');
grid on;

% Heading angle plot
subplot(3,1,2);
actual_heading = X_lat(2,:) + mpc_lat.xs(2);
plot(0:Ts:(size(X_lat,2)-1)*Ts, actual_heading*180/pi);
ylabel('Heading angle [deg]');
xlabel('Time [s]');
title('Predicted heading trajectory');
grid on;

% Steering angle plot
subplot(3,1,3);
actual_steering = U_lat + mpc_lat.us;
plot(0:Ts:(size(U_lat,2)-1)*Ts, actual_steering*180/pi);
ylabel('Steering angle [deg]');
xlabel('Time [s]');
title('Predicted steering input trajectory');
grid on;

%% Simulating Closed-Loop Combined MPC

x0 = [0 0 0 80/3.6]'; % (x, y, theta, V)
ref1 = [0 80/3.6]'; % (y ref, V ref)
ref2 = [3 120/3.6]'; % (y ref, V ref)
params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = x0;
params.myCar.u = @mpc.get_u;
params.myCar.ref = car.ref_step(ref1, ref2, 5); % delay reference step by 5s
result = simulate(params);
visualization(car, result);

% [f1, f2, f3, f4, f5, f6] = plot_results(result);

