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

%% Test 1: Open-loop trajectory test for longitudinal controller
% Test state
x_lon = [0 80/3.6]';    % Initial velocity at 80km/h
ref_lon = 120/3.6; % Reference velocity 120km/h 

% Get controller response and debug variables
[u_lon, X_lon, U_lon] = mpc_lon.get_u(x_lon, ref_lon);

% Plot open-loop predictions
figure;
subplot(2,1,1);
actual_velocity = X_lon(2,:) + mpc_lon.xs(2); % Add back the steady state offset
plot(0:Ts:(size(X_lon,2)-1)*Ts, actual_velocity*3.6);
hold on;
plot([0, (size(X_lon,2)-1)*Ts], [ref_lon, ref_lon]*3.6, 'r--');
ylabel('Velocity [km/h]');
xlabel('Time [s]');
title('Predicted velocity trajectory');
legend('Predicted', 'Reference');
grid on;

subplot(2,1,2);
actual_input = U_lon + mpc_lon.us; % Add back the steady state input
plot(0:Ts:(size(U_lon,2)-1)*Ts, actual_input);
ylabel('Throttle [-]');
xlabel('Time [s]');
title('Predicted input trajectory');
grid on;

%% Test 2: Open-loop trajectory test for lateral controller
% Test state
x_lat = [0 0]';      % Initial lateral position and heading
ref_lat = 3;        % Reference lateral position (3m lane change)

% Get controller response and debug variables
[u_lat, X_lat, U_lat] = mpc_lat.get_u(x_lat, ref_lat);

% Plot open-loop predictions
figure;
subplot(3,1,1);
actual_position = X_lat(1,:) + mpc_lat.xs(1); % Add back the steady state offset
plot(0:Ts:(size(X_lat,2)-1)*Ts, actual_position);
hold on;
plot([0, (size(X_lat,2)-1)*Ts], [ref_lat, ref_lat], 'r--');
ylabel('Lateral position [m]');
xlabel('Time [s]');
title('Predicted lateral position trajectory');
legend('Predicted', 'Reference');
grid on;

subplot(3,1,2);
actual_heading = X_lat(2,:) + mpc_lat.xs(2); % Add back the steady state offset
plot(0:Ts:(size(X_lat,2)-1)*Ts, actual_heading*180/pi);
ylabel('Heading angle [deg]');
xlabel('Time [s]');
title('Predicted heading trajectory');
grid on;

subplot(3,1,3);
actual_steering = U_lat + mpc_lat.us; % Add back the steady state input
plot(0:Ts:(size(U_lat,2)-1)*Ts, actual_steering*180/pi);
ylabel('Steering angle [deg]');
xlabel('Time [s]');
title('Predicted steering input trajectory');
grid on;

%% Test 3: Closed-loop trajectory simulation

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
