clear all
close all
clc

%% Initialize MPC Controller

Ts = 1/10;  % Sample time from project description
car = Car(Ts);
H = 2;      % 2 seconds prediction horizon

% Create non linear controller
mpc = NmpcControl(car, H);

%% Simulate Open-Loop Nonlinear MPC
x0 = [0 0 0 80/3.6]';
ref = [3 100/3.6]';
u = mpc.get_u(x0, ref);
X_pred = mpc.sol.value(mpc.X);
U_pred = mpc.sol.value(mpc.U);

% Plot predictions
figure;

% Position and velocity plots
subplot(3,1,1);
plot(0:Ts:(H), X_pred(2,:), 'b.-');  % y position
hold on;
plot([0,H], [ref(1),ref(1)], 'r--');  % y reference
ylabel('y position [m]');
grid on;

subplot(3,1,2);
plot(0:Ts:(H), X_pred(4,:)*3.6, 'b.-');  % Convert V to km/h
hold on;
plot([0,H], [ref(2)*3.6,ref(2)*3.6], 'r--');  % V reference
ylabel('velocity [km/h]');
grid on;

subplot(3,1,3);
plot(0:Ts:(H-Ts), U_pred(1,:), 'b.-');  % steering
hold on;
plot(0:Ts:(H-Ts), U_pred(2,:), 'r.-');  % throttle
ylabel('inputs');
legend('steering [rad]', 'throttle [-]');
grid on;
xlabel('time [s]');

%% Simulating Closed-Loop Nonlinear MPC

x0 = [0 0 0 80/3.6]'; % (x, y, theta, V)
ref1 = [0 80/3.6]'; % (y ref, V ref)
ref2 = [3 100/3.6]'; % (y ref, V ref)
params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = x0;
params.myCar.u = @mpc.get_u;
params.myCar.ref = car.ref_step(ref1, ref2, 2); % delay reference step by 2s
result = simulate(params);
visualization(car, result);

%[f1, f2, f3, f4, f5, f6] = plot_results(result);

