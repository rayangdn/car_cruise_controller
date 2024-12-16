clear all
close all
clc

%% System Dynamics

%% Todo 1.1: evaluate f function
Ts = 1/10; % sec
car = Car(Ts);
u = [0.8, 0.9]; % [rad -]
x = [1, 1, 0.4, 3]'; % [m m rad m/s]
x_dot = car.f(x, u);

%% Todo 1.2: Simulate the car with various step inputs

% Initialize simulation parameters
Ts = 1/10;
car = Car(Ts); 
Tf = 2.0; % Simulation end time

% Initial state (x, y, theta, V)
x0 = [0, 0, deg2rad(-2), 20/3.6]'; % (x, y, theta, V) Initial state

% Test different input scenarios (delta, u T) 
scenarios = {
    [deg2rad(2), 0.7]',   % Slight right turn with acceleration
    [deg2rad(-2), 0.5]',   % Slight left turn with moderate throttle
    [0, 1.0]',             % Straight acceleration
    [0, -0.5]'             % Straight deceleration
};

% Run simulations for each scenario
for i = 1:length(scenarios)
    params = {}; % Setup simulation parameter struct
    params.Tf = Tf;
    params.myCar.model = car;
    params.myCar.x0 = x0;
    params.myCar.u = scenarios{i};
    
    fprintf('Simulating scenario %d with steering = %.2f deg, throttle = %.2f\n', ...
            i, rad2deg(scenarios{i}(1)), scenarios{i}(2));
    
    result = simulate(params); % Simulate nonlinear model
    visualization(car, result);
    
    % Inspect the result
    % result.T % Time at every simulation step
    % result.myCar.X % State trajectory
    % result.myCar.U % Input trajectory 

    % Wait for input before next plot
    input('Press Enter to continue...');
end
