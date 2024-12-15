function [f1, f2, f3, f4, f5, f6] = plot_results(result, varargin)
%PLOT_VEHICLE_RESULTS_SEPARATE Plots vehicle simulation results in separate figures and saves them
% This function creates and saves separate figures for velocity, position/heading,
% throttle, steering, disturbance estimation, and relative distance
%
% Inputs:
%   car - Structure containing vehicle parameters
%   result - Structure containing simulation results
%   varargin - Optional input for save directory (default: 'plots')
%
% Outputs:
%   f1 - Handle to velocity figure
%   f2 - Handle to position/heading figure
%   f3 - Handle to throttle figure
%   f4 - Handle to steering figure
%   f5 - Handle to disturbance estimation figure (if available)
%   f6 - Handle to relative distance figure (if available)

    % Parse input arguments
    if nargin > 1
        save_dir = varargin{1};
    else
        save_dir = 'plots';
    end
    
    % Create plots directory if it doesn't exist
    if ~exist(save_dir, 'dir')
        mkdir(save_dir);
        fprintf('Created directory: %s\n', save_dir);
    end

    % Extract data
    t = result.T;
    x1 = result.myCar.X(1,:);  % x position
    y1 = result.myCar.X(2,:);  % y position
    theta1 = result.myCar.X(3,:);  % heading
    v1 = result.myCar.X(4,:) * 3.6;  % velocity (converted to km/h)
    
    delta1 = rad2deg(result.myCar.U(1,:));  % steering angle in degrees
    throttle1 = result.myCar.U(2,:);  % throttle input
    
    % Figure 1: Velocity
    f1 = figure('Name', 'Velocity', 'Position', [100, 100, 800, 400]);
    grid on;
    box on;
    hold on;
    plot(t, v1, 'b', 'LineWidth', 1.5, 'DisplayName', 'Car 1');
    if isfield(result.myCar, 'Ref')
        refv1 = result.myCar.Ref(2,:) * 3.6;
        stairs(t, refv1, '--b', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    end
    if isfield(result, 'otherCar')
        v2 = result.otherCar.X(4,:) * 3.6;
        plot(t, v2, 'r', 'LineWidth', 1.5, 'DisplayName', 'Car 2');
    end
    xlabel('Time [s]');
    ylabel('V [km/h]');
    title('Velocity');
    legend('show');
    saveas(gcf, fullfile(save_dir, 'velocity.png'));
    
    % [Rest of the plotting code remains the same until saving part]
    % Adding save commands for each figure...
    
    % Figure 2: Y Position and Heading
    f2 = figure('Name', 'Y Position and Heading', 'Position', [150, 150, 800, 400]);
    grid on;
    box on;
    hold on;
    yyaxis left
    plot(t, y1, 'b', 'LineWidth', 1.5, 'DisplayName', 'y position');
    if isfield(result.myCar, 'Ref')
        refy1 = result.myCar.Ref(1,:);
        stairs(t, refy1, '--b', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    end
    ylabel('y [m]');
    yyaxis right
    plot(t, rad2deg(theta1), 'r', 'LineWidth', 1.5, 'DisplayName', 'heading');
    ylabel('theta [deg]');
    xlabel('Time [s]');
    title('Y Position / Heading');
    legend('show');
    saveas(gcf, fullfile(save_dir, 'position_heading.png'));
    
    % Figure 3: Throttle Input
    f3 = figure('Name', 'Throttle', 'Position', [200, 200, 800, 400]);
    grid on;
    box on;
    hold on;
    stairs(t, throttle1, 'b', 'LineWidth', 1.5, 'DisplayName', 'Car 1');
    if isfield(result, 'otherCar')
        throttle2 = result.otherCar.U(2,:);
        stairs(t, throttle2, 'r', 'LineWidth', 1.5, 'DisplayName', 'Car 2');
    end
    xlabel('Time [s]');
    ylabel('u_T');
    title('Throttle');
    legend('show');
    saveas(gcf, fullfile(save_dir, 'throttle.png'));
    
    % Figure 4: Steering Input
    f4 = figure('Name', 'Steering', 'Position', [250, 250, 800, 400]);
    grid on;
    box on;
    hold on;
    stairs(t, delta1, 'b', 'LineWidth', 1.5, 'DisplayName', 'Car 1');
    xlabel('Time [s]');
    ylabel('delta [deg]');
    title('Steering');
    legend('show');
    saveas(gcf, fullfile(save_dir, 'steering.png'));
    
    % Figure 5: Disturbance Estimation (if available)
    if isfield(result.myCar, 'Z_hat')
        f5 = figure('Name', 'Disturbance Estimation', 'Position', [300, 300, 800, 400]);
        grid on;
        box on;
        hold on;
        z1 = result.myCar.Z_hat(5,:);
        plot(t, z1, 'b', 'LineWidth', 1.5, 'DisplayName', 'estimated disturbance');
        xlabel('Time [s]');
        ylabel('d');
        title('Disturbance Estimation');
        legend('show');
        saveas(gcf, fullfile(save_dir, 'disturbance.png'));
    else
        f5 = [];
    end
    
    % Figure 6: Relative Distance (if other car exists)
    if isfield(result, 'otherCar')
        x2 = result.otherCar.X(1,:);
        f6 = figure('Name', 'Relative Distance', 'Position', [350, 350, 800, 400]);
        grid on;
        box on;
        hold on;
        plot(t, x2-x1, 'b', 'LineWidth', 1.5, 'DisplayName', 'relative distance');
        xlabel('Time [s]');
        ylabel('dist [m]');
        title('Relative Distance to Car 2');
        legend('show');
        saveas(gcf, fullfile(save_dir, 'relative_distance.png'));
    else
        f6 = [];
    end
    
    fprintf('All figures saved in %s\n', save_dir);
end