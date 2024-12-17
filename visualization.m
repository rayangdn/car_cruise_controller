function visualization(car, result)
    % Parameters for Shifting Window
    windowWidth = 50;      % Width of the shifting window (horizontal)
    windowHeight = 4;      % Height of the shifting window (fixed vertical range)
    blockLength = 3.0;     % Length of each block in the dashed line
    blockSpacing = 6.0;    % Spacing between blocks
    carWidth = car.length; % Width of the car block
    carHeight = car.width; % Height of the car block
    roadLength = 2000;     % Total length of the road
    laneWidth = 3;         % Lane width for single lane

    plotGapW = 0.1;
    plotGapH = 0.075;
    
    n = 0;
    t = interpolate(result.T,n);
    Ts = t(2);

    x1 = interpolate(result.myCar.X(1,:),n);
    y1 = interpolate(result.myCar.X(2,:),n);
    theta1 = interpolate(result.myCar.X(3,:),n);
    v1 = interpolate(result.myCar.X(4,:),n) * 3.6;

    delta1 = rad2deg(interpolate(result.myCar.U(1,:),n));
    throttle1 = interpolate(result.myCar.U(2,:),n);

    if isfield(result.myCar, 'Ref')
        refy1 = interpolate(result.myCar.Ref(1,:),n);
        refv1 = interpolate(result.myCar.Ref(2,:),n) * 3.6;
    end

    if isfield(result.myCar, 'Z_hat')
        z1 = interpolate(result.myCar.Z_hat(5,:),n);
    end

    if isfield(result, 'otherCar')
        x2 = interpolate(result.otherCar.X(1,:),n);
        y2 = interpolate(result.otherCar.X(2,:),n);
        theta2 = interpolate(result.otherCar.X(3,:),n);
        v2 = interpolate(result.otherCar.X(4,:),n) * 3.6;

        delta2 = rad2deg(interpolate(result.otherCar.U(1,:),n));
        throttle2 = interpolate(result.otherCar.U(2,:),n);
    end

    numPoints = (length(result.T) - 1)*(n+1) + 1;
    
    %% Plot the result based on data
    
    % Define the figure
    fig = figure;
    set(fig, 'Position', [500, 400, 1100, 1000]);

    animPanel = uipanel(fig, 'Units', 'normalized', 'Position', [0, 0.75, 1, 0.25]);

    % First plot animation
    animationAx = axes(animPanel, 'Units', 'normalized', 'Position', [plotGapW/2, 0.2, 1.0-plotGapW, 0.8]);
    box on;
    hold(animationAx, 'on');
    axis(animationAx, 'equal');
    xlabel(animationAx,'X Position');
    ylabel(animationAx,'Y Position');
    title(animationAx,'Result');
    xlim(animationAx,[x1(1) - windowWidth / 2, x1(1) + windowWidth / 2]);
    ylim(animationAx,[-laneWidth/2-1, laneWidth*1.5+1]);
    % Initialize the car
    carBlock1 = patch('Parent', animationAx,'XData', [], 'YData', [], 'FaceColor', 'blue');
    timeDisplay = uicontrol('Parent', animPanel, 'Style', 'text', 'Position', [230, 20, 80, 20],'String', 'Time: 0.00 s','FontSize', 12, 'HorizontalAlignment', 'left');
    distanceDisplay = uicontrol('Parent', animPanel, 'Style', 'text', 'Position', [320, 20, 110, 20],'String', 'Distance: 0.00 m','FontSize', 12, 'HorizontalAlignment', 'left');
    displaySpeed1 = uicontrol('Parent', animPanel, 'Style', 'text', 'Position', [440, 20, 100, 20],'String', 'Velocity: 0.00 m/s','FontSize', 12, 'HorizontalAlignment', 'left', 'ForegroundColor','blue');
    if isfield(result, 'otherCar')
        carBlock2 = patch('Parent', animationAx,'XData', [], 'YData', [], 'FaceColor', 'red');
        displaySpeed2 = uicontrol('Parent', animPanel, 'Style', 'text', 'Position', [550, 20, 100, 20],'String', 'Velocity: 0.00 m/s','FontSize', 12, 'HorizontalAlignment', 'left', 'ForegroundColor','red');
    end
    car1Vertices = getCarVertices(x1(1), y1(1), theta1(1), carWidth, carHeight);
    set(carBlock1, 'XData', car1Vertices(:, 1), 'YData', car1Vertices(:, 2));
    set(displaySpeed1, 'String', sprintf('Velocity: %.2f  km/h', v1(1)));
    if isfield(result, 'otherCar')
        car2Vertices = getCarVertices(x2(1), y2(1), theta2(1), carWidth, carHeight);
        set(carBlock2, 'XData', car2Vertices(:, 1), 'YData', car2Vertices(:, 2));
        set(displaySpeed2, 'String', sprintf('Velocity: %.2f  km/h', v2(1)));
    end
    % Plot the dashed line
    numBlocks = ceil(roadLength / (blockLength + blockSpacing));
    leftLaneBlocks = gobjects(numBlocks, 1);
    for i = 1:numBlocks
        xPos = -roadLength / 2 + (i - 1) * (blockLength + blockSpacing);
        leftLaneBlocks(i) = rectangle(animationAx,'Position', [xPos, laneWidth/2-0.1, blockLength, 0.2], ...
                                      'FaceColor', 'black', 'EdgeColor', 'none');
    end
    plot(animationAx,[-windowWidth/2,roadLength],[laneWidth*3/2,laneWidth*3/2],'Color', 'black', 'LineWidth', 2);
    plot(animationAx,[-windowWidth/2,roadLength],[-laneWidth/2,-laneWidth/2],'Color', 'black', 'LineWidth', 2);
    
    % Second plot velocity
    speedplotAx = axes('Units', 'normalized', 'Position', [plotGapW/2, 0.5+plotGapH/2, 0.5-plotGapW, 0.25-plotGapH]);
    grid on;
    box on;
    hold(speedplotAx,'on');
    xlabel(speedplotAx, 'Time [s]');
    ylabel(speedplotAx, 'V [km/h]');
    title(speedplotAx, 'Velocity');
    xlim(speedplotAx, [0, t(end)]);
    ylim(speedplotAx, [min(v1) - 5, max(v1) + 5]);
    plot(speedplotAx, t, v1, 'b', 'LineWidth', 1.5, 'DisplayName', 'Car 1'); % Blue for car 1
    if isfield(result.myCar, 'Ref')
        yl = ylim(speedplotAx);
        ylim(speedplotAx, [min(yl(1), min(refv1) - 5), max(yl(2), max(refv1) + 5)]);
        stairs(speedplotAx, t, refv1, '--b', 'LineWidth', 1.5, 'DisplayName', 'Car 1 reference'); % Blue for car 1
    end
    if isfield(result, 'otherCar')
        yl = ylim(speedplotAx);
        ylim(speedplotAx, [min(yl(1), min(v2) - 5), max(yl(2), max(v2) + 5)]);
        plot(speedplotAx, t, v2, 'r', 'LineWidth', 1.5, 'DisplayName', 'Car 2'); % Red for car 2
    end
    legend(speedplotAx, 'show');

    % Third plot y pos
    inputplotAx = axes('Units', 'normalized', 'Position', [0.5+plotGapW/2, 0.5+plotGapH/2, 0.5-plotGapW, 0.25-plotGapH]);
    grid on;
    box on;
    hold(inputplotAx,'on');
    xlabel(inputplotAx, 'Time [s]');
    title(inputplotAx, 'Y Position / Heading');
    xlim(inputplotAx, [0, t(end)]);
    yyaxis left
    ylabel('y [m]');
    ylim(inputplotAx, [min(-0.5, min(y1)), max(3.5, max(y1))]);
    plot(inputplotAx, t, y1, 'b', 'LineWidth', 1.5, 'DisplayName', 'Car 1 y pos'); % Blue for car 1
    if isfield(result.myCar, 'Ref')
        stairs(inputplotAx, t, refy1, '--b', 'LineWidth', 1.5, 'DisplayName', 'Car 1 reference'); % Blue for car 1
    end
    % if isfield(result, 'otherCar')
    %     plot(inputplotAx, t, y2, 'r', 'LineWidth', 1.5, 'DisplayName', 'Car 2'); % Red for car 2
    % end
    yyaxis right
    ylabel('theta [deg]');
    if min(rad2deg(theta1)) < -1
        ylim([min(-8, min(rad2deg(theta1))), max(8, max(rad2deg(theta1)))]);
    else
        ylim([min(-1, min(rad2deg(theta1))), max(7, max(rad2deg(theta1)))]);
    end
    plot(t, rad2deg(theta1), 'r', 'LineWidth', 1.5, 'DisplayName', 'Car 1 heading');
    legend(inputplotAx, 'show');

    % Forth plot inputs
    inputplotAx = axes('Units', 'normalized', 'Position', [plotGapW/2, 0.25+plotGapH/2, 0.5-plotGapW, 0.25-plotGapH]);
    grid on;
    box on;
    hold(inputplotAx,'on');
    xlabel(inputplotAx, 'Time [s]');
    title(inputplotAx, 'Throttle');
    xlim(inputplotAx, [0, t(end)]);
    ylabel('u_T');
    ylim(inputplotAx, [min(throttle1) - 0.1, max(throttle1) + 0.1]);
    stairs(inputplotAx, t, throttle1, 'b', 'LineWidth', 1.5, 'DisplayName', 'Car 1'); % Blue for car 1
    if isfield(result, 'otherCar')
        yl = ylim(inputplotAx);
        ylim(inputplotAx, [min(yl(1), min(throttle2) - 0.1), max(yl(2), max(throttle2) + 0.1)]);
        stairs(inputplotAx, t, throttle2, 'r', 'LineWidth', 1.5, 'DisplayName', 'Car 2'); % Red for car 2
    end
    legend(inputplotAx, 'show');

    % Fifth plot y
    inputplotAx = axes('Units', 'normalized', 'Position', [0.5+plotGapW/2, 0.25+plotGapH/2, 0.5-plotGapW, 0.25-plotGapH]);
    grid on;
    box on;
    hold(inputplotAx,'on');
    xlabel(inputplotAx, 'Time [s]');
    title(inputplotAx, 'Steering');
    xlim(inputplotAx, [0, t(end)]);
    ylabel('delta [deg]');
    ylim([min(delta1)-0.5, max(delta1) + 0.5]);
    stairs(inputplotAx, t, delta1, 'b', 'LineWidth', 1.5, 'DisplayName', 'Car 1'); % Blue for car 1
    % if isfield(result, 'otherCar')
    %     stairs(inputplotAx, t, delta2, 'r', 'LineWidth', 1.5, 'DisplayName', 'Car 2'); % Red for car 2
    % end
    legend(inputplotAx, 'show');

    % Sixth plot esitmation
    disturbanceplotAx = axes('Units', 'normalized', 'Position', [plotGapW/2, plotGapH/2, 0.5-plotGapW, 0.25-plotGapH]);
    grid on;
    box on;
    hold(disturbanceplotAx,'on');
    xlabel(disturbanceplotAx, 'Time [s]');
    ylabel(disturbanceplotAx, 'd');
    title(disturbanceplotAx, 'Disturbance Estimation');
    xlim(disturbanceplotAx, [0, t(end)]);
    if isfield(result.myCar, 'Z_hat')
        ylim(disturbanceplotAx, [min(z1)-0.05, max(z1)+0.05]);
        plot(disturbanceplotAx, t, z1, 'b', 'LineWidth', 1.5, 'DisplayName', 'estimated disturbance'); % Blue for car 1
        legend(disturbanceplotAx, 'show');
    end

    disDiffplotAx = axes('Units', 'normalized', 'Position', [0.5+plotGapW/2, plotGapH/2, 0.5-plotGapW, 0.25-plotGapH]);
    grid on;
    box on;
    hold(disDiffplotAx,'on');
    xlabel(disDiffplotAx, 'Time [s]');
    ylabel(disDiffplotAx, 'dist [m]');
    title(disDiffplotAx, 'Relative Distance to Car 2');
    xlim(disDiffplotAx, [0, t(end)]);
    if isfield(result, 'otherCar')
        dist = sqrt((x2-x1).^2 + (y2-y1).^2);
        ylim(disDiffplotAx, [min(min(dist), -0.1), max(dist) + 5]);
        plot(disDiffplotAx, t, dist, 'b', 'LineWidth', 1.5, 'DisplayName', 'relative distance'); % Blue for car 1
        legend(disDiffplotAx, 'show');
    end

    % Initialize video writer
    % video = VideoWriter('car_animation.mp4', 'MPEG-4'); % Create video file
    % video.FrameRate = 24; % Set frame rate (adjust as needed)
    % open(video); % Open the video writer

    % Create the buttons
    currentIteration = 1;
    isPaused = false;
    function pauseAnimation(~, ~)
        set(startButton, 'String', 'Continue', 'Callback',@continueAnimation);
        isPaused = true;
    end

    function continueAnimation(~, ~)
        set(startButton, 'String', 'Pause', 'Callback',@pauseAnimation);
        isPaused = false;
        runAnimation();
    end

    function startAnimation(~, ~)
        set(startButton, 'String', 'Pause','Callback', @pauseAnimation);
        restartButton = uicontrol('Parent', animPanel, 'Style', 'pushbutton', 'String', 'Restart', 'FontSize', 12,'Position', [120 10 100 40], 'Callback', @restartAnimation);
        runAnimation();
    end

    function restartAnimation(~, ~)
        currentIteration = 1;
        isPaused = false;
        set(startButton, 'String', 'Pause','Callback', @pauseAnimation);
        runAnimation();
    end
    
    startButton = uicontrol('Parent', animPanel, 'Style', 'pushbutton', 'String', 'Start', 'FontSize', 12,'Position', [10 10 100 40], 'Callback', @startAnimation);

    function runAnimation()
        while currentIteration <= numPoints-1
            if ~isPaused
                tic;

                % Current position of the car
                car1Vertices = getCarVertices(x1(currentIteration), y1(currentIteration), theta1(currentIteration), carWidth, carHeight);
                set(carBlock1, 'XData', car1Vertices(:, 1), 'YData', car1Vertices(:, 2));
                set(displaySpeed1, 'String', sprintf('Velocity: %.2f  km/h', v1(currentIteration)));
                if isfield(result, 'otherCar')
                    car2Vertices = getCarVertices(x2(currentIteration), y2(currentIteration), theta2(currentIteration), carWidth, carHeight);
                    set(carBlock2, 'XData', car2Vertices(:, 1), 'YData', car2Vertices(:, 2));
                    set(displaySpeed2, 'String', sprintf('Velocity: %.2f  km/h', v2(currentIteration)));
                end

                % fixing the window to the first car
                xlim(animationAx,[x1(currentIteration) - windowWidth / 2, x1(currentIteration) + windowWidth / 2]);
                set(timeDisplay,'String', sprintf('Time: %.2f s',t(currentIteration)));
                set(distanceDisplay,'String', sprintf('Distance: %.2f m',x1(currentIteration)));
                currentIteration = currentIteration + 1;
                drawnow;
                
                elapsed = toc;
                if elapsed < Ts
                    pause(Ts - elapsed);
                end
                % frame = getframe(fig);
                % writeVideo(video, frame);
            else
                break;
            end
        end
    end

end

function vertices = getCarVertices(x, y, heading, carWidth, carHeight)
    % Get the car's vertices in its local frame
    halfWidth = carWidth / 2;
    halfHeight = carHeight / 2;
    localVertices = [
        -halfWidth, -halfHeight;
        halfWidth, -halfHeight;
        halfWidth, halfHeight;
        -halfWidth, halfHeight
    ];

    % Rotate the vertices based on the heading angle
    rotationMatrix = [
        cos(heading), -sin(heading);
        sin(heading), cos(heading)
    ];
    rotatedVertices = (rotationMatrix * localVertices')';

    % Translate the vertices to the car's position
    vertices = rotatedVertices + [x, y];
end

%% Functions
function interpolated_x = interpolate(x, n)
    interpolated_x = [];
    for i = 1:length(x)-1
        xi = x(i);
        xip1 = x(i+1);
        segment = linspace(xi, xip1, n + 2);
        interpolated_x = [interpolated_x, segment(1:end-1)];
    end
    interpolated_x = [interpolated_x, x(end)];
end
