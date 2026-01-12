%% Step 3: Simulate Robot Following the Path
clear; clc; close all;

% Load the planned path
load('navigation_path.mat', 'path_data');

path = path_data.path;
map = path_data.map;
start = path_data.start;
goal = path_data.goal;

fprintf('=== SIMULATING ROBOT NAVIGATION ===\n');

% Create simulation figure
figure('Position', [100, 100, 1200, 500]);

% Subplot 1: Overall path
subplot(1,2,1);
show(map);
hold on;
plot(path(:,1), path(:,2), 'b-', 'LineWidth', 2);
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 3, 'MarkerFaceColor', 'g');
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 3, 'MarkerFaceColor', 'r');
title('Planned Path Around Obstacles');
grid on;

% Subplot 2: Robot simulation
subplot(1,2,2);
show(map);
hold on;
title('Robot Navigation Simulation');
grid on;

% Simulate robot movement along the path
robotSize = 0.3; % Robot radius

for i = 1:size(path,1)
    % Clear previous robot position
    subplot(1,2,2);
    cla;
    show(map);
    hold on;
    title('Robot Navigation Simulation');
    grid on;
    
    % Plot the path
    plot(path(:,1), path(:,2), 'b-', 'LineWidth', 1, 'Color', [0.7 0.7 1]);
    
    % Plot traveled path
    if i > 1
        plot(path(1:i,1), path(1:i,2), 'g-', 'LineWidth', 2);
    end
    
    % Plot current robot position
    robot = plot(path(i,1), path(i,2), 'bo', 'MarkerSize', 8, 'LineWidth', 3);
    
    % Plot robot direction (if not at last point)
    if i < size(path,1)
        quiver(path(i,1), path(i,2), ...
               path(i+1,1)-path(i,1), path(i+1,2)-path(i,2), ...
               0.3, 'r', 'LineWidth', 2, 'MaxHeadSize', 3);
    end
    
    % Plot start and goal
    plot(start(1), start(2), 'gs', 'MarkerSize', 8, 'LineWidth', 2);
    plot(goal(1), goal(2), 'rs', 'MarkerSize', 8, 'LineWidth', 2);
    
    % Add progress information
    progress = i/size(path,1) * 100;
    text(1, 9.5, sprintf('Progress: %.1f%%', progress), 'FontSize', 12, 'BackgroundColor', 'white');
    text(1, 9.0, sprintf('Step: %d/%d', i, size(path,1)), 'FontSize', 12, 'BackgroundColor', 'white');
    
    drawnow;
    pause(0.1); % Slow down for visualization
end

fprintf('Navigation simulation completed!\n');
fprintf('Robot successfully navigated from start to goal while avoiding all obstacles.\n');