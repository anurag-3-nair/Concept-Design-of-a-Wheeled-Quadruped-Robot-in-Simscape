%% Step 1: Create Environment with Multiple Obstacles
clear; clc; close all;

% Create a 10x10 meter occupancy map
map = binaryOccupancyMap(10, 10, 20); % 20 cells/meter resolution

% Define various obstacles (stones, walls, etc.) - MAKE SURE THERE'S A PATH
obstacles = [
    % Large obstacles that force detours
    3, 3, 1.0;    % Large stone center (3,3), radius 1m
    7, 2, 0.8;    % Stone blocking direct path
    2, 7, 0.6;    % Stone in the middle
    6, 6, 0.7;    % Another stone
    8, 8, 0.5;    % Stone near goal
    4, 5, 0.4;    % Small stone
    5, 8, 0.3     % Small stone
];

% Add obstacles to the map
for i = 1:size(obstacles, 1)
    % Create circular obstacle
    [x, y] = meshgrid(0:0.05:10);
    inObstacle = (x - obstacles(i,1)).^2 + (y - obstacles(i,2)).^2 <= obstacles(i,3)^2;
    obstaclePoints = [x(inObstacle), y(inObstacle)];
    if ~isempty(obstaclePoints)
        setOccupancy(map, obstaclePoints, ones(size(obstaclePoints,1),1));
    end
end

% Add some wall-like obstacles
wall1 = [4, 0, 0.2, 3]; % [x, y, width, height] - vertical wall
wall2 = [0, 6, 5, 0.2]; % horizontal wall

wall1Points = [];
for y = wall1(2):0.05:(wall1(2)+wall1(4))
    for x = wall1(1):0.05:(wall1(1)+wall1(3))
        wall1Points = [wall1Points; x, y];
    end
end

wall2Points = [];
for y = wall2(2):0.05:(wall2(2)+wall2(4))
    for x = wall2(1):0.05:(wall2(1)+wall2(3))
        wall2Points = [wall2Points; x, y];
    end
end

setOccupancy(map, wall1Points, ones(size(wall1Points,1),1));
setOccupancy(map, wall2Points, ones(size(wall2Points,1),1));

% Visualize the environment
figure(1)
show(map)
title('Environment with Multiple Obstacles')
hold on;
grid on;

% Plot obstacle centers
plot(obstacles(:,1), obstacles(:,2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);