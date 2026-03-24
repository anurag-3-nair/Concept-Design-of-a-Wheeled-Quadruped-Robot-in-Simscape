clear; clc; close all;

% parameters
MAX_X = 100;
MAX_Y = 100;
MAP = 2 * (ones(MAX_X, MAX_Y));

% TOGGLE: true = random obstacle generation, false = fixed map
useRandomObstacles = false;

% synthetic obstacles
numRects = 6; % rectangles
numCircs = 6; % circles
[X, Y] = meshgrid(1:MAX_Y, 1:MAX_X);
BW = false(size(X));

if useRandomObstacles == true
    % random obstacle placement
    % rectangles
    for i = 1:numRects
        x1 = randi([5, MAX_X-25]);
        y1 = randi([5, MAX_Y-25]);
        w  = randi([8, 20]);
        h  = randi([8, 20]);
        BW(Y > y1 & Y < y1+h & X > x1 & X < x1+w) = true;
    end
    % circles
    for i = 1:numCircs
        cx = randi([10, MAX_X-10]);
        cy = randi([10, MAX_Y-10]);
        r  = randi([5, 12]);
        BW((X-cx).^2 + (Y-cy).^2 < r^2) = true;
    end

% fixed map
else
    % rectangles
    BW(Y>20 & Y<35 & X>15 & X<40) = true;
    BW(Y>58 & Y<75 & X>32 & X<55) = true;
    BW(Y>20 & Y<40 & X>65 & X<85) = true;

    % circles
    BW((X-30).^2 + (Y-75).^2 < 9^2) = true;
    BW((X-70).^2 + (Y-60).^2 < 9^2) = true;
    BW((X-50).^2 + (Y-25).^2 < 9^2) = true;
end

% defining start and goal
xStart  = 5;  yStart  = 5;
xTarget = 95; yTarget = 95;

% robot footprint inflation — increase for more clearance (5-10 recommended)
robot_radius = 8;
se = strel('disk', robot_radius);
map_inflated = imdilate(BW, se);

% ensure start and target are never blocked
map_inflated(xStart,  yStart)  = 0;
map_inflated(xTarget, yTarget) = 0;

% map values
MAP(map_inflated) = -1;
MAP(xStart,  yStart)  = 1;
MAP(xTarget, yTarget) = 0;

% map plot — three-tone: black = obstacle, grey = safety margin, white = free
figure('Position', [100 100 900 800]);
display_map = ones(MAX_X, MAX_Y);
display_map(map_inflated) = 0.6;   % grey = inflation/safety margin
display_map(BW)           = 0;     % black = actual obstacle
imshow(display_map, 'InitialMagnification', 'fit');
hold on;
axis([1 MAX_X+1 1 MAX_Y+1]);
axis on; grid on;

plot(yStart,  xStart,  'bo', 'MarkerSize', 8, 'LineWidth', 2);
text(yStart+2, xStart,  'Start',  'Color', 'blue');
plot(yTarget, xTarget, 'gd', 'MarkerSize', 8, 'LineWidth', 2);
text(yTarget+2, xTarget, 'Target', 'Color', 'green');

set(gca, 'YDir', 'normal');
title("Synthetic Obstacle Map - Computing the path ...");
drawnow;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialising the open/closed lists for A*
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

OPEN   = [];
CLOSED = [];
maxIterations  = 2e5;
iterationCount = 0;

% pre-populate CLOSED with all obstacle cells
k = 1;
for i = 1:MAX_X
    for j = 1:MAX_Y
        if MAP(i,j) == -1
            CLOSED(k,1) = i;
            CLOSED(k,2) = j;
            k = k + 1;
        end
    end
end
CLOSED_COUNT = size(CLOSED, 1);

% set starting node
xNode     = xStart;
yNode     = yStart;
OPEN_COUNT = 1;
path_cost  = 0;
goal_distance = distance(xNode, yNode, xTarget, yTarget);

OPEN(OPEN_COUNT, :) = insert_open(xNode, yNode, xNode, yNode, path_cost, goal_distance, goal_distance);
OPEN(OPEN_COUNT, 1) = 0;

CLOSED_COUNT = CLOSED_COUNT + 1;
CLOSED(CLOSED_COUNT, 1) = xNode;
CLOSED(CLOSED_COUNT, 2) = yNode;
NoPath = 1;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A* SEARCH
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

while (xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1

    exp_array = expand_array(xNode, yNode, path_cost, xTarget, yTarget, CLOSED, MAX_X, MAX_Y, MAP);
    exp_count = size(exp_array, 1);
    iterationCount = iterationCount + 1;

    if iterationCount > maxIterations
        warning('Max iterations reached — aborting search.');
        break;
    end

    % update OPEN with successor nodes
    for i = 1:exp_count
        flag = 0;
        for j = 1:OPEN_COUNT
            if exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3)
                OPEN(j,8) = min(OPEN(j,8), exp_array(i,5));
                if OPEN(j,8) == exp_array(i,5)
                    OPEN(j,4) = xNode;
                    OPEN(j,5) = yNode;
                    OPEN(j,6) = exp_array(i,3);
                    OPEN(j,7) = exp_array(i,4);
                end
                flag = 1;
                break;
            end
        end
        if flag == 0
            OPEN_COUNT = OPEN_COUNT + 1;
            OPEN(OPEN_COUNT,:) = insert_open(exp_array(i,1), exp_array(i,2), xNode, yNode, exp_array(i,3), exp_array(i,4), exp_array(i,5));
        end
    end

    % pick node with smallest f(n)
    index_min_node = min_fn(OPEN, OPEN_COUNT, xTarget, yTarget);
    if index_min_node ~= -1
        xNode     = OPEN(index_min_node, 2);
        yNode     = OPEN(index_min_node, 3);
        path_cost = OPEN(index_min_node, 6);

        CLOSED_COUNT = CLOSED_COUNT + 1;
        CLOSED(CLOSED_COUNT, 1) = xNode;
        CLOSED(CLOSED_COUNT, 2) = yNode;
        OPEN(index_min_node, 1) = 0;
    else
        NoPath = 0;
    end
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PATH RECONSTRUCTION
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if xNode == xTarget && yNode == yTarget

    % backtrack from goal to start via parent pointers
    Optimal_path = [];
    xval = CLOSED(end, 1);
    yval = CLOSED(end, 2);

    Optimal_path(1,1) = xval;
    Optimal_path(1,2) = yval;
    i = 2;

    parent_x = OPEN(node_index(OPEN, xval, yval), 4);
    parent_y = OPEN(node_index(OPEN, xval, yval), 5);

    while parent_x ~= xStart || parent_y ~= yStart
        Optimal_path(i,1) = parent_x;
        Optimal_path(i,2) = parent_y;
        inode    = node_index(OPEN, parent_x, parent_y);
        parent_x = OPEN(inode, 4);
        parent_y = OPEN(inode, 5);
        i = i + 1;
    end

    % append start node then reverse so path runs Start → Goal
    Optimal_path(i,1) = xStart;
    Optimal_path(i,2) = yStart;
    Optimal_path = flipud(Optimal_path);

    % path validation
    path_valid = true;
    for idx = 1:size(Optimal_path,1)
        if MAP(Optimal_path(idx,1), Optimal_path(idx,2)) == -1
            fprintf('WARNING: Path goes through obstacle at (%d, %d)!\n', ...
                Optimal_path(idx,1), Optimal_path(idx,2));
            path_valid = false;
        end
    end
    if path_valid
        fprintf('Path validation: PASS\n');
    end

    % -----------------------------------------------------------------------
    % Waypoint generation for pure pursuit (in-workspace, no CSV)
    % -----------------------------------------------------------------------
    cellSize = 1.0;
    originX  = 0.0;
    originY  = 0.0;

    % convert grid (row, col) → world (x, y)
    xWorld = originX + (Optimal_path(:,2) - 0.5) * cellSize;
    yWorld = originY + (Optimal_path(:,1) - 0.5) * cellSize;

    % smooth and resample at uniform spacing for pure pursuit
    t         = cumsum([0; sqrt(diff(xWorld).^2 + diff(yWorld).^2)]);
    t_uniform = (0 : 1.0 : t(end))';          % 1-cell spacing;
    xWP       = interp1(t, xWorld, t_uniform);
    yWP       = interp1(t, yWorld, t_uniform);

    % waypoints matrix 
    waypoints = [xWP, yWP];
    fprintf('Waypoints generated: %d points\n', size(waypoints, 1));

    % -----------------------------------------------------------------------
    % Animated path + final plot
    % -----------------------------------------------------------------------
    j = size(Optimal_path, 1);
    p = plot(Optimal_path(1,2)+.5, Optimal_path(1,1)+.5, 'bo', 'MarkerSize', 8, 'LineWidth', 2);

    for idx = 2:j
        pause(0.05);
        set(p, 'XData', Optimal_path(idx,2)+.5, 'YData', Optimal_path(idx,1)+.5);
        drawnow;
    end

    plot(Optimal_path(:,2)+.5, Optimal_path(:,1)+.5, 'c-', 'LineWidth', 2.5);
    title("Synthetic Obstacle Map - Path Found");

else
    pause(1);
    h = msgbox('Sorry, No path exists to the Target!', 'warn');
    uiwait(h, 5);
end