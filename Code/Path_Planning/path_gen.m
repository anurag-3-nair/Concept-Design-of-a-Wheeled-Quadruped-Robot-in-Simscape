clear; clc; close all;

% parameters
MAX_X = 100;
MAX_Y = 100;
MAP = 2 * (ones(MAX_X, MAX_Y));

% adjusts obstacle size and safety buffer together
% 1.0 = original sizes, 0.7 = smaller obstacles + tighter buffer
scale_factor = 0.8;  % tune between 0.6 and 1.2

% TOGGLE: true = random obstacle generation, false = fixed map
useRandomObstacles = false;

% synthetic obstacles
numRects = 6;
numCircs = 6;
[X, Y] = meshgrid(1:MAX_Y, 1:MAX_X);
BW = false(size(X));

if useRandomObstacles == true
    for i = 1:numRects
        x1 = randi([5, MAX_X-25]);
        y1 = randi([5, MAX_Y-25]);
        w  = round(randi([8,  20]) * scale_factor);
        h  = round(randi([8,  20]) * scale_factor);
        BW(Y > y1 & Y < y1+h & X > x1 & X < x1+w) = true;
    end
    for i = 1:numCircs
        cx = randi([10, MAX_X-10]);
        cy = randi([10, MAX_Y-10]);
        r  = round(randi([5, 12]) * scale_factor);
        BW((X-cx).^2 + (Y-cy).^2 < r^2) = true;
    end

else
    % fixed obstacle centres
    % rectangles
    cx1=27; cy1=27; hw1=round(12*scale_factor); hh1=round(7*scale_factor);
    BW(Y>cy1-hh1 & Y<cy1+hh1 & X>cx1-hw1 & X<cx1+hw1) = true;

    cx2=43; cy2=66; hw2=round(11*scale_factor); hh2=round(8*scale_factor);
    BW(Y>cy2-hh2 & Y<cy2+hh2 & X>cx2-hw2 & X<cx2+hw2) = true;

    cx3=75; cy3=30; hw3=round(10*scale_factor); hh3=round(10*scale_factor);
    BW(Y>cy3-hh3 & Y<cy3+hh3 & X>cx3-hw3 & X<cx3+hw3) = true;

    % circles
    BW((X-30).^2 + (Y-75).^2 < (round(9*scale_factor))^2) = true;
    BW((X-70).^2 + (Y-60).^2 < (round(9*scale_factor))^2) = true;
    BW((X-50).^2 + (Y-25).^2 < (round(9*scale_factor))^2) = true;
end

% defining start and goal
xStart  = 5;  yStart  = 5;
xTarget = 95; yTarget = 95;

% robot safety buffer — also scales with scale_factor
% base_radius is the "true" robot footprint at scale 1.0
base_radius  = 6;
robot_radius = round(base_radius * scale_factor);
robot_radius = max(robot_radius, 2);   % never drop below 2

se           = strel('disk', robot_radius);
map_inflated = imdilate(BW, se);

% ensure start and target are never blocked
map_inflated(xStart,  yStart)  = 0;
map_inflated(xTarget, yTarget) = 0;

% map values
MAP(map_inflated) = -1;
MAP(xStart,  yStart)  = 1;
MAP(xTarget, yTarget) = 0;

% map plot: black = obstacle, grey = safety margin, white = free
figure('Position', [100 100 900 800]);
display_map = ones(MAX_X, MAX_Y);
display_map(map_inflated) = 0.6;
display_map(BW)           = 0;
imshow(display_map, 'InitialMagnification', 'fit');
hold on;
axis([1 MAX_X+1 1 MAX_Y+1]);
axis on; grid on;

plot(yStart,  xStart,  'bo', 'MarkerSize', 8, 'LineWidth', 2);
text(yStart+2, xStart,  'Start',  'Color', 'blue');
plot(yTarget, xTarget, 'gd', 'MarkerSize', 8, 'LineWidth', 2);
text(yTarget+2, xTarget, 'Target', 'Color', 'green');

set(gca, 'YDir', 'normal');
title(sprintf('Obstacle Map  |  scale = %.1f  |  buffer = %d cells  —  Computing ...', ...
      scale_factor, robot_radius));
drawnow;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialising the open/closed lists for A*
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

OPEN   = [];
CLOSED = [];
maxIterations  = 2e5;
iterationCount = 0;

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

    Optimal_path(i,1) = xStart;
    Optimal_path(i,2) = yStart;
    Optimal_path = flipud(Optimal_path);

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

    % waypoint generation for pure pursuit 
    cellSize = 0.5;
    originX  = 0.0;
    originY  = 0.0;

    xWorld = originX + (Optimal_path(:,2) - 0.5) * cellSize;
    yWorld = originY + (Optimal_path(:,1) - 0.5) * cellSize;

    t         = cumsum([0; sqrt(diff(xWorld).^2 + diff(yWorld).^2)]);
    t_uniform = (0 : 1.0 : t(end))';
    xWP       = interp1(t, xWorld, t_uniform);
    yWP       = interp1(t, yWorld, t_uniform);

    waypoints = [xWP, yWP];
    fprintf('Waypoints generated: %d points\n', size(waypoints, 1));

    % animated final path plot
    j = size(Optimal_path, 1);
    p = plot(Optimal_path(1,2)+.5, Optimal_path(1,1)+.5, 'bo', 'MarkerSize', 8, 'LineWidth', 2);

    for idx = 2:j
        pause(0.05);
        set(p, 'XData', Optimal_path(idx,2)+.5, 'YData', Optimal_path(idx,1)+.5);
        drawnow;
    end

    plot(Optimal_path(:,2)+.5, Optimal_path(:,1)+.5, 'c-', 'LineWidth', 2.5);
    title(sprintf('Path Found  |  scale = %.1f  |  buffer = %d cells  |  %d waypoints', ...
          scale_factor, robot_radius, size(waypoints,1)));

else
    pause(1);
    h = msgbox('Sorry, No path exists to the Target!', 'warn');
    uiwait(h, 5);
end


% clear; clc; close all;
% scale = 0.5;
% 
% % parameters
% MAX_X = round(100*scale); 
% MAX_Y = round(100*scale);
% MAP = 2*(ones(MAX_X, MAX_Y));
% 
% % TOGGLE: true = random obstacle generation, false = fixed map
% useRandomObstacles = false;
% 
% % synthetic obstacles
% numRects = 6; % rectangles
% numCircs = 6; % circles
% [X, Y] = meshgrid(1:MAX_Y, 1:MAX_X);
% BW = false(size(X));
% 
% if useRandomObstacles == true
%     % random obstacle placement
%     % rectangles
%     for i = 1:numRects
%         x1 = round(randi([5, 100-25])*scale);
%         y1 = round(randi([5, 100-25])*scale);
%         w = round(randi([8, 20])*scale);
%         h = round(randi([8, 20])*scale);
%         BW(Y > y1 & Y < y1+h & X > x1 & X < x1+w) = true;
%     end
% 
%     % circles
%     for i = 1:numCircs
%         cx = round(randi([10, 100-10])*scale);
%         cy = round(randi([10, 100-10])*scale);
%         r = round(randi([5, 12])*scale);
%         BW((X-cx).^2 + (Y-cy).^2 < r^2) = true;
%     end
% 
% % fixed map
% else
%     % rectangles
%     BW(Y > round(20*scale) & Y < round(35*scale) & X > round(15*scale) & X < round(45*scale)) = true;
%     BW(Y > round(55*scale) & Y < round(75*scale) & X > round(30*scale) & X < round(55*scale)) = true;
%     BW(Y > round(20*scale) & Y < round(40*scale) & X > round(65*scale) & X < round(85*scale)) = true;
% 
%     % circles
%     BW((X-round(30*scale)).^2 + (Y-round(75*scale)).^2 < (round(12*scale))^2) = true;
%     BW((X-round(70*scale)).^2 + (Y-round(60*scale)).^2 < (round(10*scale))^2) = true;
%     BW((X-round(50*scale)).^2 + (Y-round(25*scale)).^2 < (round(10*scale))^2) = true; 
% end
% 
% % defining a start and a goal
% xStart = round(5  * scale);
% yStart = round(5  * scale);
% xTarget = round(95 * scale);
% yTarget = round(95 * scale);
% 
% % robot footprint
% robot_radius = 1;
% 
% % obstacles
% se = strel('disk', robot_radius);
% map_inflated = imdilate(BW, se);
% 
% % error checking
% map_inflated(xStart, yStart) = 0;
% map_inflated(xTarget, yTarget) = 0;
% 
% % map values
% MAP(map_inflated) = -1;
% MAP(xStart, yStart) = 1;
% MAP(xTarget, yTarget) = 0;
% 
% % map plot
% figure('Position', [100 100 900 800]);
% imshow(~map_inflated, 'InitialMagnification','fit');
% hold on;
% axis([1 MAX_X+1 1 MAX_Y+1]);
% axis on; grid on; 
% 
% plot(yStart, xStart, 'bo', 'MarkerSize', 8, 'LineWidth', 2);
% text(yStart+2, xStart, 'Start', 'Color', 'blue');
% plot(yTarget, xTarget, 'gd', 'MarkerSize', 8, 'LineWidth', 2);
% text(yTarget+2, xTarget, 'Target', 'Color', 'green');
% 
% set(gca, 'YDir', 'normal');
% title("Synthetic Obstacle Map - Computing the path ...");
% drawnow;
% 
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Initialising the open lists for A*
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% OPEN=[];
% CLOSED=[];
% maxIterations = 2e5;
% iterationCount = 0;
% 
% % adding obstacles are in the closed list
% k=1; % dummy counter
% for i = 1:MAX_X
%     for j = 1:MAX_Y
%         if(MAP(i,j) == -1)
%             CLOSED(k, 1) = i; 
%             CLOSED(k, 2) = j; 
%             k = k+1;
%         end
%     end
% end
% CLOSED_COUNT = size(CLOSED, 1);
% 
% % setting the starting node as the first node
% xNode = xStart;
% yNode = yStart;
% OPEN_COUNT = 1;
% path_cost = 0;
% goal_distance = distance(xNode, yNode, xTarget, yTarget);
% 
% OPEN(OPEN_COUNT, :) = insert_open(xNode, yNode, xNode, yNode, path_cost, goal_distance, goal_distance);
% OPEN(OPEN_COUNT, 1) = 0;
% 
% CLOSED_COUNT = CLOSED_COUNT + 1;
% CLOSED(CLOSED_COUNT, 1) = xNode;
% CLOSED(CLOSED_COUNT, 2) = yNode;
% NoPath = 1;
% 
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % START ALGORITHM
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
%     exp_array = expand_array(xNode, yNode, path_cost, xTarget, yTarget, CLOSED, MAX_X, MAX_Y, MAP);
%     exp_count = size(exp_array, 1);
%     iterationCount = iterationCount + 1;
% 
%     if iterationCount > maxIterations
%         warning('Max iterations reached, aborting search.');
%         break;
%     end
% 
%     % UPDATE LIST OPEN WITH THE SUCCESSOR NODES
%     for i = 1:exp_count
%         flag = 0;
%         for j = 1:OPEN_COUNT
%             if (exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3))
%                 OPEN(j,8) = min(OPEN(j,8), exp_array(i,5));
% 
%                 % updating the parent nodes if we found a better path
%                 if OPEN(j,8) == exp_array(i,5)
%                     OPEN(j,4) = xNode;
%                     OPEN(j,5) = yNode;
%                     OPEN(j,6) = exp_array(i,3);
%                     OPEN(j,7) = exp_array(i,4);
%                 end
%                 flag = 1;
%                 break;
%             end
%         end
% 
%         % new elements into the open list
%         if flag == 0
%             OPEN_COUNT = OPEN_COUNT + 1;
%             OPEN(OPEN_COUNT,:) = insert_open(exp_array(i,1), exp_array(i,2), xNode, yNode, exp_array(i,3), exp_array(i,4), exp_array(i,5));
%         end
%     end
% 
%     % finding out the node with the smallest fn 
%     index_min_node = min_fn(OPEN, OPEN_COUNT, xTarget, yTarget);
%     if (index_min_node ~= -1)
% 
%         % setting xNode and yNode to the node with minimum fn
%         xNode = OPEN(index_min_node, 2);
%         yNode = OPEN(index_min_node, 3);
%         path_cost = OPEN(index_min_node, 6); % cost update
% 
%         % moving the nodes to CLOSED list
%         CLOSED_COUNT = CLOSED_COUNT + 1;
%         CLOSED(CLOSED_COUNT, 1) = xNode;
%         CLOSED(CLOSED_COUNT, 2) = yNode;
%         OPEN(index_min_node, 1) = 0;
% 
%     % no path found
%     else
%       NoPath = 0;
%     end
% end
% 
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Final path reconstruction
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% if (xNode == xTarget && yNode == yTarget)
% 
%     i = size(CLOSED,1);
%     Optimal_path = [];
%     xval = CLOSED(i,1);
%     yval = CLOSED(i,2);
%     i = 1;
%     Optimal_path(i,1) = xval;  
%     Optimal_path(i,2) = yval;  
%     i = i + 1;
% 
%     % Traverse OPEN and determine the parent nodes
%     parent_x = OPEN(node_index(OPEN,xval,yval),4); % row index
%     parent_y = OPEN(node_index(OPEN,xval,yval),5); % col index
% 
%     while (parent_x ~= xStart || parent_y ~= yStart)
%         Optimal_path(i,1) = parent_x;
%         Optimal_path(i,2) = parent_y;
% 
%         inode    = node_index(OPEN, parent_x, parent_y);
%         parent_x = OPEN(inode,4);
%         parent_y = OPEN(inode,5);
%         i = i + 1;
%     end
% 
%     % add Start point
%     Optimal_path(i,1) = xStart;
%     Optimal_path(i,2) = yStart;
% 
%     % now Optimal_path is [Goal ... Start]，reverse Start --> Goal
%     Optimal_path = flipud(Optimal_path);
% 
%     % check the path 
%     j = size(Optimal_path,1);
%     path_valid = true;
%     for idx = 1:j
%         if MAP(Optimal_path(idx,1), Optimal_path(idx,2)) == -1
%             fprintf('WARNING: Path goes through/over obstacle at (%d, %d)!\n', ...
%                 Optimal_path(idx,1), Optimal_path(idx,2));
%             path_valid = false;
%         end
%     end
% 
%     if path_valid
%         fprintf('Path validation: PASS\n');
%     end
% 
%     % generating pure pursuit waypoints 
%     cellSize = 1.0;   
%     originX  = 0.0;    
%     originY  = 0.0;    
% 
%     % grid (row,col) = (xGrid,yGrid)
%     xWorld = originX + (Optimal_path(:,2) - 0.5) * cellSize; 
%     yWorld = originY + (Optimal_path(:,1) - 0.5) * cellSize;
% 
%     % pure pursuit 
%     waypoints = [xWorld, yWorld];
% 
%     % path drawing animation 
%     p = plot(Optimal_path(1,2)+.5, Optimal_path(1,1)+.5, 'bo', 'MarkerSize', 8, 'LineWidth', 2);
% 
%     for idx = 2:j
%         set(p, 'XData', Optimal_path(idx,2)+.5,'YData', Optimal_path(idx,1)+.5);
%         drawnow;
%     end
% 
%     plot(Optimal_path(:,2)+.5, Optimal_path(:,1)+.5, 'c-', 'LineWidth', 2.5);
%     title("Synthetic Obstacle Map - Path Found");
% else
%     pause(1);
%     h = msgbox('Sorry, No path exists to the Target!','warn');
%     uiwait(h,5);
% end
obstacle_height = 2; % obstacle height 2 m
% convert logical matrix BW into height matrix Z
% 1 (obstacle) -> 2m, 0 (flat) -> 0m
Z1 = double(BW) * obstacle_height;

extra_flat_width = 10;
Z2 = zeros(MAX_X, extra_flat_width);

extra_gravel_width = 50;
Z3 = 0.05 * randn(MAX_X, extra_gravel_width); % random noise

Z_final_mat = [Z1, Z2, Z3];

[rows, cols] = size(Z_final_mat);
x_total = (0 : cols-1) * cellSize;
y_total = (0 : rows-1) * cellSize;
Z_final = Z_final_mat;