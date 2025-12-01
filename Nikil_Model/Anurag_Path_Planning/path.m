clear; clc; close all;
scale = 0.5;

% parameters
MAX_X = round(100*scale);
MAX_Y = round(100*scale);
MAP = 2*(ones(MAX_X, MAX_Y));

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
        x1 = round(randi([5, 100-25])*scale);
        y1 = round(randi([5, 100-25])*scale);
        w = round(randi([8, 20])*scale);
        h = round(randi([8, 20])*scale);
        BW(Y > y1 & Y < y1+h & X > x1 & X < x1+w) = true;
    end

    % circles
    for i = 1:numCircs
        cx = round(randi([10, 100-10])*scale);
        cy = round(randi([10, 100-10])*scale);
        r = round(randi([5, 12])*scale);
        BW((X-cx).^2 + (Y-cy).^2 < r^2) = true;
    end

% fixed map
else
    % rectangles
    BW(Y > round(20*scale) & Y < round(35*scale) & X > round(15*scale) & X < round(45*scale)) = true;
    BW(Y > round(55*scale) & Y < round(75*scale) & X > round(30*scale) & X < round(55*scale)) = true;
    BW(Y > round(20*scale) & Y < round(40*scale) & X > round(65*scale) & X < round(85*scale)) = true;

    % circles
    BW((X-round(30*scale)).^2 + (Y-round(75*scale)).^2 < (round(12*scale))^2) = true;
    BW((X-round(70*scale)).^2 + (Y-round(60*scale)).^2 < (round(10*scale))^2) = true;
    BW((X-round(50*scale)).^2 + (Y-round(25*scale)).^2 < (round(10*scale))^2) = true; 

    % for the STL file
    obstacles = {};

    % independent from BW map for clear definition of obstacles
    % rectangles first
    rect1 = struct("type","rect", "x", 15*scale, "y", 20*scale, "w", (45-15)*scale, "h", (35-20)*scale );
    obstacles{end+1} = rect1;
    
    rect2 = struct("type","rect", "x", 30*scale, "y", 55*scale, "w", (55-30)*scale, "h", (75-55)*scale );
    obstacles{end+1} = rect2;
    
    rect3 = struct("type","rect", "x", 65*scale, "y", 20*scale, "w", (85-65)*scale, "h", (40-20)*scale );
    obstacles{end+1} = rect3;
    
    % circles (using original definitions) 
    circle1 = struct("type","circle", "cx", 30*scale, "cy", 75*scale, "r", 12*scale );
    obstacles{end+1} = circle1;
    
    circle2 = struct("type","circle", "cx", 70*scale, "cy", 60*scale, "r", 10*scale );
    obstacles{end+1} = circle2;
    
    circle3 = struct("type","circle", "cx", 50*scale, "cy", 25*scale, "r", 10*scale );
    obstacles{end+1} = circle3;
end

% defining a start and a goal
xStart = round(5  * scale);
yStart = round(5  * scale);
xTarget = round(95 * scale);
yTarget = round(95 * scale);

% robot footprint
robot_radius = 1;

% obstacles
se = strel('disk', robot_radius);
map_inflated = imdilate(BW, se);

% error checking
map_inflated(xStart, yStart) = 0;
map_inflated(xTarget, yTarget) = 0;

% map values
MAP(map_inflated) = -1;
MAP(xStart, yStart) = 1;
MAP(xTarget, yTarget) = 0;

% map plot
figure('Position', [100 100 900 800]);
imshow(~map_inflated, 'InitialMagnification','fit');
hold on;
axis([1 MAX_X+1 1 MAX_Y+1]);
axis on; grid on; 

plot(yStart, xStart, 'bo', 'MarkerSize', 8, 'LineWidth', 2);
text(yStart+2, xStart, 'Start', 'Color', 'blue');
plot(yTarget, xTarget, 'gd', 'MarkerSize', 8, 'LineWidth', 2);
text(yTarget+2, xTarget, 'Target', 'Color', 'green');

set(gca, 'YDir', 'normal');
title("Synthetic Obstacle Map - Computing the path ...");
drawnow;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialising the open lists for A*
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

OPEN=[];
CLOSED=[];
maxIterations = 2e5;
iterationCount = 0;

% adding obstacles in the closed list
k=1; % dummy counter
for i = 1:MAX_X
    for j = 1:MAX_Y
        if(MAP(i,j) == -1)
            CLOSED(k, 1) = i; 
            CLOSED(k, 2) = j; 
            k = k+1;
        end
    end
end
CLOSED_COUNT = size(CLOSED, 1);

% setting the starting node as the first node
xNode = xStart;
yNode = yStart;
OPEN_COUNT = 1;
path_cost = 0;
goal_distance = distance(xNode, yNode, xTarget, yTarget);

OPEN(OPEN_COUNT, :) = insert_open(xNode, yNode, xNode, yNode, path_cost, goal_distance, goal_distance);
OPEN(OPEN_COUNT, 1) = 0;

CLOSED_COUNT = CLOSED_COUNT + 1;
CLOSED(CLOSED_COUNT, 1) = xNode;
CLOSED(CLOSED_COUNT, 2) = yNode;
NoPath = 1;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Start Algorithm
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
    exp_array = expand_array(xNode, yNode, path_cost, xTarget, yTarget, CLOSED, MAX_X, MAX_Y, MAP);
    exp_count = size(exp_array, 1);
    iterationCount = iterationCount + 1;

    if iterationCount > maxIterations
        warning('Max iterations reached, aborting search.');
        break;
    end
    
    % successor node update in the open list
    for i = 1:exp_count
        flag = 0;
        for j = 1:OPEN_COUNT
            if (exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3))
                OPEN(j,8) = min(OPEN(j,8), exp_array(i,5));

                % updating the parent nodes if better path found
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
        
        % new elements into the open list
        if flag == 0
            OPEN_COUNT = OPEN_COUNT + 1;
            OPEN(OPEN_COUNT,:) = insert_open(exp_array(i,1), exp_array(i,2), xNode, yNode, exp_array(i,3), exp_array(i,4), exp_array(i,5));
        end
    end
    
    % finding out the node with the smallest fn 
    index_min_node = min_fn(OPEN, OPEN_COUNT, xTarget, yTarget);
    if (index_min_node ~= -1)
    
        % setting xNode and yNode to the node with minimum fn
        xNode = OPEN(index_min_node, 2);
        yNode = OPEN(index_min_node, 3);
        path_cost = OPEN(index_min_node, 6); 
        
        % moving the nodes to CLOSED list
        CLOSED_COUNT = CLOSED_COUNT + 1;
        CLOSED(CLOSED_COUNT, 1) = xNode;
        CLOSED(CLOSED_COUNT, 2) = yNode;
        OPEN(index_min_node, 1) = 0;
        
    % no path found
    else
      NoPath = 0;
    end
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Final path reconstruction
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (xNode == xTarget && yNode == yTarget)

    i = size(CLOSED,1);
    Optimal_path = [];
    xval = CLOSED(i,1);
    yval = CLOSED(i,2);
    
    i = 1;
    Optimal_path(i,1) = xval;  
    Optimal_path(i,2) = yval;  
    i = i + 1;
    
    % determining the parent nodes
    parent_x = OPEN(node_index(OPEN,xval,yval),4); % row index
    parent_y = OPEN(node_index(OPEN,xval,yval),5); % col index
   
    while (parent_x ~= xStart || parent_y ~= yStart)
        Optimal_path(i,1) = parent_x;
        Optimal_path(i,2) = parent_y;

        inode    = node_index(OPEN, parent_x, parent_y);
        parent_x = OPEN(inode,4);
        parent_y = OPEN(inode,5);
        i = i + 1;
    end
    
    % add Start point
    Optimal_path(i,1) = xStart;
    Optimal_path(i,2) = yStart;

    % now Optimal_path is [Goal ... Start]，reverse Start --> Goal
    Optimal_path = flipud(Optimal_path);

    % check the path 
    j = size(Optimal_path,1);
    path_valid = true;
    for idx = 1:j
        if MAP(Optimal_path(idx,1), Optimal_path(idx,2)) == -1
            fprintf('WARNING: Path goes through/over obstacle at (%d, %d)!\n', ...
                Optimal_path(idx,1), Optimal_path(idx,2));
            path_valid = false;
        end
    end
    
    if path_valid
        fprintf('Path validation: PASS\n');
    end

    % generating pure pursuit waypoints 
    cellSize = 1.0;   
    originX  = 0.0;    
    originY  = 0.0;    

    % grid (row,col) = (xGrid,yGrid)
    xWorld = originX + (Optimal_path(:,2) - 0.5) * cellSize; 
    yWorld = originY + (Optimal_path(:,1) - 0.5) * cellSize;
    
    % pure pursuit 
    waypoints = [xWorld, yWorld];

    % path drawing animation 
    p = plot(Optimal_path(1,2)+.5, Optimal_path(1,1)+.5, 'bo', 'MarkerSize', 8, 'LineWidth', 2);
    
    for idx = 2:j
        set(p, 'XData', Optimal_path(idx,2)+.5,'YData', Optimal_path(idx,1)+.5);
        drawnow;
    end

    plot(Optimal_path(:,2)+.5, Optimal_path(:,1)+.5, 'c-', 'LineWidth', 2.5);
    title("Synthetic Obstacle Map - Path Found");
else
    pause(1);
    h = msgbox('Sorry, No path exists to the Target!','warn');
    uiwait(h,5);
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% STL GENERATION 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% parameters (in meters)
floor_thickness = 0.1;   
cell_size = 1.0;    
obstacle_heights = [2.0, 2.0];

% STL file name
stl_file = 'environment.stl';

% writing a triangular facet to STL file
function write_facet(fid, v1, v2, v3)
    n = cross(v2 - v1, v3 - v1);
    n = n ./ max(norm(n), eps);
    fprintf(fid,' facet normal %f %f %f\n', n(1), n(2), n(3));
    fprintf(fid,'  outer loop\n');
    fprintf(fid,'   vertex %f %f %f\n', v1);
    fprintf(fid,'   vertex %f %f %f\n', v2);
    fprintf(fid,'   vertex %f %f %f\n', v3);
    fprintf(fid,'  endloop\n endfacet\n');
end

% extruding rectangles (as prism) 
function write_rect_prism(fid, x, y, w, h, height)
    xb = x;      
    xt = x + w;
    yb = y;      
    yt = y + h;
    z0 = 0;         % bottom 
    z1 = height;    % top

    % Define 8 vertices
    V = [xb yb z0; xt yb z0; xt yt z0; xb yt z0;
        xb yb z1; xt yb z1; xt yt z1; xb yt z1];
    
    % triangle faces
    F = [                 
        1 2 3; 1 3 4;     % bottom
        5 6 7; 5 7 8;     % top
        1 2 6; 1 6 5;     % sides
        2 3 7; 2 7 6;
        3 4 8; 3 8 7;
        4 1 5; 4 5 8
    ];

    for i = 1:size(F,1)
        write_facet(fid, V(F(i,1),:), V(F(i,2),:), V(F(i,3),:));
    end
end

% extruding a circular obstacle (as cylinder)
function write_cylinder(fid, cx, cy, r, height, N)
    theta = linspace(0, 2*pi, N+1);
    x = cx + r * cos(theta);
    y = cy + r * sin(theta);
    z0 = 0;
    z1 = height;

    % side walls
    for i = 1:N
        v1 = [x(i)   y(i)   z0];
        v2 = [x(i+1) y(i+1) z0];
        v3 = [x(i+1) y(i+1) z1];
        v4 = [x(i)   y(i)   z1];

        write_facet(fid, v1, v2, v3);
        write_facet(fid, v1, v3, v4);
    end

    % top cap
    for i = 1:N
        c = [cx cy z1];
        v1 = [x(i)   y(i)   z1];
        v2 = [x(i+1) y(i+1) z1];
        write_facet(fid, c, v1, v2);
    end
end

% writing the STL file
fid = fopen(stl_file, 'w');
fprintf(fid, 'solid environment\n');

% floor plate
map_width  = size(BW, 2) * cell_size;
map_height = size(BW, 1) * cell_size;

write_rect_prism(fid, 0, 0, map_width, map_height, floor_thickness);

% obstacles (rectangles & circles)
for k = 1:length(obstacles)
    obs = obstacles{k};

    if strcmp(obs.type, 'rect')
        height = obstacle_heights(randi(2));
        write_rect_prism(fid, obs.x, obs.y, obs.w, obs.h, height);

    elseif strcmp(obs.type,'circle')
        height = obstacle_heights(randi(2));
        write_cylinder(fid, obs.cx, obs.cy, obs.r, height, 40); 

    end
end

fprintf(fid, 'endsolid environment');
fclose(fid);

disp("STL saved as environment.stl");