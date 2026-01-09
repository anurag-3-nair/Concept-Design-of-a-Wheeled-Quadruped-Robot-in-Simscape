% wrapper function containing the main A* logic

function [MAP, OPEN, CLOSED, xNode, yNode, iterationCount, NoPath, Optimal_path] = ...
    astar_pathfinding(MAX_X, MAX_Y, BW, xStart, yStart, xTarget, yTarget, robot_radius, maxIterations)

    % Initialize MAP
    MAP = 2 * ones(MAX_X, MAX_Y);
    
    % Obstacle inflation
    se = strel('disk', robot_radius);
    map_inflated = imdilate(BW, se);
    
    % Error checking - ensure start/goal are free
    map_inflated(xStart, yStart) = 0;
    map_inflated(xTarget, yTarget) = 0;
    
    % Set map values
    MAP(map_inflated) = -1;
    MAP(xStart, yStart) = 1;
    MAP(xTarget, yTarget) = 0;
    
    % Initialize OPEN and CLOSED lists
    OPEN = [];
    CLOSED = [];
    
    % Add all obstacles to CLOSED list
    k = 1;
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
    
    % Initialize with start node
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
    
    % A* MAIN LOOP
    iterationCount = 0;
    
    while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
        exp_array = expand_array(xNode, yNode, path_cost, xTarget, yTarget, CLOSED, MAX_X, MAX_Y, MAP);
        exp_count = size(exp_array, 1);
        iterationCount = iterationCount + 1;
        
        if iterationCount > maxIterations
            warning('Max iterations reached, aborting search.');
            NoPath = 0;
            break;
        end
        
        % Update OPEN list with successor nodes
        for i = 1:exp_count
            flag = 0;
            for j = 1:OPEN_COUNT
                if (exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3))
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
        
        % Find node with minimum fn
        index_min_node = min_fn(OPEN, OPEN_COUNT, xTarget, yTarget);
        if (index_min_node ~= -1)
            xNode = OPEN(index_min_node, 2);
            yNode = OPEN(index_min_node, 3);
            path_cost = OPEN(index_min_node, 6);
            
            CLOSED_COUNT = CLOSED_COUNT + 1;
            CLOSED(CLOSED_COUNT, 1) = xNode;
            CLOSED(CLOSED_COUNT, 2) = yNode;
            OPEN(index_min_node, 1) = 0;
        else
            NoPath = 0;
        end
    end
    
    % PATH RECONSTRUCTION
    Optimal_path = [];
    
    if (xNode == xTarget && yNode == yTarget)
        i = size(CLOSED, 1);
        xval = CLOSED(i, 1);
        yval = CLOSED(i, 2);
        i = 1;
        Optimal_path(i, 1) = xval;
        Optimal_path(i, 2) = yval;
        i = i + 1;
        
        parent_x = OPEN(node_index(OPEN, xval, yval), 4);
        parent_y = OPEN(node_index(OPEN, xval, yval), 5);
        
        while(parent_x ~= xStart || parent_y ~= yStart)
            Optimal_path(i, 1) = parent_x;
            Optimal_path(i, 2) = parent_y;
            
            inode = node_index(OPEN, parent_x, parent_y);
            parent_x = OPEN(inode, 4);
            parent_y = OPEN(inode, 5);
            i = i + 1;
        end
        
        % Add the start node (was missing!)
        Optimal_path(i, 1) = xStart;
        Optimal_path(i, 2) = yStart;
    end
end