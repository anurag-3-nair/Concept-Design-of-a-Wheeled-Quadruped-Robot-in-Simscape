%% Step 2: Intelligent Path Planning with PRM

% Define start and goal positions
start = [1, 1];    % Start position
goal = [9, 9];     % Goal position

% Verify start and goal are in free space
if checkOccupancy(map, start) == 1
    fprintf('Warning: Start position is occupied! Searching for nearby free space...\n');
    % Find nearest free space
    start = findFreeSpaceNear(map, start);
end

if checkOccupancy(map, goal) == 1
    fprintf('Warning: Goal position is occupied! Searching for nearby free space...\n');
    goal = findFreeSpaceNear(map, goal);
end

fprintf('Start: [%.1f, %.1f]\n', start(1), start(2));
fprintf('Goal: [%.1f, %.1f]\n', goal(1), goal(2));

% Create PRM with optimized parameters
prm = mobileRobotPRM;
prm.Map = map;
prm.NumNodes = 200;          % More nodes for complex environments
prm.ConnectionDistance = 5;  % Connection distance

% Multiple attempts to find path
maxAttempts = 20;
pathFound = false;
bestPath = [];

fprintf('\n=== PATH PLANNING IN PROGRESS ===\n');

for attempt = 1:maxAttempts
    fprintf('Attempt %d/%d: ', attempt, maxAttempts);
    
    % Update PRM with new random nodes
    update(prm);
    
    path = findpath(prm, start, goal);
    
    if ~isempty(path)
        pathLength = sum(sqrt(sum(diff(path).^2,2)));
        fprintf('Path found! Length: %.2f meters\n', pathLength);
        
        % Keep the shortest path
        if isempty(bestPath) || pathLength < sum(sqrt(sum(diff(bestPath).^2,2)))
            bestPath = path;
        end
        pathFound = true;
        
        % If we found a reasonably short path, we can stop
        if pathLength < 15 % Reasonable threshold
            break;
        end
    else
        fprintf('No path found. Increasing nodes...\n');
        prm.NumNodes = prm.NumNodes + 50; % Increase nodes for next attempt
    end
end

%% Step 3: Analyze and Visualize the Path
if pathFound
    fprintf('\n=== PATH FOUND SUCCESSFULLY! ===\n');
    fprintf('Path length: %.2f meters\n', sum(sqrt(sum(diff(bestPath).^2,2))));
    fprintf('Number of waypoints: %d\n', size(bestPath,1));
    fprintf('The robot will navigate around all obstacles!\n');
    
    % Visualize the final result
    figure(1)
    show(prm)
    hold on;
    plot(bestPath(:,1), bestPath(:,2), 'b-', 'LineWidth', 3);
    plot(start(1), start(2), 'gs', 'MarkerSize', 15, 'LineWidth', 3, 'MarkerFaceColor', 'g');
    plot(goal(1), goal(2), 'rs', 'MarkerSize', 15, 'LineWidth', 3, 'MarkerFaceColor', 'r');
    
    % Add arrows to show direction
    for i = 1:3:length(bestPath)-1
        quiver(bestPath(i,1), bestPath(i,2), ...
               bestPath(i+1,1)-bestPath(i,1), bestPath(i+1,2)-bestPath(i,2), ...
               0, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
    end
    
    legend('Obstacles', 'Obstacle Centers', 'PRM Graph', 'Planned Path', 'Start', 'Goal');
    title('Intelligent Path Planning - Robot Navigation Around Obstacles');
    
else
    fprintf('\n=== UNABLE TO FIND PATH ===\n');
    fprintf('The obstacles are too dense. Trying advanced planning...\n');
    
    % Try with much larger PRM
    prm.NumNodes = 500;
    prm.ConnectionDistance = 3;
    update(prm);
    bestPath = findpath(prm, start, goal);
    
    if ~isempty(bestPath)
        fprintf('Path found with advanced parameters!\n');
        pathFound = true;
        % Visualize as above
    else
        error(['No path found with current obstacle configuration. ' ...
               'Consider: 1) Reducing obstacle sizes, 2) Adding gaps between obstacles']);
    end
end

%% Step 4: Save the Planned Path
if pathFound
    % Save path data for navigation
    path_data.path = bestPath;
    path_data.start = start;
    path_data.goal = goal;
    path_data.map = map;
    path_data.obstacles = obstacles;
    
    save('navigation_path.mat', 'path_data');
    fprintf('\nNavigation path saved to navigation_path.mat\n');
end

%% Helper function to find free space near occupied area
function freePoint = findFreeSpaceNear(map, point)
    % Search in increasing radius for free space
    for radius = 0.1:0.1:1.0
        for angle = 0:0.5:2*pi
            testPoint = point + radius * [cos(angle), sin(angle)];
            if testPoint(1) >= 0 && testPoint(1) <= 10 && ...
               testPoint(2) >= 0 && testPoint(2) <= 10 && ...
               checkOccupancy(map, testPoint) == 0
                freePoint = testPoint;
                return;
            end
        end
    end
    freePoint = [0.5, 0.5]; % Fallback
end