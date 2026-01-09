% unit test code that calls on astar_pathfinding.m

function run_astar_tests(show_plots)
    % show_plots: true (default) = show visualization, false = no plots
    
    if nargin < 1
        show_plots = true;
    end
    
    fprintf('\n========================================\n');
    fprintf('     A* PATH PLANNING UNIT TESTS\n');
    fprintf('========================================\n');
    if show_plots
        fprintf('  Visualization: ENABLED\n');
    else
        fprintf('  Visualization: DISABLED\n');
    end
    fprintf('========================================\n\n');
    
    % Track test results
    test_results = struct('name', {}, 'passed', {}, 'message', {});
    
    % Run Test 1
    [passed1, msg1] = test_simple_path(show_plots);
    test_results(1).name = 'Simple Path (No Obstacles)';
    test_results(1).passed = passed1;
    test_results(1).message = msg1;
    
    % Run Test 2
    [passed2, msg2] = test_obstacle_avoidance(show_plots);
    test_results(2).name = 'Obstacle Avoidance (Fixed Map)';
    test_results(2).passed = passed2;
    test_results(2).message = msg2;
    
    % Run Test 3
    [passed3, msg3] = test_random_obstacles(show_plots);
    test_results(3).name = 'Random Obstacle Map';
    test_results(3).passed = passed3;
    test_results(3).message = msg3;
    
    % Print Summary Report
    print_test_summary(test_results);
end

function [overall_pass, summary_msg] = test_simple_path(show_plots)
    fprintf('TEST 1: Simple Path (No Obstacles)\n');
    fprintf('-----------------------------------\n');
    
    assertions_passed = 0;
    total_assertions = 0;
    test_failed = false;
    failure_reasons = {};
    
    % test parameters
    MAX_X = 20;
    MAX_Y = 20;
    xStart = 2; yStart = 2;
    xTarget = 18; yTarget = 18;
    
    % empty obstacle map
    [X, ~] = meshgrid(1:MAX_Y, 1:MAX_X);
    BW = false(size(X));
    
    % main astar_pathfinding function
    [MAP, ~, ~, xNode, yNode, iterationCount, NoPath, Optimal_path] = astar_pathfinding(MAX_X, MAX_Y, BW, xStart, yStart, xTarget, yTarget, 1, 1000);
    
    % ASSERTION 1: Path should be found
    total_assertions = total_assertions + 1;
    if (xNode == xTarget && yNode == yTarget && NoPath == 1)
        fprintf('ASSERT 1: Path found to goal\n');
        assertions_passed = assertions_passed + 1;
    else
        fprintf('ASSERT 1 FAILED: No path found\n');
        test_failed = true;
        failure_reasons{end+1} = 'Path not found';
    end
    
    % ASSERTION 2: Iteration count reasonable
    total_assertions = total_assertions + 1;
    fprintf('Iterations: %d\n', iterationCount);
    if iterationCount < 1000 && iterationCount > 0
        fprintf('ASSERT 2: Iteration count within bounds\n');
        assertions_passed = assertions_passed + 1;
    else
        fprintf('ASSERT 2 FAILED: Iterations = %d\n', iterationCount);
        test_failed = true;
        failure_reasons{end+1} = sprintf('Iterations: %d', iterationCount);
    end
    
    if ~isempty(Optimal_path)
        % ASSERTION 3: Path efficiency
        total_assertions = total_assertions + 1;
        
        % Debug: Show path structure
        fprintf('Path has %d waypoints\n', size(Optimal_path, 1));
        fprintf('First waypoint: (%d, %d)\n', Optimal_path(1,1), Optimal_path(1,2));
        fprintf('Last waypoint: (%d, %d)\n', Optimal_path(end,1), Optimal_path(end,2));
        
        path_length = calculate_path_length_from_array(Optimal_path);
        dx = abs(xTarget - xStart);
        dy = abs(yTarget - yStart);
        expected_min = min(dx, dy)*sqrt(2) + abs(dx - dy);
        
        fprintf('Path length: %.2f units\n', path_length);
        fprintf('Theoretical minimum: %.2f units\n', expected_min);
        
        % Check if path is valid
        if path_length < expected_min * 0.99
            fprintf('WARNING: Path shorter than theoretical minimum!\n');
            fprintf('This suggests a bug in path reconstruction or length calculation.\n');
        end
        
        % efficiency ratio: actual v optimal path
        efficiency_ratio = (path_length / expected_min);
        fprintf('Path/Optimal ratio: %.2fx (100%% = perfect)\n', efficiency_ratio);
        
        if path_length <= expected_min * 1.5 && path_length >= expected_min * 0.99
            fprintf('ASSERT 3: Path length acceptable (within 150%% of optimal)\n');
            assertions_passed = assertions_passed + 1;
        else
            fprintf('ASSERT 3 FAILED: Path length suspicious\n');
            test_failed = true;
            failure_reasons{end+1} = 'Path length invalid';
        end
        
        % Visualization
        if show_plots
            plot_test_result(MAP, Optimal_path, xStart, yStart, xTarget, yTarget, 'Test 1: Simple Path (No Obstacles)', 1);
        end
    else
        fprintf('ASSERT 3: Skipped (no path)\n');
        total_assertions = total_assertions + 1;
    end
    
    fprintf('\n  Result: %d/%d assertions passed\n', assertions_passed, total_assertions);
    
    if test_failed
        overall_pass = false;
        summary_msg = sprintf('FAILED (%d/%d). %s', assertions_passed, total_assertions, strjoin(failure_reasons, '; '));
        fprintf('TEST FAILED\n\n');
    else
        overall_pass = true;
        summary_msg = sprintf('PASSED (%d/%d)', assertions_passed, total_assertions);
        fprintf('TEST PASSED\n\n');
    end
end

function [overall_pass, summary_msg] = test_obstacle_avoidance(show_plots)
    fprintf('TEST 2: Obstacle Avoidance (Fixed Map)\n');
    fprintf('---------------------------------------\n');
    
    assertions_passed = 0;
    total_assertions = 0;
    test_failed = false;
    failure_reasons = {};
    
    % fixed obstacles setup
    MAX_X = 30;
    MAX_Y = 30;
    xStart = 5; yStart = 15;
    xTarget = 25; yTarget = 15;
    
    [X, Y] = meshgrid(1:MAX_Y, 1:MAX_X);
    BW = false(size(X));
    
    % fixed vertical wall
    BW(Y > 5 & Y < 25 & X > 14 & X < 16) = true;
    
    % calling the astar_pathfinding function
    [MAP, ~, ~, xNode, yNode, iterationCount, NoPath, Optimal_path] = astar_pathfinding(MAX_X, MAX_Y, BW, xStart, yStart, xTarget, yTarget, 1, 5000);
    
    % ASSERTION 1: Path found
    total_assertions = total_assertions + 1;
    if (xNode == xTarget && yNode == yTarget && NoPath == 1)
        fprintf('ASSERT 1: Path found around obstacle\n');
        assertions_passed = assertions_passed + 1;
    else
        fprintf('ASSERT 1 FAILED: No path found\n');
        test_failed = true;
        failure_reasons{end+1} = 'No path';
    end
    
    % ASSERTION 2: Iterations reasonable
    total_assertions = total_assertions + 1;
    fprintf('    Iterations: %d\n', iterationCount);
    if iterationCount < 5000 && iterationCount > 0
        fprintf('ASSERT 2: Iteration count OK\n');
        assertions_passed = assertions_passed + 1;
    else
        fprintf('ASSERT 2 FAILED: Too many iterations\n');
        test_failed = true;
        failure_reasons{end+1} = 'Iterations exceeded';
    end
    
    if ~isempty(Optimal_path)
        % ASSERTION 3: No obstacle collisions
        total_assertions = total_assertions + 1;
        path_valid = true;
        obstacle_hits = 0;
        
        for idx = 1:size(Optimal_path, 1)
            if MAP(Optimal_path(idx, 1), Optimal_path(idx, 2)) == -1
                path_valid = false;
                obstacle_hits = obstacle_hits + 1;
            end
        end
        
        if path_valid
            fprintf('ASSERT 3: Path avoids all obstacles\n');
            assertions_passed = assertions_passed + 1;
        else
            fprintf('ASSERT 3 FAILED: Path hits %d obstacles!\n', obstacle_hits);
            test_failed = true;
            failure_reasons{end+1} = sprintf('%d collisions', obstacle_hits);
        end
        
        % visualization
        if show_plots
            plot_test_result(MAP, Optimal_path, xStart, yStart, xTarget, yTarget, ...
                'Test 2: Obstacle Avoidance (Fixed Map)', 2);
        end
    else
        fprintf('ASSERT 3: Skipped\n');
        total_assertions = total_assertions + 1;
    end
    
    fprintf('\n  Result: %d/%d assertions passed\n', assertions_passed, total_assertions);
    
    if test_failed
        overall_pass = false;
        summary_msg = sprintf('FAILED (%d/%d). %s', ...
            assertions_passed, total_assertions, strjoin(failure_reasons, '; '));
        fprintf('TEST FAILED\n\n');
    else
        overall_pass = true;
        summary_msg = sprintf('PASSED (%d/%d)', assertions_passed, total_assertions);
        fprintf('TEST PASSED\n\n');
    end
end

function [overall_pass, summary_msg] = test_random_obstacles(show_plots)
    fprintf('TEST 3: Random Obstacle Map\n');
    fprintf('----------------------------\n');
    
    assertions_passed = 0;
    total_assertions = 0;
    test_failed = false;
    failure_reasons = {};
    
    % random obstacles
    MAX_X = 50;
    MAX_Y = 50;
    xStart = 5; yStart = 5;
    xTarget = 45; yTarget = 45;
    
    [X, Y] = meshgrid(1:MAX_Y, 1:MAX_X);
    BW = false(size(X));
    
    rng(42);  % fixed seed for reproducibility
    numRects = 4;
    numCircs = 4;
    
    for i = 1:numRects
        x1 = randi([5, MAX_X-25]);
        y1 = randi([5, MAX_Y-25]);
        w = randi([8, 15]);
        h = randi([8, 15]);
        BW(Y > y1 & Y < y1+h & X > x1 & X < x1+w) = true;
    end
    
    for i = 1:numCircs
        cx = randi([10, MAX_X-10]);
        cy = randi([10, MAX_Y-10]);
        r = randi([5, 10]);
        BW((X-cx).^2 + (Y-cy).^2 < r^2) = true;
    end
    
    fprintf('Generated %d rectangles, %d circles\n', numRects, numCircs);
    
    % main astar_pathfinding function
    [MAP, ~, ~, xNode, yNode, iterationCount, NoPath, Optimal_path] = ...
        astar_pathfinding(MAX_X, MAX_Y, BW, xStart, yStart, xTarget, yTarget, 1, 10000);
    
    % ASSERTION 1: Path found
    total_assertions = total_assertions + 1;
    if (xNode == xTarget && yNode == yTarget && NoPath == 1)
        fprintf('ASSERT 1: Path found in random map\n');
        assertions_passed = assertions_passed + 1;
    else
        fprintf('ASSERT 1 FAILED: No path found\n');
        test_failed = true;
        failure_reasons{end+1} = 'No path';
    end
    
    % ASSERTION 2: Iterations
    total_assertions = total_assertions + 1;
    fprintf('Iterations: %d\n', iterationCount);
    if iterationCount < 10000 && iterationCount > 0
        fprintf('ASSERT 2: Completed within limit\n');
        assertions_passed = assertions_passed + 1;
    else
        fprintf('ASSERT 2 FAILED: Iteration limit\n');
        test_failed = true;
        failure_reasons{end+1} = 'Too many iterations';
    end
    
    if ~isempty(Optimal_path)
        % ASSERTION 3: Path validity
        total_assertions = total_assertions + 1;
        path_valid = true;
        
        for idx = 1:size(Optimal_path, 1)
            if MAP(Optimal_path(idx, 1), Optimal_path(idx, 2)) == -1
                path_valid = false;
                break;
            end
        end
        
        if path_valid
            fprintf('ASSERT 3: Path valid (no collisions)\n');
            assertions_passed = assertions_passed + 1;
        else
            fprintf('ASSERT 3 FAILED: Obstacle collision\n');
            test_failed = true;
            failure_reasons{end+1} = 'Path invalid';
        end
        
        % Visualization
        if show_plots
            plot_test_result(MAP, Optimal_path, xStart, yStart, xTarget, yTarget, ...
                'Test 3: Random Obstacle Map', 3);
        end
    else
        fprintf('ASSERT 3: Skipped\n');
        total_assertions = total_assertions + 1;
    end
    
    fprintf('\nResult: %d/%d assertions passed\n', assertions_passed, total_assertions);
    
    if test_failed
        overall_pass = false;
        summary_msg = sprintf('FAILED (%d/%d). %s', assertions_passed, total_assertions, strjoin(failure_reasons, '; '));
        fprintf('TEST FAILED\n\n');
    else
        overall_pass = true;
        summary_msg = sprintf('PASSED (%d/%d)', assertions_passed, total_assertions);
        fprintf('TEST PASSED\n\n');
    end
end

function plot_test_result(MAP, Optimal_path, xStart, yStart, xTarget, yTarget, title_str, fig_num)
    figure(fig_num);
    clf;
    
    map_display = (MAP ~= -1);
    imshow(~map_display, 'InitialMagnification', 'fit');
    hold on;
    axis tight;
    axis on;
    grid on;
    
    % plotting start and goal
    h1 = plot(yStart, xStart, 'bo', 'MarkerSize', 10, 'LineWidth', 3, 'MarkerFaceColor', 'blue');
    text(yStart+2, xStart, 'START', 'Color', 'blue', 'FontWeight', 'bold');
    
    h2 = plot(yTarget, xTarget, 'gd', 'MarkerSize', 10, 'LineWidth', 3, 'MarkerFaceColor', 'green');
    text(yTarget+2, xTarget, 'GOAL', 'Color', 'green', 'FontWeight', 'bold');
    
    % plotting the optimal path 
    if ~isempty(Optimal_path)
        Optimal_path = flipud(Optimal_path);
        h3 = plot(Optimal_path(:,2), Optimal_path(:,1), 'c-', 'LineWidth', 2.5);
        h4 = plot(Optimal_path(:,2), Optimal_path(:,1), 'm.', 'MarkerSize', 8);
    end
    
    set(gca, 'YDir', 'normal');
    title(title_str, 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Y coordinate');
    ylabel('X coordinate');
    
    % legend with the plotted elements
    if ~isempty(Optimal_path)
        legend([h1, h2, h3, h4], {'Start', 'Goal', 'Path', 'Waypoints'}, 'Location', 'best');
    else
        legend([h1, h2], {'Start', 'Goal'}, 'Location', 'best');
    end
    
    drawnow;
end

function print_test_summary(test_results)
    fprintf('========================================\n');
    fprintf('               TEST SUMMARY\n');
    fprintf('========================================\n\n');
    
    total_tests = length(test_results);
    passed_tests = sum([test_results.passed]);
    
    for i = 1:total_tests
        if test_results(i).passed
            fprintf('PASS: %s\n', test_results(i).name);
        else
            fprintf('FAIL: %s\n', test_results(i).name);
        end
        fprintf('     %s\n\n', test_results(i).message);
    end
    
    fprintf('========================================\n');
    fprintf('  Overall: %d/%d tests passed (%.1f%%)  \n', passed_tests, total_tests, (passed_tests/total_tests)*100);
    fprintf('========================================\n');
    
    if passed_tests == total_tests
        fprintf('\nALL TESTS PASSED! \n\n');
    else
        fprintf('\nSOME TESTS FAILED\n\n');
    end
end

function length = calculate_path_length_from_array(path)

    % path stored in reverse order (goal to start); flipping it here
    if path(1,1) ~= path(end,1)
        path = flipud(path);
    end
    
    length = 0;
    for i = 1:size(path,1)-1
        length = length + sqrt((path(i+1,1)-path(i,1))^2 + (path(i+1,2)-path(i,2))^2);
    end
end

% running the tests
run_astar_tests(true);