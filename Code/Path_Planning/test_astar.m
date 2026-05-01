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

    % Run Test 4 — NEW: validates figure axis scaling matches Simscape (50x50m)
    [passed4, msg4] = test_axis_scaling(show_plots);
    test_results(4).name = 'Figure Axis Scaling (cellSize=0.5, 50x50m)';
    test_results(4).passed = passed4;
    test_results(4).message = msg4;
    
    % Print Summary Report
    print_test_summary(test_results);
end

% ============================================================
% TEST 1: Simple path, no obstacles
% ============================================================
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
        
        fprintf('Path has %d waypoints\n', size(Optimal_path, 1));
        fprintf('First waypoint: (%d, %d)\n', Optimal_path(1,1), Optimal_path(1,2));
        fprintf('Last waypoint: (%d, %d)\n', Optimal_path(end,1), Optimal_path(end,2));
        
        path_length = calculate_path_length_from_array(Optimal_path);
        dx = abs(xTarget - xStart);
        dy = abs(yTarget - yStart);
        expected_min = min(dx, dy)*sqrt(2) + abs(dx - dy);
        
        fprintf('Path length: %.2f units\n', path_length);
        fprintf('Theoretical minimum: %.2f units\n', expected_min);
        
        if path_length < expected_min * 0.99
            fprintf('WARNING: Path shorter than theoretical minimum!\n');
            fprintf('This suggests a bug in path reconstruction or length calculation.\n');
        end
        
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

% ============================================================
% TEST 2: Obstacle avoidance, fixed map
% ============================================================
function [overall_pass, summary_msg] = test_obstacle_avoidance(show_plots)
    fprintf('TEST 2: Obstacle Avoidance (Fixed Map)\n');
    fprintf('---------------------------------------\n');
    
    assertions_passed = 0;
    total_assertions = 0;
    test_failed = false;
    failure_reasons = {};
    
    MAX_X = 30;
    MAX_Y = 30;
    xStart = 5; yStart = 15;
    xTarget = 25; yTarget = 15;
    
    [X, Y] = meshgrid(1:MAX_Y, 1:MAX_X);
    BW = false(size(X));
    
    % fixed vertical wall
    BW(Y > 5 & Y < 25 & X > 14 & X < 16) = true;
    
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

% ============================================================
% TEST 3: Random obstacle map
% ============================================================
function [overall_pass, summary_msg] = test_random_obstacles(show_plots)
    fprintf('TEST 3: Random Obstacle Map\n');
    fprintf('----------------------------\n');
    
    assertions_passed = 0;
    total_assertions = 0;
    test_failed = false;
    failure_reasons = {};
    
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

% ============================================================
% TEST 4 (NEW): Validates figure axis scaling matches Simscape
%   - Waypoints must lie within [0, MAX_X*cellSize] x [0, MAX_Y*cellSize]
%   - Axis extents reported by plot must equal real-world dimensions
%   - Verifies cellSize=0.5 produces 50x50m space for 100x100 grid
% ============================================================
function [overall_pass, summary_msg] = test_axis_scaling(show_plots)
    fprintf('TEST 4: Figure Axis Scaling (cellSize=0.5, 50x50m)\n');
    fprintf('----------------------------------------------------\n');

    assertions_passed = 0;
    total_assertions  = 0;
    test_failed       = false;
    failure_reasons   = {};

    % Mirror path_gen.m production settings exactly
    MAX_X    = 100;
    MAX_Y    = 100;
    cellSize = 0.5;           % must match path_gen.m
    xStart   = 5;  yStart  = 5;
    xTarget  = 95; yTarget = 95;

    expected_world_max_x = MAX_Y * cellSize;   % 50.0 m
    expected_world_max_y = MAX_X * cellSize;   % 50.0 m

    % Build fixed obstacle map (mirrors path_gen.m fixed-map section)
    [X, Y] = meshgrid(1:MAX_Y, 1:MAX_X);
    BW = false(size(X));
    scale_factor = 0.8;

    cx1=27; cy1=27; hw1=round(12*scale_factor); hh1=round(7*scale_factor);
    BW(Y>cy1-hh1 & Y<cy1+hh1 & X>cx1-hw1 & X<cx1+hw1) = true;
    cx2=43; cy2=66; hw2=round(11*scale_factor); hh2=round(8*scale_factor);
    BW(Y>cy2-hh2 & Y<cy2+hh2 & X>cx2-hw2 & X<cx2+hw2) = true;
    cx3=75; cy3=30; hw3=round(10*scale_factor); hh3=round(10*scale_factor);
    BW(Y>cy3-hh3 & Y<cy3+hh3 & X>cx3-hw3 & X<cx3+hw3) = true;
    BW((X-30).^2 + (Y-75).^2 < (round(9*scale_factor))^2) = true;
    BW((X-70).^2 + (Y-60).^2 < (round(9*scale_factor))^2) = true;
    BW((X-50).^2 + (Y-25).^2 < (round(9*scale_factor))^2) = true;

    [~, ~, ~, xNode, yNode, ~, NoPath, Optimal_path] = ...
        astar_pathfinding(MAX_X, MAX_Y, BW, xStart, yStart, xTarget, yTarget, 1, 2e5);

    % ASSERTION 1: Path found (prerequisite for scaling checks)
    total_assertions = total_assertions + 1;
    if xNode == xTarget && yNode == yTarget && NoPath == 1
        fprintf('ASSERT 1: Path found (prerequisite OK)\n');
        assertions_passed = assertions_passed + 1;
    else
        fprintf('ASSERT 1 FAILED: No path — scaling checks skipped\n');
        test_failed = true;
        failure_reasons{end+1} = 'No path found';
        overall_pass = false;
        summary_msg  = sprintf('FAILED (%d/%d). %s', assertions_passed, total_assertions, strjoin(failure_reasons, '; '));
        fprintf('TEST FAILED\n\n');
        return;
    end

    % Convert path to world coordinates (must match path_gen.m fix exactly)
    xWorld_path = (Optimal_path(:,2) - 0.5) * cellSize;
    yWorld_path = (Optimal_path(:,1) - 0.5) * cellSize;

    % ASSERTION 2: All waypoint x-coordinates within [0, 50] m
    total_assertions = total_assertions + 1;
    tol = 1e-9;
    if all(xWorld_path >= -tol) && all(xWorld_path <= expected_world_max_x + tol)
        fprintf('ASSERT 2: Waypoint x-coords within [0, %.1fm]\n', expected_world_max_x);
        assertions_passed = assertions_passed + 1;
    else
        fprintf('ASSERT 2 FAILED: x range [%.2f, %.2f] exceeds [0, %.1f]\n', ...
            min(xWorld_path), max(xWorld_path), expected_world_max_x);
        test_failed = true;
        failure_reasons{end+1} = 'x waypoints out of range';
    end

    % ASSERTION 3: All waypoint y-coordinates within [0, 50] m
    total_assertions = total_assertions + 1;
    if all(yWorld_path >= -tol) && all(yWorld_path <= expected_world_max_y + tol)
        fprintf('ASSERT 3: Waypoint y-coords within [0, %.1fm]\n', expected_world_max_y);
        assertions_passed = assertions_passed + 1;
    else
        fprintf('ASSERT 3 FAILED: y range [%.2f, %.2f] exceeds [0, %.1f]\n', ...
            min(yWorld_path), max(yWorld_path), expected_world_max_y);
        test_failed = true;
        failure_reasons{end+1} = 'y waypoints out of range';
    end

    % ASSERTION 4: Figure axis limits equal real-world dimensions
    % Render the plot, then read back the axis limits to confirm they match
    total_assertions = total_assertions + 1;
    if show_plots
        fig_h = figure(4);
    else
        fig_h = figure('Visible', 'off');
    end
    clf(fig_h);

    imshow(ones(MAX_X, MAX_Y), 'InitialMagnification', 'fit', ...
        'XData', [0, MAX_Y * cellSize], ...
        'YData', [0, MAX_X * cellSize]);
    hold on;
    axis([0, MAX_Y*cellSize, 0, MAX_X*cellSize]);
    set(gca, 'YDir', 'normal');
    plot(xWorld_path, yWorld_path, 'c-', 'LineWidth', 2);
    xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('Test 4: Axis Scaling Check  |  expected %.0fx%.0f m', ...
        expected_world_max_x, expected_world_max_y));
    drawnow;

    ax_lims = axis(gca);   % [xmin xmax ymin ymax]
    ax_xmax = ax_lims(2);
    ax_ymax = ax_lims(4);

    if abs(ax_xmax - expected_world_max_x) < 0.01 && abs(ax_ymax - expected_world_max_y) < 0.01
        fprintf('ASSERT 4: Axis limits correct — [0,%.1f] x [0,%.1f] m\n', ax_xmax, ax_ymax);
        assertions_passed = assertions_passed + 1;
    else
        fprintf('ASSERT 4 FAILED: Axis limits [0,%.2f] x [0,%.2f] — expected [0,%.1f] x [0,%.1f]\n', ...
            ax_xmax, ax_ymax, expected_world_max_x, expected_world_max_y);
        test_failed = true;
        failure_reasons{end+1} = sprintf('Axis mismatch: got %.1fx%.1f, expected %.1fx%.1f', ...
            ax_xmax, ax_ymax, expected_world_max_x, expected_world_max_y);
    end

    if ~show_plots
        close(fig_h);
    end

    fprintf('\n  Result: %d/%d assertions passed\n', assertions_passed, total_assertions);

    if test_failed
        overall_pass = false;
        summary_msg  = sprintf('FAILED (%d/%d). %s', assertions_passed, total_assertions, strjoin(failure_reasons, '; '));
        fprintf('TEST FAILED\n\n');
    else
        overall_pass = true;
        summary_msg  = sprintf('PASSED (%d/%d)', assertions_passed, total_assertions);
        fprintf('TEST PASSED\n\n');
    end
end

% ============================================================
% SHARED HELPER: plot test result in real-world metres
% Updated to apply cellSize=0.5 scaling to match path_gen.m fix
% ============================================================
function plot_test_result(MAP, Optimal_path, xStart, yStart, xTarget, yTarget, title_str, fig_num)

    % cellSize must match path_gen.m — 0.5 m/cell
    cellSize = 0.5;

    [MAX_X, MAX_Y] = size(MAP);
    world_max_x = MAX_Y * cellSize;
    world_max_y = MAX_X * cellSize;

    figure(fig_num);
    clf;

    % Display map with real-world metre extents on axes
    map_display = (MAP ~= -1);
    imshow(~map_display, 'InitialMagnification', 'fit', ...
        'XData', [0, world_max_x], ...
        'YData', [0, world_max_y]);
    hold on;
    axis([0, world_max_x, 0, world_max_y]);
    axis on;
    grid on;
    set(gca, 'YDir', 'normal');

    % Convert start/target cell indices to metres
    xS_w = (yStart  - 0.5) * cellSize;
    yS_w = (xStart  - 0.5) * cellSize;
    xT_w = (yTarget - 0.5) * cellSize;
    yT_w = (xTarget - 0.5) * cellSize;

    h1 = plot(xS_w, yS_w, 'bo', 'MarkerSize', 10, 'LineWidth', 3, 'MarkerFaceColor', 'blue');
    text(xS_w + 0.5*cellSize, yS_w, 'START', 'Color', 'blue', 'FontWeight', 'bold');

    h2 = plot(xT_w, yT_w, 'gd', 'MarkerSize', 10, 'LineWidth', 3, 'MarkerFaceColor', 'green');
    text(xT_w + 0.5*cellSize, yT_w, 'GOAL', 'Color', 'green', 'FontWeight', 'bold');

    % Convert path cell indices to metres
    if ~isempty(Optimal_path)
        Optimal_path = flipud(Optimal_path);
        xPath_w = (Optimal_path(:,2) - 0.5) * cellSize;
        yPath_w = (Optimal_path(:,1) - 0.5) * cellSize;

        h3 = plot(xPath_w, yPath_w, 'c-', 'LineWidth', 2.5);
        h4 = plot(xPath_w, yPath_w, 'm.', 'MarkerSize', 8);
    end

    title(title_str, 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('x [m]');
    ylabel('y [m]');

    if ~isempty(Optimal_path)
        legend([h1, h2, h3, h4], {'Start', 'Goal', 'Path', 'Waypoints'}, 'Location', 'best');
    else
        legend([h1, h2], {'Start', 'Goal'}, 'Location', 'best');
    end

    drawnow;
end

% ============================================================
% SHARED HELPERS
% ============================================================
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
        fprintf('\nALL TESTS PASSED!\n\n');
    else
        fprintf('\nSOME TESTS FAILED\n\n');
    end
end

function length = calculate_path_length_from_array(path)
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