%% Complete Navigation System
fprintf('=== COMPLETE OBSTACLE NAVIGATION SYSTEM ===\n\n');

% Step 1: Create environment
fprintf('Step 1: Creating obstacle environment...\n');
create_environment;

% Step 2: Plan path around obstacles
fprintf('Step 2: Planning path around obstacles...\n');
intellingent_path_planning;

% Step 3: Simulate navigation
fprintf('Step 3: Simulating robot navigation...\n');
simulate_navigation;

fprintf('\n=== SYSTEM READY ===\n');
fprintf('The robot can now navigate from start to goal while intelligently\n');
fprintf('finding paths around all obstacles in the environment.\n');