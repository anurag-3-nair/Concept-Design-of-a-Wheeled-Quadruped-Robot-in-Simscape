% ================= 1. Generate an 80mm wide dense point cloud wheel =================
theta = linspace(0, 2*pi, 36)'; % Increase to 36 points for smoother contact and to prevent mesh penetration
X_circle = 0.127 * cos(theta);
Y_circle = 0.127 * sin(theta);

% Wheel width 80mm = 0.08m (Z direction from -0.04m to +0.04m)
% Create three rings of point clouds: left rim, right rim, and one in the middle to completely prevent falling through the mesh
wheel_pts = [X_circle, Y_circle, repmat(0.04, 36, 1);   % Upper edge
             X_circle, Y_circle, zeros(36, 1);          % Middle ring
             X_circle, Y_circle, repmat(-0.04, 36, 1)]; % Lower edge

% ================= 2. Generate a larger gravel terrain =================
% 1. Generate a large-sized mesh grid (Length 15m, Width 10m)
grid_step = 0.02;
x_vec = -5 : grid_step : 10;  
y_vec = -5 : grid_step : 5;   
[X, Y] = meshgrid(x_vec, y_vec);

% 2. Generate gravel height
Z = 0.015 * randn(size(X));
Z = imgaussfilt(Z, 1);

% 3. Divide the boundary between flat ground and gravel (Extremely important)
% Assume the robot spawns near the origin (0,0) by default.
% We set the area where X < 1.5 meters entirely as flat ground, and after 1.5 meters is gravel.
Z(X < 1.5) = 0; 

% After running, remember to still use Z' in Simulink's Grid Surface