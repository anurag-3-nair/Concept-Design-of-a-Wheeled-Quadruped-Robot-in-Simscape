% ================= 1. 生成 80mm 宽的致密点云轮子 =================
theta = linspace(0, 2*pi, 36)'; % 增加到 36 个点，接触更平滑防穿模
X_circle = 0.127 * cos(theta);
Y_circle = 0.127 * sin(theta);

% 轮宽 80mm = 0.08m (Z方向从 -0.04m 到 +0.04m)
% 做三圈点云：左轮沿、右轮沿、以及中间加一圈，彻底防止漏穿网格
wheel_pts = [X_circle, Y_circle, repmat(0.04, 36, 1);   % 上边缘
             X_circle, Y_circle, zeros(36, 1);          % 中间圈
             X_circle, Y_circle, repmat(-0.04, 36, 1)]; % 下边缘

% ================= 2. 生成更大的碎石地面 =================
% 1. 生成大尺寸网格 (长15米，宽10米)
grid_step = 0.02;
x_vec = -5 : grid_step : 10;  
y_vec = -5 : grid_step : 5;   
[X, Y] = meshgrid(x_vec, y_vec);

% 2. 生成碎石高度
Z = 0.015 * randn(size(X));
Z = imgaussfilt(Z, 1);

% 3. 划分平地与碎石边界 (极其重要)
% 假设机器人默认在坐标原点 (0,0) 附近出生。
% 我们设定 X < 1.5 米的区域全都是平地，1.5米之后是碎石。
Z(X < 1.5) = 0; 

% 运行后，记得在 Simulink 的 Grid Surface 里依然用 Z'