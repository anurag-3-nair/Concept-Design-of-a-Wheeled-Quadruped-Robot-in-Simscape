%% WALKING GAIT - TROT PATTERN
clear; clc; close all;

%% === SIMULATION PARAMETERS ===
T_total = 10;                 % total simulation time [s]
t = linspace(0, T_total, 1000)';   % time resolution

%% === WALKING PARAMETERS ===
walk_freq = 0.5;              % Step frequency [Hz] - faster for dynamic walking
stride_hip = 10 * pi/180;     % Hip swing amplitude [rad]
stride_knee = 30 * pi/180;    % Knee lift amplitude [rad]

% Standing offsets
hip_offset = 0 * pi/180;     % Hips slightly outward
knee_offset = 10 * pi/180;    % Knees moderately bent

% Trot gait phases (diagonal pairs)
phase_FL = 0;      % Was 0
phase_FR = pi;       % Was pi  
phase_RL = pi;       % Was pi
phase_RR = 0;      % Was 0
%% === Smooth startup ramp ===
ramp_duration = 1.5;          % Faster ramp up
ramp = 0.5 * (1 - cos(pi * min(t / ramp_duration, 1)));

%% === Hip trajectories - WALKING ===
hip_FL = hip_offset + ramp .* (stride_hip * sin(2*pi*walk_freq*t + phase_FL));
hip_FR = hip_offset + ramp .* (stride_hip * sin(2*pi*walk_freq*t + phase_FR));
hip_RL = hip_offset + ramp .* (stride_hip * sin(2*pi*walk_freq*t + phase_RL));
hip_RR = hip_offset + ramp .* (stride_hip * sin(2*pi*walk_freq*t + phase_RR));

%% === Knee trajectories - WALKING ===
% Knees lift during swing phase, lower during stance
knee_lift = stride_knee * (0.5 - 0.5 * cos(4*pi*walk_freq*t)); % Double frequency for lift

knee_FL = knee_offset + ramp .* knee_lift .* (0.5 + 0.5 * sin(2*pi*walk_freq*t + phase_FL + pi/2));
knee_FR = knee_offset + ramp .* knee_lift .* (0.5 + 0.5 * sin(2*pi*walk_freq*t + phase_FR + pi/2));
knee_RL = knee_offset + ramp .* knee_lift .* (0.5 + 0.5 * sin(2*pi*walk_freq*t + phase_RL + pi/2));
knee_RR = knee_offset + ramp .* knee_lift .* (0.5 + 0.5 * sin(2*pi*walk_freq*t + phase_RR + pi/2));

%% === WALKING PID GAINS ===
Kp = 8.0;     % Higher stiffness for dynamic motion
Ki = 1.5;     % Some integral for weight support
Kd = 15.0;    % Good damping

%% === TORQUE LIMITS ===
max_torque_hip = 55.0;   % [Nm] - Higher for dynamic motion
max_torque_knee = 60.0;  % [Nm]

%% === GRAVITY COMPENSATION ===
standing_torque_hip = -1.0;   % [Nm] - Moderate inward pull
standing_torque_knee = -14.0;  % [Nm] - Good support

%% === Convert to timeseries for Simulink ===
hip_FL_sig  = timeseries(hip_FL, t);
hip_FR_sig  = timeseries(hip_FR, t);
hip_RL_sig  = timeseries(hip_RL, t);
hip_RR_sig  = timeseries(hip_RR, t);

knee_FL_sig = timeseries(knee_FL, t);
knee_FR_sig = timeseries(knee_FR, t);
knee_RL_sig = timeseries(knee_RL, t);
knee_RR_sig = timeseries(knee_RR, t);

wheel_speed = 0 * ones(size(t));  % slow rolling [rad/s]
wheel_FL_sig = timeseries(wheel_speed, t);
wheel_FR_sig = timeseries(wheel_speed, t);
wheel_RL_sig = timeseries(wheel_speed, t);
wheel_RR_sig = timeseries(wheel_speed, t);

%% === Create parameter structures ===
motor_limits.max_torque_hip = max_torque_hip;
motor_limits.max_torque_knee = max_torque_knee;

gravity_compensation.hip = standing_torque_hip;
gravity_compensation.knee = standing_torque_knee;

PID_gains.Kp = Kp;
PID_gains.Ki = Ki;
PID_gains.Kd = Kd;

%% === Save to workspace ===
assignin('base', 'hip_FL_sig', hip_FL_sig);
assignin('base', 'hip_FR_sig', hip_FR_sig);
assignin('base', 'hip_RL_sig', hip_RL_sig);
assignin('base', 'hip_RR_sig', hip_RR_sig);

assignin('base', 'knee_FL_sig', knee_FL_sig);
assignin('base', 'knee_FR_sig', knee_FR_sig);
assignin('base', 'knee_RL_sig', knee_RL_sig);
assignin('base', 'knee_RR_sig', knee_RR_sig);

assignin('base', 'motor_limits', motor_limits);
assignin('base', 'gravity_compensation', gravity_compensation);
assignin('base', 'PID_gains', PID_gains);

%% === Plot results ===
figure('Position', [100, 100, 1200, 800]);

subplot(2,2,1);
plot(t, [hip_FL hip_FR hip_RL hip_RR]*180/pi, 'LineWidth', 1.5);
title('Hip Joint Angles - WALKING'); ylabel('Angle [°]');
legend('FL','FR','RL','RR'); grid on;

subplot(2,2,2);
plot(t, [knee_FL knee_FR knee_RL knee_RR]*180/pi, 'LineWidth', 1.5);
title('Knee Joint Angles - WALKING'); ylabel('Angle [°]');
legend('FL','FR','RL','RR'); grid on;

subplot(2,2,3);
plot(t, hip_FL*180/pi, 'r-', t, knee_FL*180/pi, 'b-', 'LineWidth', 2);
title('Front Left Leg - Hip vs Knee'); ylabel('Angle [°]'); xlabel('Time [s]');
legend('Hip', 'Knee'); grid on;

subplot(2,2,4);
plot(t, hip_FR*180/pi, 'r-', t, knee_FR*180/pi, 'b-', 'LineWidth', 2);
title('Front Right Leg - Hip vs Knee'); ylabel('Angle [°]'); xlabel('Time [s]');
legend('Hip', 'Knee'); grid on;

disp(' WALKING GAIT GENERATED - TROT PATTERN');
disp('   - Dynamic walking might be more stable than static standing');
disp('   - Diagonal leg pairs move together');
disp('   - Knees lift during swing phase');