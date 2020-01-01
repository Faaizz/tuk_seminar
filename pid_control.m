% THIS SCRIPT IMPLEMENTS A PID CONTROLLER FOR TRAJECTORY TRACKING OF ROBOT
% MOTION

%% Clear workspace
clear;
clc;


%% CONFIGURATION SPACE
% X-AXIS: 0...50
% Y_AXIS: 0...50

% Sampling period
h= 0.2;


%% OBTAIN MODEL

% PID Controller Parameters (Obtained using pidTuner)
%Kp= 73.41;
%Ki= 72.16;
%Kd= 18.67;

%Kp= 2.2222;
%Ki= 0.0754;
%Kd= 6.741;

Kp= 23.086;
Ki= 1.9187;
Kd= 5.6913;

% PID Controller
D= pid(Kp, Ki, Kd);

% Lateral model
[robot_sys_tf, robot_sys_ss]= lateral_2_dof_model(h, D);


%% OBTAIN TRAJECTORY

% Robot Longitudinal Velocity (m.s^-1)
robot_vel= 5;

% Robot Initial Position
robot_init_pos= [0 50]';

% subplot(2, 2, 1);

% Appropraite Subplot to plot for potential field and planned robot path
% specified in robot_motion.m

trajectory= robot_motion(robot_vel, robot_init_pos, h);

% Get lateral coordinates
x_traj= trajectory(2,:);

% Get time vector
t_traj= trajectory(1,:);

% Reference Trajectory
ref_traj= timeseries(trajectory(2,:), trajectory(1,:), 'Name', 'reference_input');


%% SIMULATE SYSTEM RESPONSE (TRANSFER FUNCTION MODEL)

x_robot_pos= lsim(robot_sys_tf, x_traj, t_traj);

% Plot reference trajectory and system output against time
subplot(2, 2, [3,4]);

% Reference Trajectory
plot( t_traj, x_traj, '--b');
title("Lateral Position of Robot");
xlabel("time");
ylabel("x-axis (fixed earth)");
hold on
% Robot Actual Position
plot( t_traj, x_robot_pos, 'k'); 

legend("Reference Position", "Actual Position");
legend("Location", "northwest");
hold off


%% SIMULATE SYSTEM RESPONSE (STATE SPACE)

% Set simulation time
set_param('pid_control_sim', 'StopTime', string(max(t_traj)));





