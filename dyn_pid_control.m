% THIS SCRIPT IMPLEMENTS A PID CONTROLLER FOR TRAJECTORY TRACKING OF ROBOT
% MOTION

%% Clear workspace
clear;
clc;


%% CONFIGURATION SPACE
% X-AXIS: 0...50
% Y_AXIS: 0...50

% Sampling period
h= 0.01;


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


% Robot Longitudinal Velocity (m.s^-1)
robot_vel= 1;

% Lateral model
[robot_sys_tf, robot_sys_ss]= lateral_2_dof_model(h, robot_vel, D);


%% OBTAIN TRAJECTORY


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


%% SIMULATE SYSTEM RESPONSE (STATE SPACE)

% Open simulink model
open_system('dyn_pid_control_sim.slx');

% Run simulation
sim_out= sim('dyn_pid_control_sim', max(t_traj));

%% PLOT CONTROLLED MOTION
robot_controlled_motion;



