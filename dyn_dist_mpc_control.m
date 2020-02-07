% THIS SCRIPT IMPLEMENTS A MPC CONTROLLER FOR TRAJECTORY TRACKING OF ROBOT
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


% Robot Longitudinal Velocity (m.s^-1)
robot_vel= 1;

% Lateral model
robot_sys_ss= lateral_2_dof_model_continuous_time(h, robot_vel, 0);

robot_sys_ss= c2d(robot_sys_ss, h);


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


%% MPC

% Prediction horizon
prediction_horizon= 50;
control_horizon= 4;

mpcobj = mpc(robot_sys_ss, h);
mpcobj.PredictionHorizon= prediction_horizon;
mpcobj.ControlHorizon= control_horizon;

% INPUT CONSTRAINTS
% Maxmimum rate of change of wheel angle
max_delta_u= 30;    %(deg/sec)
mpcobj.ManipulatedVariables.RateMin= -max_delta_u;
mpcobj.ManipulatedVariables.RateMax= max_delta_u;

% Maximum wheel angle
max_u= 40;  %(deg)
mpcobj.ManipulatedVariables.Min= -max_u;
mpcobj.ManipulatedVariables.Max= max_u;

% Output weight
mpcobj.Weights.OutputVariables= 5;
%mpcobj.Weights.ManipulatedVariables= 0;


%% DISTURBANCES

% Wind disturbance (m/s)
wind_dist= [ 0 5 4];





%% SIMULATE SYSTEM RESPONSE (STATE SPACE)

% Open simulink model
open_system('dyn_dist_mpc_control_sim.slx');

% Run simulation
sim_out= sim('dyn_dist_mpc_control_sim', max(t_traj));



%% PLOT CONTROLLED MOTION
robot_controlled_motion;





