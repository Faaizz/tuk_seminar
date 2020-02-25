% THIS SCRIPT IMPLEMENTS A MPC CONTROLLER FOR TRAJECTORY TRACKING OF ROBOT
% MOTION


%% CONFIGURATION SPACE
% X-AXIS: 0...50
% Y_AXIS: 0...50

%% INITIALIZE

% Scenario 1
%init_scen_1;

% Scenario 2
%init_scen_2;

% Scenario 3
init_scen_3;


%% OBTAIN MODEL

% Lateral model
robot_sys_ss= yaw_lateral_2_dof_model(robot_vel, mass, I_zz, C_f, C_r, l_f, l_r);

robot_sys_ss= c2d(robot_sys_ss, h);


%% OBTAIN TRAJECTORY

% Appropraite Subplot to plot for potential field and planned robot path
% specified in robot_motion.m

%trajectory= robot_motion(robot_vel, robot_init_pos_path, h);
trajectory= trajectory_generator_apf(robot_vel, robot_init_pos_path, h);

% Get lateral coordinates
x_traj= trajectory(3,:);

% Get time vector
t_traj= trajectory(1,:);

% Reference Trajectory
ref_traj= timeseries(trajectory(3,:), trajectory(1,:), 'Name', 'reference_input');

yaw_traj= timeseries(yaw_ang, trajectory(1,:), 'Name', 'reference_yaw');


%% MPC

% Prediction horizon

% Crashes into obstacle
%prediction_horizon= 15;
%control_horizon= 4;

prediction_horizon= 25;
control_horizon= 4;

mpcobj = mpc(robot_sys_ss, h);
mpcobj.PredictionHorizon= prediction_horizon;
mpcobj.ControlHorizon= control_horizon;

% INPUT CONSTRAINTS
% Maxmimum rate of change of wheel angle
max_delta_u= 30*(pi/180);    %(rad/sec)
mpcobj.ManipulatedVariables.RateMin= -max_delta_u*h;
mpcobj.ManipulatedVariables.RateMax= max_delta_u*h;

% Maximum wheel angle
max_u= 40*(pi/180);  %(rad)
mpcobj.ManipulatedVariables.Min= -max_u;
mpcobj.ManipulatedVariables.Max= max_u;

% Output weight
mpcobj.Weights.OutputVariables= [1, 1];




%% SIMULATE SYSTEM RESPONSE (STATE SPACE)

% % Open simulink model
% open_system('mpc_control_sim.slx');
% % Set simulation time
% set_param('mpc_control_sim', 'StopTime', string(max(t_traj)));
% % Run simulation


% Open simulink model
%open_system('dyn_mpc_control_sim.slx');
% Set simulation time
%set_param('dyn_mpc_control_sim', 'StopTime', string(max(t_traj)));
% Run simulation
sim_out= sim('yaw_dof_dyn_mpc_control_sim', max(t_traj));




%% PLOT CONTROLLED MOTION
robot_controlled_motion;

