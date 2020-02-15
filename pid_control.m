% THIS SCRIPT IMPLEMENTS A PID CONTROLLER FOR TRAJECTORY TRACKING OF ROBOT
% MOTION


%% CONFIGURATION SPACE
% X-AXIS: 0...50
% Y_AXIS: 0...50


%% OBTAIN MODEL

% PID Controller Parameters (Obtained using pidTuner)
Kp= 34.370;
Kd= 41.429;
Ki= 0.315;
Tf= 0.053;


% Lateral model
robot_sys_ss= lateral_2_dof_model(robot_vel, mass, I_zz, C_f, C_r, l_f, l_r);

robot_sys_ss= c2d(robot_sys_ss, h);


%% OBTAIN TRAJECTORY

% Appropraite Subplot to plot for potential field and planned robot path
% specified in robot_motion.m

%trajectory= robot_motion(robot_vel, robot_init_pos, h);
trajectory= trajectory_generator_apf(robot_vel, robot_init_pos, h);

% Get lateral coordinates
x_traj= trajectory(2,:);

% Get time vector
t_traj= trajectory(1,:);

% Reference Trajectory
ref_traj= timeseries(trajectory(2,:), trajectory(1,:), 'Name', 'reference_input');


%% SIMULATE SYSTEM RESPONSE (STATE SPACE)

% % Open simulink model
% open_system('pid_control_sim.slx');
% % Set simulation time
% set_param('pid_control_sim', 'StopTime', string(max(t_traj)));
% % Run simulation
% sim_out_lin= sim('pid_control_sim', max(t_traj));


% Open simulink model
open_system('dyn_pid_control_sim.slx');
% Set simulation time
set_param('dyn_pid_control_sim', 'StopTime', string(max(t_traj)));
% Run simulation
sim_out= sim('dyn_pid_control_sim', max(t_traj));



%% PLOT CONTROLLED MOTION
robot_controlled_motion;



