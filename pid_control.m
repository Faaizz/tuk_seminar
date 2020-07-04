% THIS SCRIPT IMPLEMENTS A PID CONTROLLER FOR TRAJECTORY TRACKING OF ROBOT
% MOTION


%% CONFIGURATION SPACE
% X-AXIS: 0...50
% Y_AXIS: 0...50

%% INITIALIZE

% Scenario 1
%init_scen_1;

% Scenario 2
init_scen_2;

% Scenario 3
%init_scen_3;

%% OBTAIN MODEL

% Set sampling time
h= 0.01;

% Lateral model
robot_sys_ss= lateral_2_dof_model(robot_vel, mass, I_zz, C_f, C_r, l_f, l_r);

robot_sys_ss= c2d(robot_sys_ss, h);


%% OBTAIN TRAJECTORY

%trajectory= robot_motion(robot_vel, robot_init_pos_path, h);
trajectory= trajectory_generator_apf(robot_vel, robot_init_pos_path, h);

% Get lateral coordinates
x_traj= trajectory(3,:);

% Get time vector
t_traj= trajectory(1,:);

% Reference Trajectory
ref_traj= timeseries(trajectory(3,:), trajectory(1,:), 'Name', 'reference_input');


%% SIMULATE SYSTEM RESPONSE (STATE SPACE)

% Run simulation
sim_out= sim('dof_dyn_pid_control_sim', max(t_traj));



%% PLOT CONTROLLED MOTION
robot_motion_plot;




