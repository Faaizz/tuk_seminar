% THIS SCRIPT IMPLEMENTS A PID CONTROLLER FOR TRAJECTORY TRACKING OF ROBOT
% MOTION


%% CONFIGURATION SPACE
% X-AXIS: 0...50
% Y_AXIS: 0...50


%% OBTAIN MODEL

% PID Controller Parameters (Obtained using pidTuner)
Kp= 34.562;
Kd= 40.883;
Ki= 1.644;
N= 13.595;


% Lateral model
robot_sys_ss= lateral_2_dof_model(robot_vel, mass, I_zz, C_f, C_r, l_f, l_r);

robot_sys_ss= c2d(robot_sys_ss, h);


%% OBTAIN TRAJECTORY

% Appropraite Subplot to plot for potential field and planned robot path
% specified in robot_motion.m

%trajectory= robot_motion(robot_vel, robot_init_pos, h);
trajectory= trajectory_generator_apf(robot_vel, robot_init_pos_path, h);

% Get lateral coordinates
x_traj= trajectory(3,:);

% Get time vector
t_traj= trajectory(1,:);

% Reference Trajectory
ref_traj= timeseries(trajectory(3,:), trajectory(1,:), 'Name', 'reference_input');


%% SIMULATE SYSTEM RESPONSE (STATE SPACE)

% % Open simulink model
% open_system('pid_control_sim.slx');
% % Set simulation time
% set_param('pid_control_sim', 'StopTime', string(max(t_traj)));
% % Run simulation
% sim_out_lin= sim('pid_control_sim', max(t_traj));


% Open simulink model
%open_system('pid_control_sim.slx');
%open_system('dyn_pid_control_sim.slx');
% Set simulation time
%set_param('pid_control_sim', 'StopTime', string(max(t_traj)));
%set_param('dyn_pid_control_sim', 'StopTime', string(max(t_traj)));
% Run simulation
%sim_out= sim('pid_control_sim', max(t_traj));
sim_out= sim('dyn_pid_control_sim', max(t_traj));



%% PLOT CONTROLLED MOTION
lat_displacement_tracking;

% Control signal
% VehicleModel assigned from Nonlinear_model.m
VehicleModel.deltaf= sim_out.control_signal{1}.Values.Data;

simulator = VehicleDynamicsLateral.Simulator(VehicleModel, trajectory(1,:));

% Setup initial conditions
simulator.ALPHAT0 = 0;           
simulator.dPSI0 = 0;             
simulator.V0= robot_vel; 
simulator.PSI0= yaw_ang_init;

% Retrieving states from Simulink model
simulator.XT = sim_out.states{1}.Values.Data;
simulator.YT = sim_out.states{2}.Values.Data;
simulator.PSI = sim_out.states{3}.Values.Data;
simulator.VEL = sim_out.states{4}.Values.Data;
simulator.ALPHAT = sim_out.states{5}.Values.Data;
simulator.dPSI = sim_out.states{6}.Values.Data;


g = VehicleDynamicsLateral.Graphics(simulator);
g.TractorColor = 'r';

g.Frame();
%g.Animation();




