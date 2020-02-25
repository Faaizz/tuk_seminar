% THIS SCRIPT INITILIZES THE REQUIRED PARAMETERS



%% Clear workspace
clear;
clc;


%% SAMPLING TIME

%h= 0.1;    Crashes into obstacle
h= 0.05;


%% SIMULATION PARAMETERS

% Robot Longitudinal Velocity (m.s^-1)
robot_vel= 1;

% Robot Initial Position
robot_init_pos_path= [0 0]';
%robot_init_pos_sim= robot_init_pos_path;
robot_init_pos_sim= [0 0]';

% Goal point
X_goal= [50 31]';
%X_goal= 50.*[rand() rand()]';

% Initial Yaw Angle
yaw_ang_init= 0;



%% ROBOT MODEL PARAMETERS

% Mass(kg)
mass= 505;

% Yaw mass moment of Inertia(kg.m^2)
I_zz= 808.5;

% Front tyre cornering stiffness(N/rad)
%C_f= 1420;
C_f= 40e3;

% Rear tyre cornering stiffness(N/rad)
%C_r= 1420;
C_r= 40e3;

% Longitudinal distance of front wheel from center of mass(m)
l_f= 0.35;

% Longitudinal distance of back wheels from center of mass(m)
l_r= 0.4125;



%% TRAJECTORY PARAMETERS

% Attractive Potential Constant
k_att= 0.01;

% Repulsive Potential Constant
k_rep= 10;

% Obstacle Radius (Assuming circular obstacle of constant radius) (m)
obs_radius= 1;

% Maximum Longitudinal Velocity
V_MAX= robot_vel;  % m/sec

% Maximum Longitudinal Deceleration
A_MAX= 0.2;   % m/sec^2

% Obstacle Influence Range
rol_not= V_MAX/(2*A_MAX);

% Stopping Criteria
stopping_criteria= 0.001;   % Stop when Attractive Potential is at/lower than this value

% Robot Size Alllowance
robot_size_allowance= 1.5; % 150 cm

% Obstacles (sharp corners)
obstacles= [ [14.87 33.28]' [10 8]' [26 12]' [19 19]' [34 23]' ];

% Obstacles (mild corners)
%obstacles= [ [14.87 33.28]' [32 30]' [19 19]' [34.49 17.25]' [27 18]'];

% Maximum number of iterations
max_iterations= 150000;





