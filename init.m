% THIS SCRIPT INITILIZES THE REQUIRED PARAMETERS



%% Clear workspace
clear;
clc;


%% SAMPLING TIME
h= 0.05;


%% SIMULATION PARAMETERS

% Robot Longitudinal Velocity (m.s^-1)
robot_vel= 1;

% Robot Initial Position
robot_init_pos= [0 50]';

% Goal point
X_goal= [40 5]';

% Initial Yaw Angle
yaw_ang_init= -0.0588+(pi/2);

%% ROBOT MODEL PARAMETERS

% Mass(kg)
mass= 505;

% Yaw mass moment of Inertia(kg.m^2)
I_zz= 808.5;

% Front tyre cornering stiffness(N/deg)
%C_f= 1420;
C_f= 40000;

% Rear tyre cornering stiffness(N/deg)
%C_r= 1420;
C_r= 40000;

% Longitudinal distance of front wheel from center of mass(m)
l_f= 0.35;

% Longitudinal distance of back wheels from center of mass(m)
l_r= 0.4125;



%% TRAJECTORY PARAMETERS

% Attractive Potential Constant
k_att= 0.1;

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
obstacles= [ [14.87 33.28]' [32 30]' [19 19]' [34.49 17.25]' [27 18]' ];

% Obstacles (mild corners)
%obstacles= [ [14.87 33.28]' [32 30]' [19 19]' [34.49 17.25]'];







