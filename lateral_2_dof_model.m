function [sys_d_tf_cl, sys_d_ss]= lateral_2_dof_model(h, D)

% This function returns:
% 1- A Closed loop Discrete time Transfer Function model with a PID cntroller
% 2- Discrete time State space model (No PID incorporated)

% PARAMETERS
% h- sampling period
% D- PID Controller


%% CONSTANTS

% Mass(kg)
m= 1010;
% Yaw mass moment of Inertia(kg.m^2)
I_zz= 1617;
% Front tyre cornering stiffness(N/deg)
C_f= 710;
% Rear tyre cornering stiffness(N/deg)
C_r= 710;
% Forward velocity (longitudinal)(m.s^-1)
v= 2;
% Longitudinal distance of front wheel from center of mass(m)
l_f= 0.70;
% Longitudinal distance of back wheels from center of mass(m)
l_r= 0.85;

%% STATE SPACE REPRESENTATION

% If we select the state variables as:
% -Lateral displacement of center of mass
% -Side slip angle
% -Yaw angle
% -Yaw velocity
% And select our input variable as the steering angle of the front wheel.
% And select our output variables as:
% -Lateral Displacement of center of mass
% -Yaw velocity
% We have the following continuous time state matrix

% State Matrix
A_1= [ 0 v  v  0];
A_2= [ 0 -((C_f + 2*C_r) / (m*v)) 0 (((2*l_r*C_r - l_f*C_f)/(m*(v^2))) -1 )];
A_3= [ 0 0 0 1];
A_4= [ 0 ((2*l_r*C_r - l_f*C_f)/I_zz) 0 -((2*(l_r^2)*C_r + (l_f^2)*C_f)/(I_zz*v))];

% Continuous time state Matrix
A_c= [ A_1; A_2; A_3; A_4 ];

% Continuous time Input Matrix
B_c= [0; (C_f/(m*v)); 0; (l_f*C_f)/(I_zz)];

% Continuous time Output Matrix
C_c= [ 1 0 0 0; 0 0 0 1 ];

% Continuous time Feedthrough Matrix
D_c= [0 0]';

% Continuous Time System
sys_c= ss(A_c, B_c, C_c, D_c);

sys_c.InputName= 'Steering Angle';
sys_c.StateName= { 'Lateral Displacement of CoM', 'Side-slip Angle', 'Yaw Angle', 'Yaw Velocity' };
sys_c.OutputName= {'Lateral Displacement of CoM', 'Yaw Velocity'};



%% DISCRETIZATION (STATE SPACE MODEL)

sys_d_ss= c2d(sys_c, h);


%% TRANSFER FUNCTION MODEL
% System transfer function model with lateral displacement as output
[num, den]= ss2tf(A_c, B_c, [1 0 0 0], 0);

sys_c_tf= tf(num, den);


%% PID CONTROLLER

% Controller Parameters(Obtained using pidTuner), defined in pid_control.m
% Kp= 2.2356;
% Ki= 0.4610;
% Kd= 2.7103;

% Controller Transfer Function
% D= pid(Kp, Ki, Kd);


%% CLOSED LOOP SYSTEM

% Loop Transfer Function
sys_c_tf_loop= sys_c_tf * D;

% Closed loop Transfer Function
sys_c_tf_cl= feedback(sys_c_tf_loop, 1);


%% DISCRETE TIME STSTEM (TRANSFER FUNCTION MODEL)

% Sampling period
%h= 1/10;    % 10 samples per second

% Discretization
sys_d_tf_cl= c2d(sys_c_tf_cl, h);


end



















