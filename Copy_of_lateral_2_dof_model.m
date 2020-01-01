% This script presents a 2 DOF Bicycle Model representation of a 3 wheel,
% mobile robot with small-angles assumptions

%% CLEAR
clear;
clc;

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
v= 5;
% Longitudinal distance of front wheel from center of mass(cm)
l_f= 50;
% Longitudinal distance of back wheels from center of mass(cm)
l_r= 100;

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


%% TRANSFER FUNCTION MODEL
% System transfer function model with lateral displacement as output

b_1= abs(A_c(2,2));
b_2= abs(A_c(2,4));
b_3= abs(B_c(2));

c_1= abs(A_c(4,2));
c_2= abs(A_c(4,4));
c_3= abs(B_c(4));

tf_one= tf([v*(b_3+c_3)  v*(b_3*c_2 + b_2*c_3) ], [1 (b_1 + c_2) (b_1*c_2 - b_2*c_1) 0]);
tf_two= tf([(v*c_3) v*(b_1*c_3 + b_3*c_1)], [ 1 (c_2 + b_1) (c_2*b_1 - c_1*b_2) 0 0 ]);

sys_c_tf= tf_one + tf_two;

