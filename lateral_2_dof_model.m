function sys_c= lateral_2_dof_model(robot_vel, mass, I_zz, C_f, C_r, l_f, l_r)

% This function returns a Closed loop continuous time model of the 2 DOF lateral vehicle model

%% ARGUMENTS:
%- robot_vel: Robot longitudinal velocity(m/s^2)
%- mass: Mass(kg)
%- I_zz: Yaw mass moment of inertia(kg.m^2)
%- C_f: Front tyre cornering stiffness(N/deg)
%- C_r: Rear tyre cornering stiffness(N/deg)
%- l_f: Longitudinal distance of front wheel from center of mass(m)
%- l_r: Longitudinal distance of rear wheel from center of mass(m)

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
A_1= [ 0 robot_vel  robot_vel  0];
A_2= [ 0 -((C_f + 2*C_r) / (mass*robot_vel)) 0 (((2*l_r*C_r - l_f*C_f)/(mass*(robot_vel^2))) -1 )];
A_3= [ 0 0 0 1];
A_4= [ 0 ((2*l_r*C_r - l_f*C_f)/I_zz) 0 -((2*(l_r^2)*C_r + (l_f^2)*C_f)/(I_zz*robot_vel))];

% Continuous time state Matrix
A_c= [ A_1; A_2; A_3; A_4 ];

% Continuous time Input Matrix
B_c= [0; (C_f/(mass*robot_vel)); 0; (l_f*C_f)/(I_zz)];

% Continuous time Output Matrix
C_c= [ 1 0 0 0];

% Continuous time Feedthrough Matrix
D_c= 0;

% Continuous Time System
sys_c= ss(A_c, B_c, C_c, D_c);


sys_c.InputName= 'Front Wheel Angle';

sys_c.StateName= { 'Lateral Displacement of CoM', 'Side-slip Angle', 'Yaw Angle', 'Yaw Velocity' };

sys_c.OutputName= {'Lateral Displacement of CoM'};


end



















