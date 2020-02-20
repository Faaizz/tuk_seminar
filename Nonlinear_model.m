function [sys,x0,str,ts] = Nonlinear_model(t,x,u,flag)
% This file is a s-function template for simulating the simple vehicle model in Simulink.

% Choosing tire model
% TireModel = VehicleDynamicsLateral.TireLinear();
% TireModel.k= 40000;
TireModel = VehicleDynamicsLateral.TirePacejka();
TireModel.a0 = 1;
TireModel.a1 = 0;
TireModel.a2 = 800;
TireModel.a3 = 3000;
TireModel.a4 = 50;
TireModel.a5 = 0;
TireModel.a6 = 0;
TireModel.a7 = -1;
TireModel.a8 = 0;
TireModel.a9 = 0;
TireModel.a10 = 0;
TireModel.a11 = 0;
TireModel.a12 = 0;
TireModel.a13 = 0;

% Choosing vehicle model
VehicleModel = VehicleDynamicsLateral.VehicleSimpleNonlinear();

% Defining vehicle parameters
% Mass(kg)
mass= evalin('base','mass');
% Yaw mass moment of Inertia(kg.m^2)
IT= evalin('base','I_zz');
% Longitudinal distance of front wheel from center of mass(m)
a= evalin('base','l_f');
% Longitudinal distance of back wheels from center of mass(m)
b= evalin('base','l_r');
% Wheelbase
lT= a+b;
% Mass over rear axle
mR0 = (a/lT) * mass;
% Mass over front axle
mF0 = mass-mR0;

VehicleModel.lT= lT;
VehicleModel.mR0 = mR0;
VehicleModel.mF0 = mF0;
VehicleModel.IT = IT;
% Number of ront wheels
VehicleModel.nF = 1;
% Number of rear wheels
VehicleModel.nR = 2;
% Track of the car
VehicleModel.wT = 2;
VehicleModel.muy = .8;
VehicleModel.tire = TireModel;

% Save to base workspace
assignin('base', 'VehicleModel', VehicleModel);

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes();

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,VehicleModel);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,VehicleModel);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
% end csfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes()

% Definitions
sizes = simsizes;
sizes.NumContStates  = 6;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6;
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);

% Setting initial conditions
% Velocity
robot_vel= evalin('base','robot_vel');
% Position
robot_init_pos_sim= evalin('base','robot_init_pos_sim');
% Yaw angle
yaw_ang_init= evalin('base','yaw_ang_init');

x0  = [robot_init_pos_sim(1) robot_init_pos_sim(2) yaw_ang_init robot_vel 0 0];
str = [];
ts  = [0 0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%


function sys = mdlDerivatives(t,x,u,vehicle)

% Defining input
vehicle.deltaf = u(1);
vehicle.Fxf = u(2);
vehicle.Fxr = u(3);

% Getting the vehicle model function (state equations)
ModelFunction = @vehicle.Model;

sys = ModelFunction(t,x,0);

% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(~,x,~,~)

% Output are all state variables
sys = x;

% end mdlOutputs

%% See Also
%
% <../index.html Home> | <TemplateSimpleSimulink.html Template Simple Simulink>
%
