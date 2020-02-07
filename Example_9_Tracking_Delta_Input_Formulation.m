
%% Example_9_Tracking_Delta_Input_Formulation.m
%
% Tracking of a Time-Vayring Output Reference using the Delta Input Formulation
%
% Related Slides:         9-21
% Related Simulink Model: none
%
% Experiments:            none
%
% Remarks:                Requires MPT (people.ee.ethz.ch/~mpt/3/)
%
% Model Predictive Control
%
% apl. Prof Dr.-Ing. Daniel G�rges
% Electromobility Research Group
% Department of Electrical and Computer Engineering
% University of Kaiserslautern
% www.eit.uni-kl.de/en/jem/teaching/lectures/
%
% Please report any error to goerges@eit.uni-kl.de

%% Clear
clear;
close all;

%% OBTAIN MODEL

% Sampling period
h= 0.1;

% Robot Longitudinal Velocity (m.s^-1)
robot_vel= 3;

% Lateral model
robot_sys_ss= lateral_2_dof_model_continuous_time(h, robot_vel, 0);

%% Modeling of the LTI System
%A = [1 1; 0 1];
A= robot_sys_ss.A;
%B = [1; 0.5];
B= robot_sys_ss.B;
%C = [1 0];
C= robot_sys_ss.C;
D = 0;
Ts = h;
sys = LTISystem('A',A,'B',B,'C',C,'D',D,'Ts',Ts)
%x_0 = [0; 0];
x_0= [0; 0; 0; 0];
sys.initialize(x_0)

%% OBTAIN TRAJECTORY

% Robot Initial Position
robot_init_pos= [0 50]';

% subplot(2, 2, 1);

% Appropraite Subplot to plot for potential field and planned robot path
% specified in robot_motion.m

trajectory= robot_motion(robot_vel, robot_init_pos, h*2);

% Get lateral coordinates
x_traj= trajectory(2,:);

% Get time vector
t_traj= trajectory(1,:);


%% Definition of the Cost Function
Q = eye(sys.ny);
sys.y.penalty = QuadFunction(Q);
sys.u.with('deltaPenalty');
R = eye(sys.nu);
sys.u.deltaPenalty = QuadFunction(R);

%% Definition of the Constraints
%sys.x.min = [-5; -5];
%sys.x.max = [5; 5];
%sys.u.min= -1;  
%sys.u.min = -40;
%sys.u.max = 1;
%sys.u.max = 40;
sys.u.with('deltaMin');
sys.u.with('deltaMax');
sys.u.deltaMin = -40*Ts;
sys.u.deltaMax = 40*Ts;

%% Definition of a Time-Varying Output Reference
sys.y.with('reference');
sys.y.reference = 'free';

%% Definition of the Model Predictive Controller
prediction_horizon= 10;
ctrl = MPCController(sys,prediction_horizon)

%% Definition of the Closed-Loop System
loop = ClosedLoop(ctrl,sys)

%% Computation of the Closed-Loop Input, State, and Output Sequence and Cost
%N_sim = 30;
N_sim= length(x_traj);
u_0 = 0;
%y_ref = [ones(1,10) 2*ones(1,10) 3*ones(1,10)];
y_ref= x_traj;
data = loop.simulate(x_0,N_sim,'u.previous',u_0,'y.reference',y_ref)

%% Visualization of the Closed-Loop Output Sequence and the Reference Sequence
figure;
hold on;
plot(0:N_sim-1,data.Y,'b');
stairs(0:N_sim-1,y_ref,'b:');
legend('y','y_{ref}');
xlabel('t');

%% Generation of an Explicit Model Predictive Controller
%expctrl = ctrl.toExplicit()

%% Visualization of the Regions
%figure;

%% Example_9_Tracking_Delta_Input_Formulation.m
%
% Tracking of a Time-Vayring Output Reference using the Delta Input Formulation
%
% Related Slides:         9-21
% Related Simulink Model: none
%
% Experiments:            none
%
% Remarks:                Requires MPT (people.ee.ethz.ch/~mpt/3/)
%
% Model Predictive Control
%
% apl. Prof Dr.-Ing. Daniel G�rges
% Electromobility Research Group
% Department of Electrical and Computer Engineering
% University of Kaiserslautern
% www.eit.uni-kl.de/en/jem/teaching/lectures/
%
% Please report any error to goerges@eit.uni-kl.de

%% Clear
clear;
close all;

%% OBTAIN MODEL

% Sampling period
h= 0.1;

% Robot Longitudinal Velocity (m.s^-1)
robot_vel= 3;

% Lateral model
robot_sys_ss= lateral_2_dof_model_continuous_time(h, robot_vel, 0);

%% Modeling of the LTI System
%A = [1 1; 0 1];
A= robot_sys_ss.A;
%B = [1; 0.5];
B= robot_sys_ss.B;
%C = [1 0];
C= robot_sys_ss.C;
D = 0;
Ts = h;
sys = LTISystem('A',A,'B',B,'C',C,'D',D,'Ts',Ts)
%x_0 = [0; 0];
x_0= [0; 0; 0; 0];
sys.initialize(x_0)

%% OBTAIN TRAJECTORY

% Robot Initial Position
robot_init_pos= [0 50]';

% subplot(2, 2, 1);

% Appropraite Subplot to plot for potential field and planned robot path
% specified in robot_motion.m

trajectory= robot_motion(robot_vel, robot_init_pos, h*2);

% Get lateral coordinates
x_traj= trajectory(2,:);

% Get time vector
t_traj= trajectory(1,:);


%% Definition of the Cost Function
Q = eye(sys.ny);
sys.y.penalty = QuadFunction(Q);
sys.u.with('deltaPenalty');
R = eye(sys.nu);
sys.u.deltaPenalty = QuadFunction(R);

%% Definition of the Constraints
%sys.x.min = [-5; -5];
%sys.x.max = [5; 5];
%sys.u.min= -1;  
%sys.u.min = -40;
%sys.u.max = 1;
%sys.u.max = 40;
sys.u.with('deltaMin');
sys.u.with('deltaMax');
sys.u.deltaMin = -40*Ts;
sys.u.deltaMax = 40*Ts;

%% Definition of a Time-Varying Output Reference
sys.y.with('reference');
sys.y.reference = 'free';

%% Definition of the Model Predictive Controller
prediction_horizon= 10;
ctrl = MPCController(sys,prediction_horizon)

%% Definition of the Closed-Loop System
loop = ClosedLoop(ctrl,sys)

%% Computation of the Closed-Loop Input, State, and Output Sequence and Cost
%N_sim = 30;
N_sim= length(x_traj);
u_0 = 0;
%y_ref = [ones(1,10) 2*ones(1,10) 3*ones(1,10)];
y_ref= x_traj;
data = loop.simulate(x_0,N_sim,'u.previous',u_0,'y.reference',y_ref)

%% Visualization of the Closed-Loop Output Sequence and the Reference Sequence
figure;
hold on;
plot(0:N_sim-1,data.Y,'b');
stairs(0:N_sim-1,y_ref,'b:');
legend('y','y_{ref}');
xlabel('t');

%% Generation of an Explicit Model Predictive Controller
%expctrl = ctrl.toExplicit()

%% Visualization of the Regions
%figure;

%expctrl.partition.plot();