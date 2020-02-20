% THIS SCRIPT PLOTS A STEP RESPONSE FOR THE PREDICTION AND THE SIMULATION 
% MODELS IN  ORDER TO SEE THE DIFFERENCE IN THEIR DYNAMICS


%% SETUP

% SIMULATION

% End time (seconds)
t_end= 50;
time= 0:0.1:t_end;

% Step input
input= zeros(1, size(time, 2))+.9;

%% PREDICTION MODEL

% Choosing tire
TireModel = VehicleDynamicsLateral.TirePacejka();
% Choosing vehicle
PredModel = VehicleDynamicsLateral.VehicleSimpleLinear();

% Complete system definition
PredModel.tire = TireModel;

% Set input (wheel angle)
PredModel.deltaf= input;

% Define simulation object
simulator= VehicleDynamicsLateral.Simulator(PredModel, time);

% Run Simulation
simulator.Simulate();

% Results
XT = simulator.XT;
YT = simulator.YT;
PSI = simulator.PSI;
VEL = simulator.VEL;
ALPHAT = simulator.ALPHAT;
dPSI = simulator.dPSI;

% Display
g = VehicleDynamicsLateral.Graphics(simulator);
g.TractorColor = 'r';

g.Frame();
%g.Animation();




%% SIMULATION MODEL

% Choosing tire
TireModel = VehicleDynamicsLateral.TirePacejka();

% Choosing vehicle
SimModel = VehicleDynamicsLateral.VehicleSimpleNonlinear();

% Complete system definition
SimModel.tire = TireModel;

% Set input (wheel angle)
SimModel.deltaf= input;

% Define simulation object
simulator_2= VehicleDynamicsLateral.Simulator(SimModel, time);

% Run Simulation
simulator_2.Simulate();

% Results
XT = simulator_2.XT;
YT = simulator_2.YT;
PSI = simulator_2.PSI;
VEL = simulator_2.VEL;
ALPHAT = simulator_2.ALPHAT;
dPSI = simulator_2.dPSI;

% Display
g_2 = VehicleDynamicsLateral.Graphics(simulator_2);
g_2.TractorColor = 'r';

g_2.Frame();
%g.Animation();





