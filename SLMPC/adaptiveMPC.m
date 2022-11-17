%% Control of an Inverted Pendulum on a Cart

%%
% This plant is modeled in Simulink with commonly used blocks.
mdlPlant = 'mpc_pendcartPlant';
load_system(mdlPlant)
open_system([mdlPlant '/Pendulum and Cart System'],'force')

%% Control Structure
mdlMPC = 'mpc_pendcartAdaptiveMPC';
open_system(mdlMPC)

%% Linear Plant Model
% Since the MPC controller requires a linear time-invariant (LTI) plant
% model for prediction, linearize the Simulink plant model at the initial
% operating point.
%
% Specify linearization input and output points.
io(1) = linio([mdlPlant '/dF'],1,'openinput');
io(2) = linio([mdlPlant '/F'],1,'openinput');
io(3) = linio([mdlPlant '/Pendulum and Cart System'],1,'openoutput');
io(4) = linio([mdlPlant '/Pendulum and Cart System'],3,'openoutput');

%%
% Create operating point specifications for the plant initial conditions.
opspec = operspec(mdlPlant);

%%
% The first state is cart position _x_, which has a known initial state of
% 0.
opspec.States(1).Known = true;
opspec.States(1).x = 0;

%%
% The third state is pendulum angle _theta_, which has a known initial
% state of 0.
opspec.States(3).Known = true;
opspec.States(3).x = 0;

%%
% Compute operating point using these specifications.
options = findopOptions('DisplayReport',false);
[op, op_report] = findop(mdlPlant,opspec,options);

x0 = [op_report.States(1).x; op_report.States(2).x; op_report.States(3).x; op_report.States(4).x];
y0 = [op_report.Outputs(1).y;op_report.Outputs(2).y];
u0 = [op_report.Inputs(1).u op_report.Inputs(2).u];

%%
% Obtain the linear plant model at the specified operating point.
plant = linearize(mdlPlant,op,io);
plant.InputName = {'dF';'F'};
plant.OutputName = {'x';'theta'};

bdclose(mdlPlant)

%% MPC Design
% The plant has two inputs, _dF_ and _F_, and two outputs, _x_ and _theta_.
% In this example, _dF_ is specified as an unmeasured disturbance used by
% the MPC controller for better disturbance rejection. Set the plant
% signal types.
plant = setmpcsignals(plant,'ud',1,'mv',2);

%% Set up MPC
Ts = 0.01;
PredictionHorizon = 50;
ControlHorizon = 5;
dplant = c2d(plant, Ts)
mpcobj = mpc(dplant,Ts,PredictionHorizon,ControlHorizon);
mpcobj.Model.Nominal = struct('X', x0, 'U', u0, 'Y', y0, 'DX', [0 0]);

%% MPC parameters
mpcobj.MV.Min = -50;
mpcobj.MV.Max = 50;

mpcobj.MV.ScaleFactor = 100;
mpcobj.Weights.MVRate = 1;
mpcobj.Weights.OV = [1.2 1]; % weights are for x and theta respectively

%% Adaptive MPC Parameter Estimation
% Initial estimates
[num, den] = tfdata(dplant);
Aq = den{1};
Bq = num;


%% Disturbance Rejection
% Update the input disturbance model.
disturbance_model = getindist(mpcobj);
setindist(mpcobj,'model',disturbance_model*10);

%%
% Update the output disturbance model.
disturbance_model = getoutdist(mpcobj);
setoutdist(mpcobj,'model',disturbance_model*10);

%% Closed-Loop Simulation
% Validate the MPC design with a closed-loop simulation in Simulink.
open_system([mdlMPC '/Scope'])
sim(mdlMPC)

%%
% Close the Simulink model.
% bdclose(mdlMPC)
