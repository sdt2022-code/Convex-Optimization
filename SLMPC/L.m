function [A, B, C, D, U, Y, X, DX] = L(x_estimated,Theta_estimated, Xi, x_measured,Theta_measured)
%% First Way
% io = getlinio('CSTR_OpenLoop')
% 
% io(1) = linio([mdlPlant '/dF'],1,'openinput');
% io(2) = linio([mdlPlant '/F'],1,'openinput');
% io(3) = linio([mdlPlant '/Pendulum and Cart System'],1,'openoutput');
% io(4) = linio([mdlPlant '/Pendulum and Cart System'],3,'openoutput');
% % % the first state is cart position x, which is measured at this estimated
% % by the mpc model
% 
% opspec.States(1).Known = true;
% opspec.States(1).x = x_measured;
% % The third state is pendulum angle _theta_, which has a known initial
% % state of 0.
% opspec.States(3).Known = true;
% opspec.States(3).x = Theta_measured;
% 
% %%
% % Compute operating point using these specifications.
% options = findopOptions('DisplayReport',false);
% [op, op_report] = findop(mdlPlant,opspec,options);
% 
% x0 = [op_report.States(1).x; op_report.States(2).x; op_report.States(3).x; op_report.States(4).x];
% y0 = [op_report.Outputs(1).y;op_report.Outputs(2).y];
% u0 = [op_report.Inputs(1).u op_report.Inputs(2).u];
% 
% %%
% % Obtain the linear plant model at the specified operating point.
% plant = linearize(mdlPlant,op,io);
% 
% A = plant.A
% B = plant.B
% C = plant.C
% D = plant.D



%% Second way

%input = [df, f]
% X = states(estimated) 
% Y = Measured(Xand theta)

DX = zeros(4;1);


% Define constant outputs
Ts = 0.01;
C = [1,0,0,0;0,0,1,0];
D = zeros(2,1);


% Nominal U are obtained from measurements
U = [x_measured; Theta_measured];

% Nominal X and Y are obtained from estimated MPC states
Y = [x_estimated; Theta_estimated];
X = [x_estimated; Theta_estimated];

% Analytical linearization of mechanistic CSTR model (continuous time)
[A, Bo] = getContinuous(x_estimated,Theta_estimated, Xi, x_measured,Theta_measured);

% Convert continuous to discrete time
[A, Bo] = adasblocks_utilDicretizeModel(A, Bo, Ts);
B = Bo(:,3);


end

function getContinuous(x_estimated,Theta_estimated, Xi, x_measured,Theta_measured)

end


