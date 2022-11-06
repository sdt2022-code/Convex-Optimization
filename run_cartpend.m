clear all, close all, clc

%% Model Parameters (Estimates)
m_hat = 1;
M_hat = 5;
L_hat = 2;
g_hat = -10;
d_hat =1;

%% True Parameters
m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

%% Controls & Plotting Variables
dt = 0.001;
sim_time = 10;
tspan = 0:dt:10;
n = size(tspan,2);
u_t = zeros(1, n);

y0 = [-3; 0; pi+.1; 0];
yd = [1; 0; pi; 0];
yd_extended = zeros(n, 4);
for i = 1:n
    yd_extended(i,:) = yd';
end

controller_type = "LQR";

%% Run Experiment
y_t = zeros(n, 4);
y = y0;
if controller_type == "LQR"
    % define parameters for LQR control
    [A_hat, B_hat] = linearization(m_hat, M_hat, L_hat, g_hat, d_hat);
    Q = [1 0 0 0;
         0 1 0 0;
         0 0 10 0;
         0 0 0 100];
    R = 0.01;
    K = lqr(A_hat,B_hat,Q,R);
    
    % simulate controller
    for i = 1:n
        y_t(i,:) = y';
        u_t(i) = LQR_controller(K, y, yd);
        dy = cartpend(y, m, M, L, g, d, u_t(i));
        y = y + dy*dt;
    end
end

%% Draw Animation
figure()
for k=1:100:n
    drawcartpend(y_t(k,:),m,M,L);
end

%% Plot States
figure()
plot(tspan,y_t)
hold on
plot(tspan, yd_extended, '--')
title("Pendulum State Over Time")
xlabel("Time [s]")
ylabel("State, Units = [m, m/s, rad, rad/s]")
legend(["Position", "Velocity", "Angle", "Angular Velocity", ...
        "Desired Position", "Desired Velocity", "Desired Angle", "Designed Angular Velocity"])

%% Plot Input
figure()
plot(tspan,u_t)
title("Input Value Over Time")
xlabel("Time [s]")
ylabel("Force on cart [N]")