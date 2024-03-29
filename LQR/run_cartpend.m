clear all, close all, clc

%% Model Parameters (Estimates)
m_hat = 1;
M_hat = 1;
L_hat = 0.5;
g_hat = -9.81;
d_hat = 5;

%% True Parameters
m = 1;
M = 1;
L = 0.5;
g = -9.81;
d = 5;

%% Controls & Plotting Variables
dt = 0.001;
sim_time = 10;
tspan = 0:dt:10;
n = size(tspan,2);
u_t = zeros(1, n);

y0 = [0; 0; pi; 0];
yd = [10; 0; pi; 0];
yd_extended = zeros(n, 4);
for i = 1:n
    yd_extended(i,:) = yd' - [0 0 pi 0];
end

controller_type = "LQR";
% controller_type = "MPC"
% controller_type = "AMPC"

%% Run Experiment
y_t = zeros(n, 4);
y = y0;
if controller_type == "LQR"
    % define parameters for LQR control
    [A_hat, B_hat] = linearization(m_hat, M_hat, L_hat, g_hat, d_hat);
    Q = [1 0 0 0;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1];
    R = 0.01;
    K = lqr(A_hat,B_hat,Q,R);
    
    % simulate controller
    for i = 1:n
        y_t(i,:) = y' - [0 0 0 0];
        u_t(i) = LQR_controller(K, y, yd);
%         u_t(i) = clamp(u_t(i), -50, 50);
        dy = cartpend(y, m, M, L, g, d, u_t(i));
        y = y + dy*dt;
    end
elseif controller_type == "MPC"
    % define parameters for MPC control
    [A_hat, B_hat] = linearization(m_hat, M_hat, L_hat, g_hat, d_hat);
    Q = [100 0 0 0;
         0 0 0 0;
         0 0 100 0;
         0 0 0 0];
    R = 1;
    N = 10;

    Ts = 0.1;
    dlti = c2d(ss(A_hat, B_hat, eye(4), 0), Ts);
    Ad_hat = dlti.A;
    Bd_hat = dlti.B;

    MPC_u = zeros(n, N);
    
    % simulate controller
    for i = 1:n
        y_t(i,:) = y';
        if mod(i, Ts / dt) == 1 % compute MPC
            u_vec = MPC(Ad_hat, Bd_hat, Q, R, N, y, yd);
            u_t(i) = u_vec(1);
        else % zero order hold
            u_t(i) = u_t(i-1);
        end
        % u_t(i) = clamp(u_t(i), -10, 10);
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
plot(tspan,y_t- pi * [zeros(n,2) ones(n,1) zeros(n,1)])
hold on
plot(tspan, yd_extended, '--')
title("Pendulum State Over Time")
xlabel("Time [s]")
ylabel("State, Units = [m, m/s, rad, rad/s]")
axis([0 10 -6 12])
legend(["Position", "Velocity", "Angle", "Angular Velocity", ...
        "Desired Position", "Desired Velocity", "Desired Angle", "Designed Angular Velocity"])

%% Plot Input
figure()
plot(tspan,u_t)
title("Input Value Over Time")
xlabel("Time [s]")
ylabel("Force on cart [N]")

function y = clamp(x, l, u)
    y = min(max(x,l), u);
end