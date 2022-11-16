function u = MPC(Ad_hat, Bd_hat, Q, R, N, y, yd)

% decision variables [ x[k+1] ... x[k+N] u[k] u[k+N-1] ] in R^(5N)
% each x is 4 dimensional and each u is 1 dimensional => 5N elements

%% define cost
H = zeros(5*N, 5*N);
f = zeros(5*N, 1);
Qyd = Q*yd;
for i = 1:N
    H(4*i-3:4*i, 4*i-3:4*i) = Q;
    H(4*N+i, 4*N+i) = R;
    f(4*i-3:4*i) = -Qyd;
end

%% define equality constraints (dynamics constraints)
M = zeros(4*N, 5*N);
for i =1:N
    M(4*i-3:4*i, 4*N+i) = Bd_hat;
end
for i=1:N-1
    M(4*i+1:4*i+4, 4*i-3:4*i) = Ad_hat;
end
Aeq = eye(4*N,5*N) - M;
Beq = [Ad_hat*y; zeros(4*N-4,1)];

%% lower and upper bounds
u_lower = -10;
u_upper = 10;
x_lower = yd - [4; 2; 0.6; 1];
x_upper = yd + [4; 2; 0.6; 1];

lb = zeros(5*N,1);
ub = zeros(5*N,1);
for i =1:N
    lb(4*i-3:4*i) = x_lower;
    lb(4*N+i) = u_lower;
    ub(4*i-3:4*i) = x_upper;
    ub(4*N+i) = u_upper;

end

options = optimset('Display', 'off');
x = quadprog(H, f, [], [], Aeq, Beq, [], [], [], options);
disp("x")
x(4*N-3:4*N) - yd

u = x(4*N+1:5*N);

end