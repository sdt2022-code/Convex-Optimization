function [A, B] = linearization(m, M, L, g, d)

s = 1; % pendulum up (s=1)

A = [0 1 0 0;
    0 -d/M m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) s*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; s*1/(M*L)];

end