function [ddq, lambda] = floating_leg_dynamics(q, dq, tau, p)
% q: 5x1, dq: 5x1, tau: 2x1, p: struct of parameters

M  = M_leg(q, p);      % 5x5
h  = h_leg(q, dq, p);  % 5x1
Jc = Jc_leg(q, p);     % 2x5
Jc_dot_dq = Jcdot_leg(q, dq, p); % 2x1

S = [0 0 0 1 0;
     0 0 0 0 1];

A = [M, -Jc';
     Jc, zeros(2,2)];

rhs = [S' * tau - h;
       -Jc_dot_dq];

sol = A \ rhs;
ddq = sol(1:5);
lambda = sol(6:7);
end
      
