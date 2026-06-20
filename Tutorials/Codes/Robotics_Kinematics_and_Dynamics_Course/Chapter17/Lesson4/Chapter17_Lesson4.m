function [qddot, lambda] = contact_dynamics(M, h, Jc, tau, f_ext, qd, ac_des)
%CONTACT_DYNAMICS Solve constrained dynamics via KKT system.
%   M*qddot + h = tau + f_ext + Jc.'*lambda
%   Jc*qddot + Jc_dot*qd = ac_des
%
% Inputs:
%   M      (n x n)     inertia matrix
%   h      (n x 1)     bias term
%   Jc     (m x n)     contact Jacobian
%   tau    (n x 1)     generalized effort
%   f_ext  (n x 1)     external forces
%   qd     (n x 1)     velocities
%   ac_des (m x 1)     desired constraint acceleration (0 for rigid contact)
%
% Outputs:
%   qddot  (n x 1)     accelerations
%   lambda (m x 1)     contact forces

if nargin < 7 || isempty(ac_des)
    ac_des = zeros(size(Jc,1),1);
end

n = size(M,1);
m = size(Jc,1);

% Approximate Jc_dot*qd as zero for low-speed motions
Jc_dot_qd = zeros(m,1);

KKT = [M, -Jc.';
       Jc, zeros(m,m)];

rhs = [tau + f_ext - h;
       ac_des - Jc_dot_qd];

sol = KKT \ rhs;
qddot  = sol(1:n);
lambda = sol(n+1:end);
end
      
