
function qdd_star = contactConsistentAcc(M, Jc, Jc_dot_qdot, qdd_des)
%CONTACTCONSISTENTACC Compute contact-consistent acceleration
%   M            : (n,n) mass matrix
%   Jc           : (nc,n) contact Jacobian
%   Jc_dot_qdot  : (nc,1) = Jc_dot(q,qdot) * qdot
%   qdd_des      : (n,1) unconstrained acceleration

Minv = inv(M);
Lambda_c = inv(Jc * Minv * Jc.');      % operational contact inertia
Jc_dyn_pinv = Minv * Jc.' * Lambda_c;  % M-consistent pseudo-inverse
Nc = eye(size(M,1)) - Jc_dyn_pinv * Jc;

qdd_star = Nc * qdd_des - Jc_dyn_pinv * Jc_dot_qdot;
end
