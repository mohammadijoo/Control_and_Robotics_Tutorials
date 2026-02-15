function [omega, alpha, a, ac] = forward_recursion(links, q, qd, qdd, g)
% FORWARD_RECURSION  Newton-Euler forward pass (velocities/accelerations)
%   links(i) has fields:
%     R  (3x3), p (3x1), rc (3x1), joint_type ('R' or 'P')

if nargin < 5
    g = [0; 0; 9.81];
end

n = numel(links);
z = [0; 0; 1];

omega = zeros(3, n);
alpha = zeros(3, n);
a = zeros(3, n);
ac = zeros(3, n);

omega_prev = zeros(3, 1);
alpha_prev = zeros(3, 1);
a_prev = -g;  % base acceleration includes gravity

for i = 1:n
    L = links(i);
    qi = q(i);
    qdi = qd(i);
    qddi = qdd(i);

    a_base = a_prev ...
           + cross(alpha_prev, L.p) ...
           + cross(omega_prev, cross(omega_prev, L.p));

    if L.joint_type == 'R'
        omega_i = L.R * omega_prev + z * qdi;
        alpha_i = L.R * alpha_prev + z * qddi + cross(omega_i, z * qdi);
        a_i = L.R * a_base;
    elseif L.joint_type == 'P'
        omega_i = L.R * omega_prev;
        alpha_i = L.R * alpha_prev;
        a_i = L.R * a_base ...
            + z * qddi ...
            + cross(2 * omega_i, z * qdi);
    else
        error('joint_type must be ''R'' or ''P''');
    end

    rc = L.rc;
    ac_i = a_i ...
         + cross(alpha_i, rc) ...
         + cross(omega_i, cross(omega_i, rc));

    omega(:, i) = omega_i;
    alpha(:, i) = alpha_i;
    a(:, i) = a_i;
    ac(:, i) = ac_i;

    omega_prev = omega_i;
    alpha_prev = alpha_i;
    a_prev = a_i;
end
end
      
