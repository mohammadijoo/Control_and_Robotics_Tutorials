function [hG, A] = centroidalMomentum(robot, q, qdot)
% robot: rigidBodyTree (floating-base or fixed-base with added base)
% q    : configuration vector
% qdot : generalized velocity vector (same length as q, with appropriate mapping)

% Number of bodies (excluding base)
bodies = robot.Bodies;
N = numel(bodies);

% Compute COM of each body and total mass
M = 0;
p_list = zeros(3, N);
m_list = zeros(1, N);

for i = 1:N
    bi = bodies{i};
    m_list(i) = bi.Mass;
    M = M + bi.Mass;
    p_list(:,i) = centerOfMass(robot, q, bi.Name);
end

pG = (p_list * m_list.') / M;  % system COM in world frame

% Assemble spatial inertias and Jacobians at COM
nq = numel(q);
A = zeros(6, nq);
for i = 1:N
    bi = bodies{i};
    % Spatial inertia in body frame (mass + inertia about COM)
    Ibody = bodySpatialInertia(bi); % user-defined helper (6x6)

    % Geometric Jacobian for body frame
    J = geometricJacobian(robot, q, bi.Name); % 6 x ndof

    % Transform inertia and Jacobian so that they are expressed at system COM
    % (approximation: neglecting small differences in orientation)
    % Shift linear part by (p_i - pG)
    r = p_list(:,i) - pG;
    X = spatialTransform(r); % 6x6 transform from body COM to system COM

    I_Gi = X.' * Ibody * X;
    J_Gi = X * J;

    A = A + I_Gi * J_Gi;
end

hG = A * qdot;
end
      
