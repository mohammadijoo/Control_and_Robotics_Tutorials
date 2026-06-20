function M = mobilityGruebler(lambda_dim, N, joint_dofs)
%MOBILITYGRUEBLER  Compute mechanism mobility using the Gruebler/Kutzbach formula.
%   lambda_dim : 6 for spatial, 3 for planar
%   N          : number of links including ground
%   joint_dofs : vector of joint DOFs f_j

    J = numel(joint_dofs);
    M = lambda_dim * (N - 1 - J) + sum(joint_dofs);
end

% Example: planar 4-bar
M4 = mobilityGruebler(3, 4, ones(1,4))

% Rank-based DOF from constraint Jacobian
function dof = dofFromConstraints(Jphi)
%Jphi: m x n constraint Jacobian
    r = rank(Jphi);
    n = size(Jphi, 2);
    dof = n - r;
end

% Example constraint Jacobian for q1 + q2 + q3 = const
Jphi = [1 1 1];
dofJ = dofFromConstraints(Jphi)
      
