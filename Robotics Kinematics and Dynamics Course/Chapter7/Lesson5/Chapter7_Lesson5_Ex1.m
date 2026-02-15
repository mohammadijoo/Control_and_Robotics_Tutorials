function [relErr, Jnum, Jana] = checkJacobianRigidBodyTree(robot, q, baseName, eeName, h)

    if nargin < 5
        h = 1e-6;
    end

    q = q(:).';
    fkHandle = @(qcol) pose6FromTree(robot, qcol.', baseName, eeName);

    % Numeric Jacobian (6 x n)
    Jnum = numericJacobianVec(fkHandle, q(:), h);

    % Analytic geometric Jacobian (base frame)
    Jana = geometricJacobian(robot, q, eeName);

    diff = Jnum - Jana;
    err  = norm(diff, "fro");
    ref  = norm(Jana, "fro") + 1e-8;
    relErr = err / ref;
end

function x = pose6FromTree(robot, q, baseName, eeName)
    T = getTransform(robot, q.', eeName, baseName); % 4x4
    p = T(1:3, 4);
    rpy = rotm2eul(T(1:3,1:3), "ZYX").';  % 3x1
    x = [p; rpy];
end
      
