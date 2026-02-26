function demo_configurations()
    % Rigid body configuration as (R, p)
    R = eye(3);
    p = zeros(3,1);
    q = struct('R', R, 'p', p);

    fprintf('Valid rotation: %d\n', isRotationMatrix(q.R));

    % Joint-space configuration for planar 2R arm
    theta1 = 0.5;
    theta2 = -0.3;
    q_joint = [theta1; theta2];
    disp('Joint configuration q:');
    disp(q_joint);

    % With Robotics System Toolbox, one would instead use:
    % robot = rigidBodyTree;
    % ... (add bodies and joints)
    % q_struct = homeConfiguration(robot);
end

function flag = isRotationMatrix(R)
    I = eye(3);
    RT_R = R' * R;
    orth_error = norm(RT_R - I, 'fro');
    det_error = abs(det(R) - 1.0);
    tol = 1e-6;
    flag = (orth_error <= tol) && (det_error <= tol);
end
      
