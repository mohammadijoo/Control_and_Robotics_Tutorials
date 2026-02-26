% Assume 'robot' is a rigidBodyTree and 'endEffector' its end-effector name.
q = rand(robot.NumBodies, 1);   % joint configuration
J = geometricJacobian(robot, q', endEffector);  % 6 x n Jacobian
F = [Nx; Ny; Nz; Fx; Fy; Fz];                    % 6 x 1 wrench in base frame
tau = J' * F;                                    % n x 1 joint torques
      
