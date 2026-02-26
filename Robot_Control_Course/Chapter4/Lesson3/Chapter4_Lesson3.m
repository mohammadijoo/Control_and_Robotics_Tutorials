
function qdot = redundancy_control(q, x, xdot, x_d, xdot_d_ff, Kp, Kd, k_h)
%REDUNDANCY_CONTROL Velocity-level null-space redundancy control.
%
% q, x, xdot, x_d, xdot_d_ff: column vectors
% Kp, Kd: task-space gain matrices
% k_h: null-space gradient descent gain

    % User-defined Jacobian and secondary gradient
    J = jacobian_robot(q);        % m x n
    grad_h = secondary_gradient(q); % n x 1

    % Task-space velocity command
    x_error = x_d - x;
    xdot_error = xdot_d_ff - xdot;
    xdot_d = xdot_d_ff + Kp * x_error + Kd * xdot_error;

    % Moore-Penrose pseudoinverse
    JJt = J * J.';
    Jsharp = J.' / JJt;  % J^T * (J J^T)^{-1}

    % Null-space projector
    [m, n] = size(J);
    N = eye(n) - Jsharp * J;

    % Joint velocities
    qdot_task = Jsharp * xdot_d;
    qdot_null = -k_h * grad_h;

    qdot = qdot_task + N * qdot_null;
end
