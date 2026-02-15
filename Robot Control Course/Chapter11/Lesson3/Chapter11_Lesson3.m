
function [x_next, P_next] = robot_ekf_step(x, P, u, z, dt, Q, R)
% x, P : current state and covariance
% u    : control input (joint torques)
% z    : measurement [q_enc; ee_pos]
% dt   : sampling time
% Q, R : process and measurement noise covariances

n = numel(x) / 2;
q  = x(1:n);
dq = x(n+1:end);

% ----- Process model -----
qddot = robot_forward_dynamics(q, dq, u);  % user-supplied

q_pred  = q  + dt * dq;
dq_pred = dq + dt * qddot;
x_pred  = [q_pred; dq_pred];

F = numerical_jacobian(@(xx) f_process(xx, u, dt), x);

P_pred = F * P * F.' + Q;

% ----- Measurement model -----
z_pred = h_measurement(x_pred);

H = numerical_jacobian(@h_measurement, x_pred);

y = z - z_pred;
S = H * P_pred * H.' + R;
K = P_pred * H.' / S;

x_next = x_pred + K * y;
I = eye(size(P_pred));
P_next = (I - K * H) * P_pred * (I - K * H).' + K * R * K.';

end

function x_next = f_process(x, u, dt)
    % helper for numerical Jacobian
    n = numel(x) / 2;
    q  = x(1:n);
    dq = x(n+1:end);
    qddot = robot_forward_dynamics(q, dq, u);
    q_next  = q  + dt * dq;
    dq_next = dq + dt * qddot;
    x_next = [q_next; dq_next];
end

function z = h_measurement(x)
    global n_dof
    q  = x(1:n_dof);
    z_q  = q;
    ee   = forward_kinematics(q);  % 3x1
    z = [z_q; ee];
end

function J = numerical_jacobian(fun, x)
    fx = fun(x);
    m  = numel(fx);
    n  = numel(x);
    J  = zeros(m, n);
    eps = 1e-6;
    for i = 1:n
        dx = zeros(n,1);
        dx(i) = eps;
        f_plus = fun(x + dx);
        J(:, i) = (f_plus - fx) / eps;
    end
end
