
function tau = hybridPositionForceControl(q, qdot, x_d, xdot_d, Fd_n, robot, n_world, Kp_tan, Kd_tan, kf)
%HYBRIDPOSITIONFORCECONTROL Compute joint torques for hybrid position/force control.
%  q, qdot   : joint state (nx1)
%  x_d, xdot_d : desired end-effector position/velocity (3x1)
%  Fd_n      : desired normal force (scalar)
%  robot     : robot model with forwardKinematics, jacobian, inverseDynamics
%  n_world   : 3x1 unit normal vector (world frame)

if nargin < 8 || isempty(Kp_tan)
    Kp_tan = 200 * eye(3);
end
if nargin < 9 || isempty(Kd_tan)
    Kd_tan = 40 * eye(3);
end
if nargin < 10 || isempty(kf)
    kf = 50;
end

n = n_world(:) / norm(n_world);
S_f = n * n.';
S_p = eye(3) - S_f;

x = robot.forwardKinematics(q);   % 3x1
J = robot.jacobian(q);            % 3xn
xdot = J * qdot;

% Project desired position onto tangent plane
delta = x_d - x;
normal_component = (n.' * delta) * n;
x_d_proj = x_d - normal_component;

e_p = S_p * (x_d_proj - x);
edot_p = S_p * (xdot_d - xdot);

F_t_cmd = S_p * (Kp_tan * e_p + Kd_tan * edot_p);

F_meas = robot.forceSensor();     % 3x1
F_n = n.' * F_meas;
e_f = Fd_n - F_n;
F_n_cmd = Fd_n + kf * e_f;
F_n_vec = F_n_cmd * n;

F_cmd = F_t_cmd + F_n_vec;

tau = J.' * F_cmd;
tau_bias = robot.inverseDynamics(q, qdot, zeros(size(q)));
tau = tau + tau_bias;
end
