
function tau_sat = ct_controller_step(q_meas, qd, qd_dot, qd_ddot)
%#codegen
% Global or persistent variables can store state between calls
% (for q_prev, qf, qdot_est, tau_prev)

persistent Ts Kp Kd tau_min tau_max dtau_min dtau_max ...
           q_prev qf qdot_est tau_prev alpha_vel beta_pos robot

if isempty(Ts)
    Ts = 0.001;
    n = numel(q_meas);

    Kp = diag(100 * ones(1,n));
    Kd = diag(20 * ones(1,n));

    tau_min = -50 * ones(n,1);
    tau_max =  50 * ones(n,1);
    dtau_min = -200 * Ts * ones(n,1);
    dtau_max =  200 * Ts * ones(n,1);

    omega_c_vel = 2 * pi * 20;   % 20 Hz cut-off
    omega_c_pos = 2 * pi * 5;    % 5 Hz cut-off

    alpha_vel = 1 / (1 + omega_c_vel * Ts);
    beta_pos  = 1 / (1 + omega_c_pos * Ts);

    q_prev = q_meas;
    qf = q_meas;
    qdot_est = zeros(n,1);
    tau_prev = zeros(n,1);

    % robot: RigidBodyTree created in base workspace
    robot = evalin('base', 'robot');
end

% 1) Filtering
qf = beta_pos * qf + (1 - beta_pos) * q_meas;
dq_raw = (q_meas - q_prev) / Ts;
qdot_est = alpha_vel * qdot_est + (1 - alpha_vel) * dq_raw;
q_prev = q_meas;

% 2) Errors
e_q = qd - qf;
e_qdot = qd_dot - qdot_est;

% 3) Dynamics using RigidBodyTree
M = massMatrix(robot, qf');
Cqdot = velocityProduct(robot, qf', qdot_est');
g = gravityTorque(robot, qf');

v = qd_ddot + Kd * e_qdot + Kp * e_q;
tau_pre = M * v + Cqdot' + g';

% 4) Rate limiting
du = tau_pre - tau_prev;
du = min(max(du, dtau_min), dtau_max);
tau_rl = tau_prev + du;

% 5) Saturation
tau_sat = min(max(tau_rl, tau_min), tau_max);
tau_prev = tau_sat;
