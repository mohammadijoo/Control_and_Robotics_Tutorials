
function v_safe = cbf_safety_filter(q, v_nom, robot)
% CBF-based safety filter at joint-velocity level
% q      : current joint positions (nx1)
% v_nom  : nominal joint velocities (nx1)
% robot  : rigidBodyTree (for FK and Jacobian)

n = numel(q);
gamma_joint = 5.0;
gamma_ws    = 5.0;
lambda_slack = 1e3;

% Joint limits from robot model
q_min = robot.homeConfiguration;
q_max = robot.homeConfiguration;
for i = 1:n
    q_min(i) = robot.Bodies{i}.Joint.PositionLimits(1);
    q_max(i) = robot.Bodies{i}.Joint.PositionLimits(2);
end

% Forbidden sphere
c_obs = [0.5; 0.5; 0.0];
R_obs = 0.3;

% Build constraints A v <= b
A = [];
b = [];

% Joint limits
for i = 1:n
    h_min = q(i) - q_min(i);
    e_i   = zeros(1, n); e_i(i) = 1;
    % -e_i v <= gamma_joint * h_min
    A = [A; -e_i];
    b = [b; gamma_joint * h_min];

    h_max = q_max(i) - q(i);
    % e_i v <= gamma_joint * h_max
    A = [A; e_i];
    b = [b; gamma_joint * h_max];
end

% Workspace avoidance using end-effector body
eeName = robot.BodyNames{end};
T = getTransform(robot, q, eeName);
p = T(1:3, 4);
J = geometricJacobian(robot, q, eeName);
Jp = J(1:3, :);  % translational part

diff = p - c_obs;
h_ws = diff' * diff - R_obs^2;

% 2 diff^T Jp v + gamma_ws * h_ws >= 0
% => -(2 diff^T Jp) v <= gamma_ws * h_ws
a_ws = -2 * diff' * Jp;
A = [A; a_ws];
b = [b; gamma_ws * h_ws];

% QP: min 0.5 [v;delta]' H [v;delta] + f' [v;delta]
H = blkdiag(eye(n), lambda_slack);
f = [-v_nom; 0];

m = size(A, 1);
A_qp = [A, -ones(m, 1);  % A v - delta <= b
        zeros(1, n), -1]; % -delta <= 0
b_qp = [b; 0];

% No equality constraints
Aeq = [];
beq = [];

opts = optimoptions('quadprog', 'Display', 'off');
x_opt = quadprog(H, f, A_qp, b_qp, Aeq, beq, [], [], [], opts);
v_safe = x_opt(1:n);
