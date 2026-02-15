
nq = 7;  % number of joints
mx = 6;  % task-space dimension

Jx = randn(mx, nq);  % example Jacobian
Jp = eye(nq);       % posture task

x_err    = randn(mx, 1);
xdot_ref = zeros(mx, 1);
Kx       = 5 * eye(mx);

Wx = eye(mx);
Wp = 0.1 * eye(nq);

qdot_nom = zeros(nq, 1);
qdot_ref = zeros(nq, 1);

v_x = xdot_ref - Kx * x_err;

w1 = 1000;     % high priority weight
w2 = 1.0;      % low priority weight
lambda_reg = 1e-3;

H = w1 * (Jx' * Wx' * Wx * Jx) + ...
    w2 * (Jp' * Wp' * Wp * Jp) + ...
    lambda_reg * eye(nq);

f = -w1 * (Jx' * Wx' * Wx * v_x) - ...
    -w2 * (Jp' * Wp' * Wp * qdot_ref) - ...
    lambda_reg * qdot_nom;

qdot_min = -0.5 * ones(nq, 1);
qdot_max =  0.5 * ones(nq, 1);

A   = [ eye(nq); -eye(nq) ];
b   = [ qdot_max; -qdot_min ];

% No equality constraints in this example
Aeq = [];
beq = [];

options = optimoptions('quadprog','Display','none');

[qdot_star, fval, exitflag] = quadprog(H, f, A, b, Aeq, beq, [], [], [], options);

disp('exitflag = '), disp(exitflag)
disp('qdot_star = '), disp(qdot_star)
