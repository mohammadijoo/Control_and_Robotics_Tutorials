syms q1 q2 l1 l2 xd yd real

% Forward kinematics
x = l1*cos(q1) + l2*cos(q1 + q2);
y = l1*sin(q1) + l2*sin(q1 + q2);

q = [q1; q2];
f = [x; y];

% Jacobian J(q)
J = jacobian(f, q);

% Cost phi(q) = 0.5 * ||f(q) - x_d||^2
xd_sym = sym('xd');
yd_sym = sym('yd');
e = f - [xd_sym; yd_sym];
phi = sym(0.5) * (e.' * e);

% Gradient and Hessian
grad_phi = gradient(phi, q);
H_phi = hessian(phi, q);

disp('J(q) ='); disp(J);
disp('grad_phi(q) ='); disp(grad_phi);
disp('H_phi(q) ='); disp(H_phi);

% Numeric evaluation
l1_val = 1.0; l2_val = 0.8;
xd_val = 1.2; yd_val = 0.3;
q_val = [0.5; -0.3];

J_num = double(subs(J, {l1, l2, xd_sym, yd_sym, q1, q2}, ...
                      {l1_val, l2_val, xd_val, yd_val, q_val(1), q_val(2)}));
grad_num = double(subs(grad_phi, {l1, l2, xd_sym, yd_sym, q1, q2}, ...
                                {l1_val, l2_val, xd_val, yd_val, q_val(1), q_val(2)}));

disp('J_num ='); disp(J_num);
disp('grad_num ='); disp(grad_num);
      
