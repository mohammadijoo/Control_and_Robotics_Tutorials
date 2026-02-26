syms q1 q2 dq1 dq2 ddq1 ddq2 real
syms m1 m2 l1 l2 I1 I2 g real

q  = [q1; q2];
dq = [dq1; dq2];

% Kinetic energy of a simple 2R planar arm
x1 = (l1/2)*cos(q1);
y1 = (l1/2)*sin(q1);
x2 = l1*cos(q1) + (l2/2)*cos(q1 + q2);
y2 = l1*sin(q1) + (l2/2)*sin(q1 + q2);

J1 = jacobian([x1; y1], q);
J2 = jacobian([x2; y2], q);

v1 = J1*dq;
v2 = J2*dq;

T_trans = 0.5*m1*(v1.'*v1) + 0.5*m2*(v2.'*v2);
w1 = dq1;
w2 = dq1 + dq2;
T_rot = 0.5*I1*w1^2 + 0.5*I2*w2^2;

T = simplify(T_trans + T_rot);
V = m1*g*y1 + m2*g*y2;
L = T - V;

% Metric M(q) = d^2 T / (d dq^2)
M = simplify(hessian(T, dq));

% Euler-Lagrange equations
ddq = [ddq1; ddq2];
Q   = sym('Q', [2 1]);  % generalized forces

EL = sym(zeros(2,1));
for k = 1:2
    dL_ddqk   = functionalDerivative(L, dq(k));
    ddt_term  = jacobian(dL_ddqk, [q; dq])*[dq; ddq];
    dL_dqk    = functionalDerivative(L, q(k));
    EL(k)     = simplify(ddt_term - dL_dqk - Q(k));
end

EL = simplify(EL);

% Solve for ddq symbolically: M(q)*ddq + C(q,dq)*dq + g(q) = Q
A = jacobian(EL, ddq);     % should coincide with M(q)
b = simplify(EL - A*ddq);  % remaining terms C(q,dq)*dq + g(q) - Q

% Export A and b as MATLAB functions usable inside Simulink
matlabFunction(A, 'Vars', {q1,q2,m1,m2,l1,l2,I1,I2}, 'File', 'M_metric.m');
matlabFunction(b, 'Vars', {q1,q2,dq1,dq2,m1,m2,l1,l2,I1,I2,g,Q}, 'File', 'Cgrav_forces.m');
      
