syms q1 q2 l1 l2 real
syms fx fy real

% End-effector position
x = l1*cos(q1) + l2*cos(q1 + q2);
y = l1*sin(q1) + l2*sin(q1 + q2);

J = jacobian([x; y], [q1; q2]);   % 2x2 translational Jacobian

f = [fx; fy];
tau = simplify(J.' * f);         % generalized torques

disp('Jacobian J(q):');
disp(J);
disp('Generalized torques tau(q,f):');
disp(tau);

% Example numerical evaluation:
l1v = 1.0; l2v = 0.8;
q1v = 0.5; q2v = -0.3;
fxv = 10.0; fyv = 5.0;

Jnum = double(subs(J, {q1,q2,l1,l2}, {q1v,q2v,l1v,l2v}));
taunum = double(subs(tau, {q1,q2,l1,l2,fx,fy}, {q1v,q2v,l1v,l2v,fxv,fyv}));

disp('Numeric J:'); disp(Jnum);
disp('Numeric tau:'); disp(taunum);

% In Simulink, these expressions can be implemented in a MATLAB Function block
% that takes (q1,q2,fx,fy) as inputs and outputs tau1,tau2.
      
