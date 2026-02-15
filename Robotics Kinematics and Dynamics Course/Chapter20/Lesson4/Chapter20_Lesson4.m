syms q dq I b k tau real
x = [q; dq];

f = [dq;
     (tau - b*dq - k*q)/I];

% Jacobians needed for sensitivities
A = jacobian(f, x);   % df/dx
B = jacobian(f, I);   % df/dI

disp('A(q,dq,I) = ');
disp(A);
disp('B(q,dq,I) = ');
disp(B);

% Generate numeric functions for use in simulation loops
matlabFunction(f, 'File', 'f_pendulum', 'Vars', {q, dq, tau, I, b, k});
matlabFunction(A, 'File', 'A_pendulum', 'Vars', {q, dq, tau, I, b, k});
matlabFunction(B, 'File', 'B_pendulum', 'Vars', {q, dq, tau, I, b, k});
      
