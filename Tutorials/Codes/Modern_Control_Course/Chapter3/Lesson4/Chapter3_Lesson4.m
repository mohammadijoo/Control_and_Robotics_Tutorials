% ===== Code block 1 extracted from Chapter3/Lesson4.html =====
% LTI example: Phi(t) = expm(A*(t-t0))
A = [0 1; -2 -3];
t0 = 0;
t  = 1.25;
Phi = expm(A*(t-t0));

x0 = [1; 0];
x_t = Phi*x0;

disp('Phi(t) ='); disp(Phi);
disp('x(t) ='); disp(x_t);

% LTV example: integrate Phi_dot = A(t)*Phi, Phi(t0)=I using ode45
Afun = @(tt) [0 1; -(2 + 0.1*tt) -3];
n = 2;

phi0 = reshape(eye(n), [], 1); % vec(Phi(t0))

ode = @(tt, phi_vec) reshape( Afun(tt) * reshape(phi_vec, n, n), [], 1 );

tspan = [0 2];
opts = odeset('RelTol',1e-9,'AbsTol',1e-12);
[tt, phi_sol] = ode45(ode, tspan, phi0, opts);

Phi_tf = reshape(phi_sol(end,:).', n, n);
x_tf = Phi_tf * x0;

disp('Phi(2) approx ='); disp(Phi_tf);
disp('x(2) approx ='); disp(x_tf);

% ===== Code block 2 extracted from Chapter3/Lesson4.html =====
% Offline assembly idea after n runs (pseudo-structure)
% Suppose logs are stored as x1(t), x2(t), ..., xn(t) each size n-by-N
% Then Phi(t_k) = [x1(:,k) x2(:,k) ... xn(:,k)] for each sample k.
