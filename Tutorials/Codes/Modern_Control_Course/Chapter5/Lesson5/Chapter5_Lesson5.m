% Unscaled system
A = [0 1 0;
     0 0 1;
    -1 -10 -1000];
B = [0;0;1];
C = [1 0 0];
D = 0;

sys = ss(A,B,C,D);

% Scaling matrix S = diag(1, omega, omega^2)
omega = 10;
S = diag([1, omega, omega^2]);

As = inv(S)*A*S;
Bs = inv(S)*B;
Cs = C*S;
Ds = D;

sys_s = ss(As,Bs,Cs,Ds);

% Verify output equivalence via simulation
t = linspace(0,0.02,400);
u = ones(size(t));     % step input
x0 = [0;0;0];
z0 = inv(S)*x0;

[y1, t1, x1] = lsim(sys,   u, t, x0);
[y2, t2, z2] = lsim(sys_s, u, t, z0);

% Map z back to x and compute output from original C for comparison
x_from_z = (S*z2')';
y2_phys = (C*x_from_z')';

max_err = max(abs(y1 - y2_phys));
disp(["max |y1 - y2_phys| = ", num2str(max_err)]);
