
J = 0.02;
b = 0.1;
k = 1.0;

A = [  0,     1;
     -k/J, -b/J ];
B = [ 0;
      1/J ];
C = [ 1, 0 ];
D = 0;

sys = ss(A, B, C, D);

% Example: simulate step torque using lsim
t = linspace(0, 5, 1001);
u = ones(size(t));        % tau(t) = 1
x0 = [0; 0];
[y, t, x] = lsim(sys, u, t, x0);
