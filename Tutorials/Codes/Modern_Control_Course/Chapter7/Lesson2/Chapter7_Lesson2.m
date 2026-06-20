% ===== Code block 1 extracted from Chapter7/Lesson2.html =====
% Example system
A = [0 1; -2 -3];
B = [0; 1];
x0 = [1; 0];

% Input u(t): unit pulse on [0,2]
u = @(t) (t >= 0 && t <= 2);

% Forced response x_zs(t) = ∫_0^t exp(A*(t-s))*B*u(s) ds
x_zs = @(t) integral(@(s) expm(A*(t - s)) * B * u(s), 0, t, 'ArrayValued', true);

% Total state x(t) = expm(A*t)*x0 + x_zs(t)
t = linspace(0, 6, 601);
X = zeros(2, numel(t));
for k = 1:numel(t)
    tk = t(k);
    X(:,k) = expm(A*tk)*x0 + x_zs(tk);
end

disp(X(:,1));
disp(X(:,end));

% ===== Code block 2 extracted from Chapter7/Lesson2.html =====
A = [0 1; -2 -3];
B = [0; 1];
C = eye(2);
D = zeros(2,1);
sys = ss(A,B,C,D);

t = linspace(0, 6, 601);
u = double(t >= 0 & t <= 2);  % sampled pulse input
x0 = [1;0];

[y, t_out, x_out] = lsim(sys, u, t, x0);

% Compare x_out' (2xN) with your convolution result X
% (Assuming X computed as above)
err = max(vecnorm((x_out' - X), 2, 1));
disp(err);
