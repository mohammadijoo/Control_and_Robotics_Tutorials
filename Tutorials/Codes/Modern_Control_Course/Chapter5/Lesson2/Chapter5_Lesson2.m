% ===== Code block 1 extracted from Chapter5/Lesson2.html =====
function [A,B,C,D] = companion_from_ode(a,b)
% Build companion matrices for:
%   y^(n) + a(n) y^(n-1) + ... + a(2) y' + a(1) y = b u
% Input a is [a0 a1 ... a(n-1)] in the lesson's notation.

a = a(:).';              % row
n = length(a);
A = zeros(n,n);
A(1:n-1,2:n) = eye(n-1);
A(n,:) = -a;             % [-a0 ... -a(n-1)]
B = zeros(n,1); B(n) = b;
C = zeros(1,n); C(1) = 1;
D = 0;
end

% Example: y'' + 3 y' + 2 y = u
[A,B,C,D] = companion_from_ode([2 3], 1);

sys = ss(A,B,C,D);

t = linspace(0,5,400);
u = ones(size(t));       % step
x0 = [0;0];              % [y(0); y'(0)]
y = lsim(sys,u,t,x0);

disp(A); disp(B);
fprintf('y(tf) = %.6f\n', y(end));

% ===== Code block 2 extracted from Chapter5/Lesson2.html =====
% Sketch: programmatic Simulink construction (outline)
% new_system('companion_model'); open_system('companion_model');
% Add blocks: In1 (u), Sum, Gains for -a_i, Integrators (n), Out1 (y)
% Wire:
%   x1dot = x2, x2dot = x3, ..., x(n-1)dot = xn
%   xndot = sum(-a_i * xi) + b*u
% Output y = x1
% Then set solver options and simulate via sim('companion_model')
