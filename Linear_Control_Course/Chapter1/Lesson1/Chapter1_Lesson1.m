a = 1.0;
b = 2.0;
K = 3.0;
r0 = 1.0;

h = 0.001;
t_final = 2.0;
N = floor(t_final / h);

t = linspace(0.0, t_final, N + 1);
y = zeros(1, N + 1);
u = zeros(1, N + 1);

y(1) = 0.0;

for k = 1:N
    e_k = r0 - y(k);
    u(k) = K * e_k;
    dy = -(a + b * K) * y(k) + b * K * r0;
    y(k + 1) = y(k) + h * dy;
end
u(N + 1) = u(N);

plot(t, y, 'LineWidth', 1.5); hold on;
y_ref = r0 * ones(size(t));
plot(t, y_ref, '--', 'LineWidth', 1.0);
xlabel('time [s]');
ylabel('output');
legend('y(t)', 'reference');
title('First-order closed-loop response (MATLAB simulation)');
grid on;

% In Simulink, one can build an equivalent diagram using:
%  - A Step block for r(t)
%  - A Sum block computing e(t) = r(t) - y(t)
%  - A Gain block with gain K
%  - A first-order plant implemented via Transfer Fcn or State-Space
% These blocks can be connected to mimic the feedback loop drawn earlier.
