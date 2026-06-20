% Chapter18_Lesson3.m
% Port-Hamiltonian mass-spring-damper simulation (RK4) + energy-balance check
% Can be run in MATLAB or GNU Octave.

clear; clc;

m = 1.5;
k = 12.0;
d = 0.8;
omega = 1.4;

u = @(t) sin(omega*t);
H = @(x) 0.5*k*x(1)^2 + 0.5*(x(2)^2)/m;
yout = @(x) x(2)/m;
diss = @(x) d*(x(2)/m)^2;
f = @(t,x) [x(2)/m;
           -k*x(1) - d*(x(2)/m) + u(t)];

T = 20.0;
h = 0.002;
t = 0:h:T;
N = numel(t);

x = zeros(2,N);
x(:,1) = [0.15; 0.0];

Hvals = zeros(1,N);
yvals = zeros(1,N);
uvals = zeros(1,N);
dvals = zeros(1,N);

for i = 1:N
    xi = x(:,i);
    ti = t(i);
    Hvals(i) = H(xi);
    yvals(i) = yout(xi);
    uvals(i) = u(ti);
    dvals(i) = diss(xi);

    if i < N
        k1 = f(ti, xi);
        k2 = f(ti + 0.5*h, xi + 0.5*h*k1);
        k3 = f(ti + 0.5*h, xi + 0.5*h*k2);
        k4 = f(ti + h, xi + h*k3);
        x(:,i+1) = xi + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
    end
end

supplyVals = yvals .* uvals;
rhs = -dvals + supplyVals;
rhsIntegral = trapz(t, rhs);
residual = (Hvals(end) - Hvals(1)) - rhsIntegral;

fprintf('Final state [q, p] = [%g, %g]\n', x(1,end), x(2,end));
fprintf('Initial energy     = %.8f\n', Hvals(1));
fprintf('Final energy       = %.8f\n', Hvals(end));
fprintf('Energy balance residual = %.4e\n', residual);

% Optional CSV export
M = [t(:), x(1,:).', x(2,:).', Hvals(:), uvals(:), yvals(:), dvals(:), supplyVals(:)];
writematrix(M, 'Chapter18_Lesson3_matlab_output.csv');

figure;
plot(t, x(1,:), t, x(2,:));
grid on;
xlabel('Time [s]');
ylabel('States');
legend('q(t)', 'p(t)');
title('Port-Hamiltonian states');

figure;
plot(t, Hvals, t, Hvals(1) + cumtrapz(t, rhs), '--');
grid on;
xlabel('Time [s]');
ylabel('Energy');
legend('H(t)', 'H(0)+int(-diss+y*u)dt');
title('Energy balance verification');

% Simulink note:
% A Simulink model can be built with two Integrator blocks (q, p),
% a MATLAB Function block implementing pH dynamics, and scopes for q, p, H.
