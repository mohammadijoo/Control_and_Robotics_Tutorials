% Chapter19_Lesson2.m
% Separation of Variables & Modal Decomposition (1D Heat Equation, Dirichlet BC)
%
% PDE: u_t = alpha * u_xx, x in (0,L), u(0,t)=u(L,t)=0
% Series: u(x,t) = sum_{n=1..N} a_n exp(-alpha*(n*pi/L)^2 t) sin(n*pi x/L)
%
% This script computes coefficients a_n numerically for u0(x)=x(L-x),
% then plots u(x,t) at several times.

clear; clc;

L = 1.0;
alpha = 0.02;
N = 40;             % modes
M = 5001;           % integration points for coefficients
x_int = linspace(0, L, M);

u0 = @(x) x .* (L - x);

% Compute coefficients: a_n = (2/L) * int_0^L u0(x)*sin(n*pi*x/L) dx
a = zeros(N,1);
for n = 1:N
    phi = sin(n*pi*x_int/L);
    a(n) = (2/L) * trapz(x_int, u0(x_int) .* phi);
end

% Evaluate on spatial grid
x = linspace(0, L, 200);

times = [0.0, 1.0, 2.5, 5.0, 6.0];
figure; hold on; grid on;
for k = 1:numel(times)
    t = times(k);
    u = zeros(size(x));
    for n = 1:N
        lam = (n*pi/L)^2;
        u = u + a(n) * exp(-alpha*lam*t) * sin(n*pi*x/L);
    end
    plot(x, u, 'DisplayName', sprintf('t=%.2f', t));
end
xlabel('x'); ylabel('u(x,t)');
title('1D Heat equation via modal decomposition (Dirichlet BC)');
legend('show');

% ---- Simulink hint (modal truncation as finite-dimensional LTI) ----
% A = -alpha * diag((1:N).^2*pi^2/L^2)
% qdot = A*q + B*u_in(t),  y = C*q  (choose C to reconstruct u at sensors)
% Use "State-Space" block with matrices A,B,C,D.
