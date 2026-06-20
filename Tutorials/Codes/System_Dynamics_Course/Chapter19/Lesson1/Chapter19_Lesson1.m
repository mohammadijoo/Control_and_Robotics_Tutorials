% Chapter19_Lesson1.m
% 1D heat/diffusion and wave equations (explicit finite differences)

clear; clc; close all;

%% Heat / diffusion equation: u_t = alpha u_xx
L = 1.0; alpha = 0.2;
Nx = 81; x = linspace(0, L, Nx); dx = x(2) - x(1);
r = 0.45;                % stability: r <= 0.5
dt = r * dx^2 / alpha;
Nt = floor(0.25 / dt);

u = sin(pi*x) + 0.2*sin(3*pi*x);
u(1) = 0; u(end) = 0;

figure; hold on;
for n = 0:Nt
    if mod(n, max(1, floor(Nt/4))) == 0
        plot(x, u, 'DisplayName', sprintf('t = %.3f', n*dt));
    end
    un = u;
    u(2:end-1) = un(2:end-1) + alpha*dt/dx^2 * (un(3:end) - 2*un(2:end-1) + un(1:end-2));
    u(1) = 0; u(end) = 0;
end
grid on; xlabel('x'); ylabel('u(x,t)'); title('Heat / Diffusion Equation'); legend;

%% Wave equation: u_tt = c^2 u_xx
c = 1.0;
Nxw = 201; xw = linspace(0, L, Nxw); dxw = xw(2) - xw(1);
s = 0.95;                % CFL: s <= 1
dtw = s * dxw / c;
Ntw = floor(1.0 / dtw);

up = exp(-180*(xw - 0.35).^2); up(1) = 0; up(end) = 0;
uc = up;
cfl2 = (c*dtw/dxw)^2;
uc(2:end-1) = up(2:end-1) + 0.5*cfl2 * (up(3:end) - 2*up(2:end-1) + up(1:end-2));

figure; hold on;
for n = 0:Ntw
    if mod(n, max(1, floor(Ntw/4))) == 0
        plot(xw, uc, 'DisplayName', sprintf('t = %.3f', n*dtw));
    end
    unext = zeros(size(uc));
    unext(2:end-1) = 2*uc(2:end-1) - up(2:end-1) + cfl2 * (uc(3:end) - 2*uc(2:end-1) + uc(1:end-2));
    up = uc; uc = unext;
end
grid on; xlabel('x'); ylabel('u(x,t)'); title('Wave Equation'); legend;

%% Simulink note
% The same semi-discrete models can be built in Simulink by stacking Integrator
% blocks for the state-space form obtained after spatial discretization.
