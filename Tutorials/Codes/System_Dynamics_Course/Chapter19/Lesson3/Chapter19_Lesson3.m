% Chapter19_Lesson3.m
% Finite Difference (FD) and Finite Element (FE) Approximations (Conceptual Level)
%
% This script provides:
%   1) 1D heat equation via FD (method of lines) + theta method in time.
%   2) 1D Poisson -u''=f(x) via linear FEM (Galerkin) on uniform mesh.
%
% MATLAB toolboxes:
%   - Base MATLAB is sufficient for this script.
%   - (Optional) PDE Toolbox offers higher-level FEM routines, but we implement from scratch here.

clear; clc;

%% 1) Heat equation u_t = alpha u_xx on [0,1], Dirichlet u(0)=u(1)=0
L = 1.0;
N = 101;                 % nodes
x = linspace(0, L, N)';
dx = x(2) - x(1);
alpha = 0.05;

u0 = sin(pi*x);
u0(1) = 0.0; u0(end) = 0.0;

% interior size
n = N - 2;

% Laplacian on interior nodes (Dirichlet)
e = ones(n,1);
A = spdiags([e -2*e e], [-1 0 1], n, n) / (dx*dx);

theta = 0.5;             % Crank-Nicolson
dt = 0.4 * (dx*dx) / alpha;
steps = 200;

I = speye(n);
LHS = (I - theta*dt*alpha*A);
RHS = (I + (1-theta)*dt*alpha*A);

uL = u0(1);
uR = u0(end);

% boundary vector for interior equation
bc = zeros(n,1);
bc(1) = bc(1) + uL/(dx*dx);
bc(end) = bc(end) + uR/(dx*dx);

v = u0(2:end-1);

for k = 1:steps
    rhs = RHS*v + dt*alpha*bc;
    v = LHS \ rhs;
end

uHeatFinal = zeros(N,1);
uHeatFinal(1) = uL;
uHeatFinal(end) = uR;
uHeatFinal(2:end-1) = v;

fprintf('Heat FD done. Final time = %.6f\n', steps*dt);

%% 2) Poisson -u'' = f(x) on [0,1], u(0)=u(1)=0, exact solution u=sin(pi x)
Nel = 40;
xf = linspace(0,1,Nel+1)';
h = xf(2) - xf(1);
Nn = Nel + 1;

K = zeros(Nn, Nn);
F = zeros(Nn, 1);

f = @(xx) (pi^2) * sin(pi*xx);

for e = 1:Nel
    i = e;
    j = e + 1;

    Ke = (1/h) * [ 1 -1; -1 1 ];
    xm = 0.5*(xf(i)+xf(j));
    fe = f(xm);
    Fe = fe*(h/2)*[1;1];

    K(i:i+1, i:i+1) = K(i:i+1, i:i+1) + Ke;
    F(i:i+1) = F(i:i+1) + Fe;
end

% Dirichlet eliminate nodes 1 and Nn
free = 2:Nn-1;
u0_bc = 0.0; uL_bc = 0.0;

Fmod = F - K(:,1)*u0_bc - K(:,end)*uL_bc;
Kff = K(free, free);
Ff  = Fmod(free);

uf = Kff \ Ff;

uFEM = zeros(Nn,1);
uFEM(1) = u0_bc; uFEM(end) = uL_bc;
uFEM(free) = uf;

uExact = sin(pi*xf);
errInf = norm(uFEM - uExact, inf);
fprintf('FEM Poisson max error = %.6e\n', errInf);

%% (Optional) plots
figure; plot(x, uHeatFinal, 'LineWidth', 1.5); grid on;
xlabel('x'); ylabel('u'); title('Heat equation (FD space + theta time)');

figure; plot(xf, uFEM, 'o-', xf, uExact, '--', 'LineWidth', 1.5); grid on;
xlabel('x'); ylabel('u'); title('Poisson equation (linear FEM)'); legend('FEM','Exact');

%% Note on Simulink (conceptual):
% If you form the MOL ODE for interior nodes: v_dot = alpha*A*v + alpha*bc,
% you can implement it in Simulink as:
%   - Integrator block for state v
%   - Matrix Gain block with alpha*A feeding back
%   - Constant block alpha*bc as input
% This yields a large state-space model representing the distributed PDE.
