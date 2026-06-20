% Chapter11_Lesson5.m
% LTV controllability Gramian using MATLAB ode45.
% The same ODE can be implemented in Simulink by using a MATLAB Function
% block for A(t), B(t) and an Integrator block for the vectorized W dynamics.

clear; clc;
t0 = 0.0;
tf = 4.0;
n = 2;

Phi0 = eye(n);
W0 = zeros(n);
z0 = [Phi0(:); W0(:)];
opts = odeset('RelTol',1e-9,'AbsTol',1e-11);
[t,z] = ode45(@(t,z) augmented_ode(t,z,n), [t0 tf], z0, opts);

zf = z(end,:).';
Phi = reshape(zf(1:n*n), n, n);
Wc = reshape(zf(n*n+1:end), n, n);
Wc = 0.5*(Wc + Wc.');

disp('Phi(tf,t0) =');
disp(Phi);
disp('Wc(t0,tf) =');
disp(Wc);
disp('eig(Wc) =');
disp(eig(Wc));

if min(eig(Wc)) > 1e-8
    disp('The LTV system is controllable on this interval.');
else
    disp('The test is singular or ill-conditioned on this interval.');
end

function dz = augmented_ode(t,z,n)
    Phi = reshape(z(1:n*n), n, n);
    W = reshape(z(n*n+1:end), n, n);
    At = A_ltv(t);
    Bt = B_ltv(t);
    dPhi = At*Phi;
    dW = At*W + W*At.' + Bt*Bt.';
    dz = [dPhi(:); dW(:)];
end

function At = A_ltv(t)
    At = [0, 1; -(2.0 + 0.60*sin(t)), -0.25];
end

function Bt = B_ltv(t)
    Bt = [0; 1.0 + 0.45*cos(t)];
end
