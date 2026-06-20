% Chapter13_Lesson4.m
% Damping in MDOF Systems and Mode Shapes with Damping
%
% Demonstrates:
% 1) Undamped modes: K*phi = (w^2)*M*phi (generalized eigenproblem)
% 2) Rayleigh damping: C = alpha*M + beta*K -> diagonal modal damping matrix
% 3) Non-proportional damping: state-space eigenanalysis -> complex poles
% 4) Free response simulation with ode45
%
% This is designed as a teaching script (single file).

clear; clc;

% --------------------------
% Example system (3-DOF chain)
% --------------------------
m1 = 1.2; m2 = 1.0; m3 = 0.8;
k1 = 2500; k2 = 1800; k3 = 1200;

M = diag([m1 m2 m3]);
K = [ k1+k2   -k2      0;
      -k2     k2+k3   -k3;
       0      -k3      k3 ];

% --------------------------
% (A) Undamped modal analysis
% --------------------------
% eig(K,M) solves K*V = M*V*D
[V,D] = eig(K,M);
lam = diag(D);              % omega^2
omega = sqrt(lam);

% Mass-normalize: V' M V = I
Phi = V;
for i=1:size(Phi,2)
    mi = Phi(:,i)'*M*Phi(:,i);
    Phi(:,i) = Phi(:,i)/sqrt(mi);
end

disp('Undamped natural frequencies (rad/s):');
disp(omega.');

disp('Check Phi''*M*Phi (should be I):');
disp(Phi'*M*Phi);

disp('Check Phi''*K*Phi (should be diag(omega^2)):');
disp(Phi'*K*Phi);

% ------------------------------------
% (B) Rayleigh damping from two targets
% ------------------------------------
zeta1 = 0.02;   % target at mode 1
zeta3 = 0.05;   % target at mode 3

% alpha + beta*w^2 = 2*zeta*w at two modes
A = [1 omega(1)^2; 1 omega(3)^2];
b = [2*zeta1*omega(1); 2*zeta3*omega(3)];
ab = A\b;
alpha = ab(1); beta = ab(2);

C_ray = alpha*M + beta*K;
Cm_ray = Phi'*C_ray*Phi;

fprintf('Rayleigh coefficients: alpha=%.6f, beta=%.6f\n', alpha, beta);
disp('Modal damping matrix (Rayleigh) Phi''*C*Phi ~ diagonal:');
disp(Cm_ray);

zeta_modal = 0.5*(alpha./omega + beta.*omega);
disp('Modal damping ratios (Rayleigh):');
disp(zeta_modal.');

% ------------------------------------------
% (C) Non-proportional damping (example)
% ------------------------------------------
c13 = 45;  % dashpot between DOF1 and DOF3
c2g = 35;  % dashpot DOF2 to ground
C_np = zeros(3);
C_np(1,1) = C_np(1,1) + c13;
C_np(3,3) = C_np(3,3) + c13;
C_np(1,3) = C_np(1,3) - c13;
C_np(3,1) = C_np(3,1) - c13;
C_np(2,2) = C_np(2,2) + c2g;

disp('Non-proportional damping matrix C_np:');
disp(C_np);

Cm_np = Phi'*C_np*Phi;
disp('Modal damping matrix (non-proportional) Phi''*C_np*Phi (has off-diagonals):');
disp(Cm_np);

% State-space eigenanalysis
n = size(M,1);
Z = zeros(n);
I = eye(n);
Astate = [Z I; -M\K -M\C_np];
lambda = eig(Astate);

% Extract one pole per conjugate pair (imag>0)
lambda_pos = lambda(imag(lambda) > 1e-8);
[~,idx] = sort(imag(lambda_pos));
lambda_pos = lambda_pos(idx);

disp('Complex poles (sigma + j*wd), first three modes:');
for i=1:min(3, numel(lambda_pos))
    sigma = real(lambda_pos(i));
    wd = imag(lambda_pos(i));
    wn = sqrt(sigma^2 + wd^2);
    zeta = -sigma/wn;
    fprintf('Mode %d: sigma=%+.6f, wd=%.6f, wn=%.6f, zeta=%.6f\n', i, sigma, wd, wn, zeta);
end

% ------------------------------------------
% (D) Free response simulation (non-prop)
% ------------------------------------------
x0 = [0.01; 0; 0];
v0 = [0; 0; 0];
z0 = [x0; v0];

tspan = [0 5];
odefun = @(t,z) [ z(n+1:end);
                 -M\(C_np*z(n+1:end) + K*z(1:n)) ];

opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
[t,z] = ode45(odefun, tspan, z0, opts);

x = z(:,1:n);
figure;
plot(t,x,'LineWidth',1.2);
grid on;
xlabel('t [s]');
ylabel('displacement');
title('Non-proportional damping free response (3-DOF)');
legend('x1','x2','x3');
