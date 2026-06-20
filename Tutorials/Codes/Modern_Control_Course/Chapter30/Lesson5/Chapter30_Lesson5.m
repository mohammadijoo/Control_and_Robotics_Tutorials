% Chapter30_Lesson5.m
% Bridge lab for Modern Control, Chapter 30 Lesson 5.
%
% Requires for full functionality:
%   Control System Toolbox
% Optional:
%   Simulink, Robust Control Toolbox, Model Predictive Control Toolbox
%
% Run:
%   Chapter30_Lesson5

clear; clc;

A = [0 1; 0 0];
B = [0; 1];
C = [1 0];
D = 0;

Q = diag([10 1]);
R = 0.25;

fprintf('--- Pole placement ---\n');
Kpp = place(A, B, [-2 -3]);
disp('Kpp ='); disp(Kpp);
disp('eig(A-B*Kpp) ='); disp(eig(A-B*Kpp).');

fprintf('\n--- LQR ---\n');
[Klqr, P, polesLqr] = lqr(A, B, Q, R);
disp('Klqr ='); disp(Klqr);
disp('Riccati matrix P ='); disp(P);
disp('LQR closed-loop poles ='); disp(polesLqr.');

fprintf('\n--- Observer design by duality ---\n');
L = place(A', C', [-8 -9])';
disp('L ='); disp(L);
disp('eig(A-L*C) ='); disp(eig(A-L*C).');

fprintf('\n--- Separation principle check ---\n');
Asep = [A-B*Klqr, B*Klqr; zeros(2), A-L*C];
disp('eig(separation matrix) ='); disp(eig(Asep).');

fprintf('\n--- Robustness preview: scalar structured uncertainty ---\n');
E = [0 0; 1 0];
for delta = -4:0.5:4
    lam = eig(A + delta*E - B*Klqr);
    maxReal = max(real(lam));
    if maxReal < 0
        status = 'stable';
    else
        status = 'unstable';
    end
    fprintf('delta=%+5.2f  max_real=%+8.4f  %s\n', delta, maxReal, status);
end

fprintf('\n--- Finite-horizon unconstrained MPC/LQR ---\n');
Ts = 0.05;
sysd = c2d(ss(A,B,C,D), Ts);
Ad = sysd.A;
Bd = sysd.B;

Qd = Q;
Rd = R;
Qf = P;
N = 25;

[Pseq, Kseq] = finite_horizon_lqr_local(Ad, Bd, Qd, Rd, Qf, N);
K0 = Kseq(:,:,1);
disp('First MPC gain K0 ='); disp(K0);

x = [1; 0];
cost = 0;
for k = 1:80
    u = -K0*x;
    cost = cost + x'*Qd*x + u'*Rd*u;
    x = Ad*x + Bd*u;
end
disp('Final state after repeated first-gain simulation ='); disp(x.');
fprintf('Approximate accumulated cost = %.6f\n', cost);

fprintf('\n--- Optional Simulink skeleton ---\n');
if license('test','Simulink')
    model = 'Chapter30_Lesson5_Simulink_Observer_LQR';
    if bdIsLoaded(model), close_system(model, 0); end
    new_system(model);
    open_system(model);
    add_block('simulink/Sources/Step', [model '/Initial command']);
    add_block('simulink/Continuous/State-Space', [model '/Plant']);
    add_block('simulink/Sinks/Scope', [model '/Scope']);
    set_param([model '/Plant'], 'A', 'A-B*Klqr', 'B', 'B', 'C', 'C', 'D', 'D');
    add_line(model, 'Initial command/1', 'Plant/1');
    add_line(model, 'Plant/1', 'Scope/1');
    save_system(model);
    fprintf('Created Simulink model: %s.slx\n', model);
else
    fprintf('Simulink license not detected; skipped model generation.\n');
end

function [Pseq, Kseq] = finite_horizon_lqr_local(A, B, Q, R, Qf, N)
    nx = size(A,1);
    nu = size(B,2);
    Pseq = zeros(nx,nx,N+1);
    Kseq = zeros(nu,nx,N);
    Pseq(:,:,N+1) = Qf;
    for k = N:-1:1
        Pnext = Pseq(:,:,k+1);
        S = R + B'*Pnext*B;
        K = S\(B'*Pnext*A);
        Kseq(:,:,k) = K;
        Pseq(:,:,k) = Q + A'*Pnext*(A-B*K);
    end
end
