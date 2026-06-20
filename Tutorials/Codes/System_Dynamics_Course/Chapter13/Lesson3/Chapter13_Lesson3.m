% Chapter13_Lesson3.m
% System Dynamics (Control Engineering) — Chapter 13, Lesson 3
% Modal Coordinates and Decoupling of MDOF Systems (Undamped Case)
%
% This script:
% 1) Builds a 3-DOF mass-spring chain (M, K)
% 2) Solves generalized EVP: K*phi = lambda*M*phi
% 3) Mass-normalizes: Phi'*M*Phi = I, Phi'*K*Phi = Omega^2
% 4) Simulates forced response in modal coordinates (decoupled ODEs)
% 5) Builds a simple Simulink model programmatically (optional)

clear; clc;

%% 1) Model
m1 = 1.0; m2 = 1.2; m3 = 0.9;
k1 = 1200; k2 = 900; k3 = 700; k4 = 1100;

M = diag([m1 m2 m3]);
K = [k1+k2, -k2,   0;
     -k2,  k2+k3, -k3;
      0,    -k3,  k3+k4];

%% 2) Modal decomposition (symmetric generalized eigenproblem)
[Phi,Lam] = eig(K,M);               % columns: eigenvectors
lam = diag(Lam);
[lam, idx] = sort(lam,'ascend');
Phi = Phi(:,idx);

omega = sqrt(max(lam,0));

% Mass-normalize columns: Phi'*M*Phi = I
for i=1:size(Phi,2)
    mi = Phi(:,i)'*M*Phi(:,i);
    Phi(:,i) = Phi(:,i)/sqrt(mi);
end

Omega2 = Phi'*K*Phi;
disp('Natural frequencies (rad/s):'); disp(omega');
disp('Check Phi''*M*Phi:'); disp(Phi'*M*Phi);
disp('Check Phi''*K*Phi:'); disp(Omega2);

%% 3) Forcing and decoupled simulation
F0 = 10;
Omega = 0.9*omega(1);

f = @(t)[F0*sin(Omega*t); 0; 0];      % physical force
p = @(t)Phi'*f(t);                     % modal force (mass-normalized)

% Modal ODE: qdd + diag(omega.^2) q = p(t)
n = 3;
ode = @(t,z)[ z(n+1:end);
             - (omega.^2).*z(1:n) + p(t) ];

tspan = [0 8];
z0 = zeros(2*n,1); % [q0; qd0]

opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
[t,z] = ode45(ode,tspan,z0,opts);
q = z(:,1:n).';
x = Phi*q; % x(t) = Phi q(t)

figure; plot(t,x(1,:), 'LineWidth',1.2); grid on;
xlabel('t [s]'); ylabel('x_1 [m]'); title('Displacement of DOF 1 via modal simulation');

figure; plot(t,q.', 'LineWidth',1.2); grid on;
xlabel('t [s]'); ylabel('q_i'); legend('q_1','q_2','q_3');
title('Modal coordinates');

%% 4) Optional: build a Simulink model programmatically
% This creates a state-space block for the *decoupled* modal system:
%   [q; qd]' = A*[q; qd] + B*u,  with u(t) = sin(Omega t)
%   y = x1 = e1^T * Phi * q

make_simulink = true;
if make_simulink
    mdl = 'Chapter13_Lesson3_Simulink';
    if bdIsLoaded(mdl); close_system(mdl,0); end
    new_system(mdl); open_system(mdl);

    % State-space matrices
    A = [zeros(n), eye(n);
         -diag(omega.^2), zeros(n)];
    B = [zeros(n,1); Phi.'*[F0;0;0]];  % modal forcing gain for u=sin(Omega t)
    C = [ [1 0 0]*Phi, zeros(1,n) ];   % x1 depends only on q
    D = 0;

    % Add blocks
    add_block('simulink/Sources/Sine Wave',[mdl '/Sine'], 'Amplitude','1', 'Frequency',num2str(Omega));
    add_block('simulink/Continuous/State-Space',[mdl '/ModalSS']);
    set_param([mdl '/ModalSS'],'A','A','B','B','C','C','D','D');

    add_block('simulink/Sinks/Scope',[mdl '/Scope']);

    % Connect
    add_line(mdl,'Sine/1','ModalSS/1');
    add_line(mdl,'ModalSS/1','Scope/1');

    % Push matrices to base workspace so the block can see them
    assignin('base','A',A); assignin('base','B',B);
    assignin('base','C',C); assignin('base','D',D);

    set_param(mdl,'StopTime','8');
    save_system(mdl);
    disp(['Simulink model created: ' mdl '.slx']);
end
