% Chapter19_Lesson4.m
% System Dynamics — Chapter 19, Lesson 4
% Transport delay models and pseudo-distributed approximations.
%
% Demonstrates:
%  (1) Gd(s) = exp(-sT)
%  (2) Padé approximation (built-in)
%  (3) Stable N-lag chain approximation: (1/(1+sT/N))^N
%  (4) Step response of Gp(s)=1/(s+1) with delay.

clear; clc;

T = 1.0;          % delay [s]
nPade = 2;        % Padé order
Nchain = 8;       % chain order

s = tf('s');
Gp = 1/(s+1);

% Padé approximation of exp(-sT)
[numP, denP] = pade(T, nPade);
GdPade = tf(numP, denP);

% Stable pseudo-distributed chain approximation
GdChain = 1;
for k = 1:Nchain
    GdChain = series(GdChain, 1/(1 + s*(T/Nchain)));
end

% Combine with plant
G_pade  = series(Gp, GdPade);
G_chain = series(Gp, GdChain);

% 'Ideal' delay for comparison (MATLAB handles delays symbolically for some ops)
G_ideal = Gp;
G_ideal.InputDelay = T;

t = linspace(0, 12, 2000);

figure;
step(G_ideal, G_pade, G_chain, t);
grid on;
legend('Ideal delay','Padé','Chain');
title('First-order plant with transport delay: step response');

% Frequency-domain: delay phase
w = logspace(-2, 2, 800);
[magI, phI] = bode(tf(1,1,'InputDelay',T), w); %#ok<ASGLU>
[magP, phP] = bode(GdPade, w); %#ok<ASGLU>
[magC, phC] = bode(GdChain, w); %#ok<ASGLU>

figure;
semilogx(w, squeeze(phI), w, squeeze(phP), w, squeeze(phC));
grid on;
xlabel('\omega [rad/s]'); ylabel('Phase [deg]');
legend('Ideal delay','Padé','Chain');
title('Delay phase: ideal vs approximations');

% Discrete-time exact delay via buffer (optional)
dt = 0.001;
K = round(T/dt);
buf = zeros(K+1,1);
y = 0;
a = 1; b = 1;
tEnd = 8;
steps = round(tEnd/dt);

for k = 1:steps
    u = 1;                 % step
    uDel = buf(1);
    buf(1:end-1) = buf(2:end);
    buf(end) = u;
    y = y + dt*(-a*y + b*uDel);
end
disp('Done.');
