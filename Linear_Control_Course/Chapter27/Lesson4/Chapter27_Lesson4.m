% Parameters
omega_n = 10;      % natural frequency
zeta0   = 0.4;     % existing damping
zeta1   = 0.9;     % desired effective damping

s = tf('s');

% Closed-loop from internal reference to output
T0 = omega_n^2 / (s^2 + 2*zeta0*omega_n*s + omega_n^2);

% Prefilter F(s)
F  = (s^2 + 2*zeta0*omega_n*s + omega_n^2) ...
   / (s^2 + 2*zeta1*omega_n*s + omega_n^2);

% Reference-to-output transfer with prefilter
T_ref = F * T0;

% Step responses
figure;
step(T0, T_ref);
legend('Without prefilter', 'With prefilter F(s)');
grid on;
title('Command shaping via prefilter');

% Cubic reference profile
q_f = 1;   % final position
Tmove = 1; % move duration
t = linspace(0, Tmove, 500);
s_norm = t / Tmove;
q = 3*q_f*s_norm.^2 - 2*q_f*s_norm.^3;

figure;
plot(t, q);
xlabel('Time [s]');
ylabel('q(t)');
title('Cubic reference trajectory');
grid on;

% Simulink implementation:
% - Use a "Transfer Fcn" block for F(s).
% - Use a "Transfer Fcn" block for the plant closed-loop model or the plant
%   and controller separately.
% - Generate q(t) (cubic profile) with a MATLAB Function block or a
%   polynomial block, then feed it through F(s) into the existing servo loop.
