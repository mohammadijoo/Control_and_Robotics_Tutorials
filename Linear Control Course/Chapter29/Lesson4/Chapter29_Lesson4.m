% Physical parameters
m  = 1000;   % kg
b  = 50;     % N*s/m
Ku = 500;    % N per control unit

% Desired design parameters (example)
zeta = 0.6;
Ts   = 5.0;
wn   = 4/(zeta*Ts);

Kp = (2*zeta*wn*m - b)/Ku;
Ki = (wn^2*m)/Ku;

fprintf('PI gains: Kp = %.3f, Ki = %.3f\n', Kp, Ki);

% Plant G(s) = Ku / (m s + b)
s = tf('s');
G = Ku / (m*s + b);

% PI controller C(s) = Kp + Ki/s
C = Kp + Ki/s;

% Closed-loop with unity feedback
T = feedback(C*G, 1);

% Step response (1 m/s step)
figure;
step(T);
grid on;
title('Cruise Control Closed-Loop Step Response (PI)');
ylabel('Speed [m/s]');
xlabel('Time [s]');

% --- Simulink notes (conceptual) ---
% In Simulink, one can create a block diagram:
% - Use a Step block as reference r(t)
% - Sum block to compute e(t) = r(t) - v(t)
% - PI controller implemented via Gain (Kp) and Integrator blocks (Ki*1/s)
% - First-order Transfer Fcn block for G(s) = Ku/(m*s + b)
% - Feedback connection from output v(t) to the Sum block
