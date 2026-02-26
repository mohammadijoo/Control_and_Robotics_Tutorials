J = 0.01;
B = 0.1;

s = tf('s');
G = 1 / (J*s^2 + B*s);       % robot joint model

Kp = 30; Ki = 80;
C_low = Kp + Ki/s;           % PI

wh = 200;                    % rolloff corner
F2 = 1 / (1 + s/wh)^2;       % 2nd-order rolloff
C = C_low * F2;

L = C*G;

figure; bodemag(L)
grid on
title('Open-loop with high-frequency rolloff')

Tny = -L / (1 + L);          % noise -> output

% Simulate measurement noise
t = linspace(0,1,5000);
dt = t(2)-t(1);
sigma_n = 0.01;
n = sigma_n*randn(size(t));

y = lsim(Tny, n, t);

figure;
plot(t, y);
xlabel('time (s)');
ylabel('y due to measurement noise');
grid on
title('Output noise with rolloff')

% In Simulink:
%   - Use a Plant block with transfer function G
%   - Implement C using a PID Controller block with derivative filter
%   - Add extra low-pass filters to controller output or measurement line
%   - Use 'Bode Plot' or 'Linear Analysis Tool' to inspect loop shape
