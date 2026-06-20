% Parameters for a DC-motor-driven joint
J = 0.01;  % inertia
B = 0.1;   % viscous friction
K = 1.0;   % motor gain

s = tf('s');
P = K / (J*s + B);

Kp = 5.0;
Ki = 10.0;
C = Kp + Ki/s;           % PI controller

L = C*P;
S = feedback(1, L);      % S(s) = 1 / (1 + L)
Gd = P*S;                % disturbance at plant input -> output

figure;
bodemag(S);
grid on;
title('Sensitivity S(s)');

figure;
bodemag(Gd);
grid on;
title('Disturbance-to-output P(s) S(s)');

% Time-domain disturbance rejection: step disturbance at plant input
t = 0:0.001:5;
[y_d, t_d] = step(Gd, t);
figure;
plot(t_d, y_d);
xlabel('Time [s]');
ylabel('Output y(t) due to unit disturbance');
grid on;

% For Simulink:
% - Use Transfer Fcn blocks for P(s) and C(s),
% - connect them in a feedback loop,
% - inject a Step block as an additive disturbance at the plant input,
% - log the output to compare with the MATLAB response above.
