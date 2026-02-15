% Plant: G(s) = 1 / (M s^2 + B s + K)
M = 1; B = 2; K = 20;
G = tf(1, [M B K]);

% Baseline PD controller
Kp = 40; Kd = 8;
C0 = tf([Kd Kp], [1 0]);

% First-order low-pass filter F(s) = wf / (s + wf)
wf = 50;
F = tf(wf, [1 wf]);

% Case 1: measurement-path filter
% Closed-loop from r to y with F in feedback path:
T_meas = feedback(G*C0, F);   % y/r

% Case 2: controller-path filter
% Closed-loop from r to y with F cascaded with C0:
T_ctrl = feedback(G*C0*F, 1); % y/r

figure;
bodemag(T_meas, T_ctrl)
legend('Measurement-path filter','Controller-path filter')
grid on

% Simulink note:
% In Simulink, build two subsystems:
% - One where the encoder signal passes through a Transfer Fcn block F(s) before feedback.
% - One where F(s) is in series with the controller block.
% Then compare step responses and noise response using "Band-Limited White Noise" blocks.
